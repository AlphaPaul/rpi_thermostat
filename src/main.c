/*
 * Copyright (c) 2019 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Sample echo app for CDC ACM class
 *
 * Sample app for USB CDC ACM class driver. The received data is echoed back
 * to the serial port.
 */

#include <stdio.h>
#include <string.h>

// Generic UART and buffer libraries
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>

// USB Libraries
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/logging/log.h>

// GPIO for LED driving
#include <zephyr/drivers/gpio.h>

// Sensor functions
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/util.h>



//
// GPIO Functions and initializations
//
/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   2000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static struct device * hts221Dev = NULL;
static struct device *uartDev = NULL;


static void uart_send(char* str);


/*
* @function: blinky
* @description: infinite loop blinking the LED. Blocking, other threads must be started or interrupts configured
* @parameters: none
* @return: none
*/
static void blinky(){
	int ret;
	ret = gpio_pin_toggle_dt(&led);
	if (ret < 0) {
		return;
	}

}

static void blinky_init(){
	int ret;

	if (!gpio_is_ready_dt(&led)) {
		return;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return;
	}
}

//
// I2C Functions and initializations
//
static void hts221_init(){
	hts221Dev = DEVICE_DT_GET_ONE(st_hts221);
	if (!device_is_ready(hts221Dev)) {
		uart_send("hts221 device not ready \r\n");
		return;
	}
	else
	{
		uart_send("hts221 device OK \r\n");
	}
	
}

static void hts221_read(){
	char str[64];
	int strIndex = 0;
	struct sensor_value temp, hum;
	if (sensor_sample_fetch(hts221Dev) < 0) {
		uart_send("Sensor sample update error\n");
		return;
	}

	if (sensor_channel_get(hts221Dev, SENSOR_CHAN_AMBIENT_TEMP, &temp) < 0) {
		uart_send("Cannot read HTS221 temperature channel\n");
		return;
	}

	if (sensor_channel_get(hts221Dev, SENSOR_CHAN_HUMIDITY, &hum) < 0) {
		uart_send("Cannot read HTS221 humidity channel\n");
		return;
	}

	/* display temperature */
	strIndex += sprintf(str + strIndex, "T:%d.%d C\r\n", temp.val1, temp.val2/10000);

	/* display humidity */
	sprintf(str + strIndex, "RH:%d.%d%%\r\n", hum.val1, hum.val2/10000);

	uart_send(str);
}


//
// USB and CDC Functions and initializations
//

LOG_MODULE_REGISTER(cdc_acm_echo, LOG_LEVEL_INF);

#define RING_BUF_SIZE 1024
uint8_t ring_buffer[RING_BUF_SIZE];

struct ring_buf ringbuf;

#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
USBD_CONFIGURATION_DEFINE(config_1,
			  USB_SCD_SELF_POWERED,
			  200);

USBD_DESC_LANG_DEFINE(sample_lang);
USBD_DESC_STRING_DEFINE(sample_mfr, "ZEPHYR", 1);
USBD_DESC_STRING_DEFINE(sample_product, "Zephyr USBD CDC ACM", 2);
USBD_DESC_STRING_DEFINE(sample_sn, "0123456789ABCDEF", 3);

USBD_DEVICE_DEFINE(sample_usbd,
		   DEVICE_DT_GET(DT_NODELABEL(zephyr_udc0)),
		   0x2fe3, 0x0001);

static int enable_usb_device_next(void)
{
	int err;

	err = usbd_add_descriptor(&sample_usbd, &sample_lang);
	if (err) {
		LOG_ERR("Failed to initialize language descriptor (%d)", err);
		return err;
	}

	err = usbd_add_descriptor(&sample_usbd, &sample_mfr);
	if (err) {
		LOG_ERR("Failed to initialize manufacturer descriptor (%d)", err);
		return err;
	}

	err = usbd_add_descriptor(&sample_usbd, &sample_product);
	if (err) {
		LOG_ERR("Failed to initialize product descriptor (%d)", err);
		return err;
	}

	err = usbd_add_descriptor(&sample_usbd, &sample_sn);
	if (err) {
		LOG_ERR("Failed to initialize SN descriptor (%d)", err);
		return err;
	}

	err = usbd_add_configuration(&sample_usbd, &config_1);
	if (err) {
		LOG_ERR("Failed to add configuration (%d)", err);
		return err;
	}

	err = usbd_register_class(&sample_usbd, "cdc_acm_0", 1);
	if (err) {
		LOG_ERR("Failed to register CDC ACM class (%d)", err);
		return err;
	}

	err = usbd_init(&sample_usbd);
	if (err) {
		LOG_ERR("Failed to initialize device support");
		return err;
	}

	err = usbd_enable(&sample_usbd);
	if (err) {
		LOG_ERR("Failed to enable device support");
		return err;
	}

	LOG_DBG("USB device support enabled");

	return 0;
}
#endif /* IS_ENABLED(CONFIG_USB_DEVICE_STACK_NEXT) */

static void interrupt_handler(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		if (uart_irq_rx_ready(dev)) {
			int recv_len, rb_len;
			uint8_t buffer[64];
			size_t len = MIN(ring_buf_space_get(&ringbuf),
					 sizeof(buffer));

			recv_len = uart_fifo_read(dev, buffer, len);
			if (recv_len < 0) {
				LOG_ERR("Failed to read UART FIFO");
				recv_len = 0;
			};

			rb_len = ring_buf_put(&ringbuf, buffer, recv_len);
			rb_len += ring_buf_put(&ringbuf, "\r\n--", 4);
			if (rb_len < recv_len) {
				LOG_ERR("Drop %u bytes", recv_len - rb_len);
			}

			LOG_DBG("tty fifo -> ringbuf %d bytes", rb_len);
			if (rb_len) {
				uart_irq_tx_enable(dev);
			}
		}

		if (uart_irq_tx_ready(dev)) {
			uint8_t buffer[64];
			int rb_len, send_len;

			rb_len = ring_buf_get(&ringbuf, buffer, sizeof(buffer));
			if (!rb_len) {
				LOG_DBG("Ring buffer empty, disable TX IRQ");
				uart_irq_tx_disable(dev);
				continue;
			}

			send_len = uart_fifo_fill(dev, buffer, rb_len);
			if (send_len < rb_len) {
				LOG_ERR("Drop %d bytes", rb_len - send_len);
			}

			LOG_DBG("ringbuf -> tty fifo %d bytes", send_len);
		}
	}
}

static void uart_init(){

	uint32_t baudrate, dtr = 0U;
	int ret = 0;
	// Setting up the usb cdc to detect interrupts
	uartDev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);
	if (!device_is_ready(uartDev)) {
		LOG_ERR("CDC ACM device not ready");
		return ;
	}
	#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
		ret = enable_usb_device_next();
	#else
		ret = usb_enable(NULL);
	#endif
	if (ret != 0) {
		LOG_ERR("Failed to enable USB");
		return;
	}
	ring_buf_init(&ringbuf, sizeof(ring_buffer), ring_buffer);

	LOG_INF("Wait for DTR");

	while (true) {
		uart_line_ctrl_get(uartDev, UART_LINE_CTRL_DTR, &dtr);
		if (dtr) {
			break;
		} else {
			/* Give CPU resources to low priority threads. */
			k_sleep(K_MSEC(100));
		}
	}
	LOG_INF("DTR set");


	/* They are optional, we use them to test the interrupt endpoint */
	ret = uart_line_ctrl_set(uartDev, UART_LINE_CTRL_DCD, 1);
	if (ret) {
		LOG_WRN("Failed to set DCD, ret code %d", ret);
	}

	ret = uart_line_ctrl_set(uartDev, UART_LINE_CTRL_DSR, 1);
	if (ret) {
		LOG_WRN("Failed to set DSR, ret code %d", ret);
	}

	/* Wait 100ms for the host to do all settings */
	k_msleep(100);

	ret = uart_line_ctrl_get(uartDev, UART_LINE_CTRL_BAUD_RATE, &baudrate);
	if (ret) {
		LOG_WRN("Failed to get baudrate, ret code %d", ret);
	} else {
		LOG_INF("Baudrate detected: %d", baudrate);
	}

	uart_irq_callback_set(uartDev, interrupt_handler);

	/* Enable rx interrupts */
	uart_irq_rx_enable(uartDev);
}

static void uart_send(char* str){

	int send_len;

	uart_irq_tx_enable(uartDev);

	// while (!uart_irq_tx_ready(uartDev)) {
	// 	k_msleep(100);
	// }

	send_len = uart_fifo_fill(uartDev, str, strlen(str));
	if (send_len < strlen(str)) {
		LOG_ERR("Drop %d bytes", strlen(str) - send_len);
	}

	LOG_DBG("ringbuf -> tty fifo %d bytes", send_len);
	
}


int main(void)
{
	
	uart_init();
	hts221_init();
	blinky_init();

	while(1){
		blinky();
		hts221_read();

		k_msleep(SLEEP_TIME_MS);

	}

	

	return 0;
}
