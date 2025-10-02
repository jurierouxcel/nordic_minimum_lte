#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <string.h>
#include <nrf_modem.h>
#include <nrf_modem_at.h>
#include <nrf_modem_gnss.h>
#include <modem/nrf_modem_lib.h>

#define SLEEP_TIME_MS   1000

// The devicetree node identifier for the LEDs.
#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)
#define LED2_NODE DT_ALIAS(led2)
#define LED3_NODE DT_ALIAS(led3)

// Get the LED hardware information.
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(LED2_NODE, gpios);
static const struct gpio_dt_spec led3 = GPIO_DT_SPEC_GET(LED3_NODE, gpios);

NRF_MODEM_LIB_ON_INIT(lwm2m_init_hook, on_modem_lib_init, NULL);

static void on_modem_lib_init(int ret, void *ctx)
{
	ARG_UNUSED(ctx);

	printf("on modem init()\n");

	printf("Sending: AT+CGMR\n");
	int err;
	char buf[64];
	buf[0] = 0;
	err = nrf_modem_at_cmd(buf, sizeof(buf), "AT+CGMR");
	if (err)
	{
		printf("Failed to get modem firmware type, error: %d\n", err);
		return;
	}

	if (strnlen(&buf[0], 64) < 64)
	{
		printf("Received: %s\n", &buf[0]);
	}
}

int main(void)
{
	int ret0;
	int ret1;
	int ret2;
	int ret3;

	// Validate the GPIO pin for the LEDs.
	if (!gpio_is_ready_dt(&led0) || !gpio_is_ready_dt(&led1) ||
		!gpio_is_ready_dt(&led2) || !gpio_is_ready_dt(&led3))
	{
		return 0;
	}

	// Configure LEDs.
	bool led_state = true;
	ret0 = gpio_pin_configure_dt(&led0, GPIO_OUTPUT_ACTIVE);
	ret1 = gpio_pin_configure_dt(&led1, GPIO_OUTPUT_ACTIVE);
	ret2 = gpio_pin_configure_dt(&led2, GPIO_OUTPUT_ACTIVE);
	ret3 = gpio_pin_configure_dt(&led3, GPIO_OUTPUT_ACTIVE);
	if ((ret0 < 0) || (ret1 < 0) || (ret2 < 0) || (ret3 < 0))
	{
		return 0;
	}

	int ret = nrf_modem_lib_init();
	if (ret)
	{
		printf("Modem library init failed, err: %d", ret);
	}

	while (1)
	{
		// Toggle LED 0.
		ret0 = gpio_pin_toggle_dt(&led0);
		ret1 = gpio_pin_toggle_dt(&led1);
		ret2 = gpio_pin_toggle_dt(&led2);
		ret3 = gpio_pin_toggle_dt(&led3);
		if ((ret0 < 0) || (ret1 < 0) || (ret2 < 0) || (ret3 < 0))
		{
			return 0;
		}

		// Output UART.
		led_state = !led_state;
		printf("LED state: %s\n", led_state ? "ON" : "OFF");

		// Sleep.
		k_msleep(SLEEP_TIME_MS);
	}

	return 0;
}
