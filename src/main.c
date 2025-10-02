#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <string.h>
#include <nrf_modem.h>
#include <nrf_modem_at.h>
#include <nrf_modem_gnss.h>
#include <modem/nrf_modem_lib.h>
#include <modem/lte_lc.h>

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

/**
 * The GNSS event handler is called in interrupt service routine context.
 */
static void gnss_event_handler(int event_id)
{
	printf("GNSS event received.\n");

    int err;
	struct nrf_modem_gnss_pvt_data_frame pvt_data;

    /* Process event */
    switch (event_id)
	{
    case NRF_MODEM_GNSS_EVT_PVT:
        // Read PVT data.
        err = nrf_modem_gnss_read(&pvt_data, sizeof(pvt_data), NRF_MODEM_GNSS_DATA_PVT);
		printf("Read PVT with long: %f lat: %f\n", pvt_data.latitude, pvt_data.longitude);
		break;
	default:
		break;
    }
}

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

	// Enable LTE-M, NB-IOT and GNSS.
	err = nrf_modem_at_printf("AT%%XSYSTEMMODE=1,1,1,0");
	if (err)
	{
		printf("XSYSTEMMODE could not be set, error: %d\n", err);
		return;
	}

	// Activate GNSS mode. 
	err = nrf_modem_at_printf("AT+CFUN=%d", LTE_LC_FUNC_MODE_ACTIVATE_GNSS);
	if (err)
	{
		printf("Failed to set functional mode. Please check XSYSTEMMODE.\n");
		return;
	}

	err = nrf_modem_gnss_event_handler_set(gnss_event_handler);
	if (err)
	{
		printf("Modem GNSS failed to start, err: %d\n", err);
	}

	/* Enable all supported NMEA messages. */
	uint16_t nmea_mask = NRF_MODEM_GNSS_NMEA_RMC_MASK |
			     NRF_MODEM_GNSS_NMEA_GGA_MASK |
			     NRF_MODEM_GNSS_NMEA_GLL_MASK |
			     NRF_MODEM_GNSS_NMEA_GSA_MASK |
			     NRF_MODEM_GNSS_NMEA_GSV_MASK;

	if (nrf_modem_gnss_nmea_mask_set(nmea_mask) != 0)
	{
		printf("Failed to set GNSS NMEA mask.\n");
		return;
	}

	uint8_t use_case = NRF_MODEM_GNSS_USE_CASE_MULTIPLE_HOT_START ||
		NRF_MODEM_GNSS_USE_CASE_LOW_ACCURACY;
	if (nrf_modem_gnss_use_case_set(use_case) != 0)
	{
		printf("Failed to set GNSS use case.\n");
	}

	// Default to no power saving for GNSS.
	uint8_t power_mode = NRF_MODEM_GNSS_PSM_DISABLED;
	if (nrf_modem_gnss_power_mode_set(power_mode) != 0)
	{
		printf("Failed to set GNSS power saving mode.\n");
		return;
	}

	// Set NGSS retry and reporting interval.
	uint16_t fix_retry = 5;
	uint16_t fix_interval = 1;
	if (nrf_modem_gnss_fix_retry_set(fix_retry) != 0)
	{
		printf("Failed to set GNSS fix retry.\n");
		return;
	}

	if (nrf_modem_gnss_fix_interval_set(fix_interval) != 0)
	{
		printf("Failed to set GNSS fix interval.\n");
		return;
	}

	// Start GNSS.
	ret = nrf_modem_gnss_start();
	if (ret)
	{
		printf("Modem GNSS failed to start, err: %d\n", ret);
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
		printf("Modem library init failed, err: %d\n", ret);
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
