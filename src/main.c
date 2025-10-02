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
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <nrfx_timer.h>
#include <zephyr/irq.h>

// Sleep time.
#define SLEEP_TIME_MS   1000

// Timer instance.
#define TIMER_INST_IDX 0
// Timer time.
#define TIME_TO_WAIT_MS 5000UL

// Flash partition.
#define TEST_PARTITION storage_partition
#define TEST_PARTITION_OFFSET FIXED_PARTITION_OFFSET(TEST_PARTITION)
#define TEST_PARTITION_DEVICE FIXED_PARTITION_DEVICE(TEST_PARTITION)
#define FLASH_PAGE_SIZE 4096

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

nrfx_timer_t timer_inst = NRFX_TIMER_INSTANCE(TIMER_INST_IDX);;

static void timer_handler(nrf_timer_event_t event_type, void * p_context)
{
    if (event_type == NRF_TIMER_EVENT_COMPARE0)
    {
        char * p_msg = p_context;
        printf("Timer finished. Context passed to the handler: >%s<\n", p_msg);
    }
}

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

void test_timer()
{
	// Configure timer.
	IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_TIMER_INST_GET(TIMER_INST_IDX)), IRQ_PRIO_LOWEST,
                NRFX_TIMER_INST_HANDLER_GET(TIMER_INST_IDX), 0, 0);
	// timer_inst = NRFX_TIMER_INSTANCE(TIMER_INST_IDX);
	uint32_t base_frequency = NRF_TIMER_BASE_FREQUENCY_GET(timer_inst.p_reg);
    nrfx_timer_config_t config = NRFX_TIMER_DEFAULT_CONFIG(base_frequency);
    config.bit_width = NRF_TIMER_BIT_WIDTH_32;
    config.p_context = "Some context";

	nrfx_err_t status = nrfx_timer_init(&timer_inst, &config, timer_handler);
    if (status != NRFX_SUCCESS)
	{
		printf("Failed to init timer.\n");
	}

	nrfx_timer_clear(&timer_inst);

	// Creating variable desired_ticks to store the output of nrfx_timer_ms_to_ticks function.
    uint32_t desired_ticks = nrfx_timer_ms_to_ticks(&timer_inst, TIME_TO_WAIT_MS);
    printf("Time to wait: %lu ms.\n", TIME_TO_WAIT_MS);

	/*
     * Set the timer channel 0 in the extended compare mode to clear and repeat the timer and
     * trigger an interrupt if internal counter register is equal to desired_ticks.
     */
    nrfx_timer_extended_compare(&timer_inst, NRF_TIMER_CC_CHANNEL0, desired_ticks,
                                NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);

	nrfx_timer_enable(&timer_inst);
}

void test_flash()
{
	const struct device *flash_dev = TEST_PARTITION_DEVICE;
	struct flash_parameters flash_params;

	memcpy(&flash_params, flash_get_parameters(flash_dev), sizeof(flash_params));

	printf("Write data to internal flash.\n");

	if (!device_is_ready(flash_dev))
	{
		printf("Internal storage device not ready\n");
		return;
	}

	// Erase flash.
	uint32_t offset = TEST_PARTITION_OFFSET;
	if (flash_erase(flash_dev, offset, FLASH_PAGE_SIZE) != 0)
	{
		printf("Erase failed!\n");
	}
	else
	{
		printf("Erase succeeded!\n");
	}

	uint8_t data[4] = {1, 2, 3, 4};
	// Write data to flash.
	if (flash_write(flash_dev, TEST_PARTITION_OFFSET, &data[0], 4) != 0)
	{
		printf("Write failed!\n");
		return;
	}

	// Read data from flash.
	uint8_t data_read[4];
	printf("Reading from flash.\n");
	if (flash_read(flash_dev, TEST_PARTITION_OFFSET, &data_read[0], 4) != 0)
	{
		printf("Read failed!\n");
		return;
	}

	// Compare read data.
	if (memcmp(&data[0], &data_read[0], 4))
	{
		printf("Data read does not match data written!\n");
	}
	else
	{
		printf("Data read matches data written. Good!\n");
	}
}

int main(void)
{
	int ret0;
	int ret1;
	int ret2;
	int ret3;

	test_timer();

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

	test_flash();

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
