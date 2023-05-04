/*!
 * \file      sx1276-board.c
 *
 * \brief     Target board SX1276 driver implementation
 *
 * \author    ck ( beelinker )
 */
#include "boards.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_spi.h"
#include "nrf_delay.h"
#include "app_error.h"
#include "radio.h"
#include "sx1276-board.h"

#define BOARD_TCXO_WAKEUP_TIME                      1

/*!
 * Radio driver structure initialization
 */
const struct Radio_s Radio =
{
    sx1276_init,
    sx1276_get_status,
    sx1276_set_modem,
    sx1276_set_channel,
    sx1276_is_channel_free,
    sx1276_random,
    sx1276_set_rx_config,
    sx1276_set_tx_config,
    sx1276_check_rf_frequency,
    sx1276_get_time_on_air,
    sx1276_send,
    sx1276_set_sleep,
    sx1276_set_stby,
    sx1276_set_rx,
    sx1276_start_cad,
    sx1276_set_tx_continuous_wave,//FSK
    sx1276_read_rssi,
    sx1276_write,
    sx1276_read,
    sx1276_write_buffer,
    sx1276_read_buffer,
    sx1276_set_max_payload_length,
    sx1276_set_public_network,
    sx1276_get_wakeup_time,
    NULL, // void ( *IrqProcess )( void )
    NULL, // void ( *RxBoosted )( uint32_t timeout ) - SX126x Only
    NULL, // void ( *SetRxDutyCycle )( uint32_t rxTime, uint32_t sleepTime ) - SX126x Only
    radio_spi_init,
    sx1276_io_lowpower_manage,
};

static  bool radio_spi_init_flg = false;
static const nrf_drv_spi_t radio_spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
static nrf_drv_spi_config_t spi_radio_config = NRF_DRV_SPI_DEFAULT_CONFIG;

static volatile bool radio_spi_xfer_done;                                   /**< Flag used to indicate that SPI instance completed the transfer. */

void sx1276_status_io_init(nrfx_gpiote_evt_handler_t st_callback)
{
    ret_code_t err_code;

    ASSERT(st_callback);

    if (!nrf_drv_gpiote_is_init())
    {
        err_code = nrf_drv_gpiote_init();
        APP_ERROR_CHECK(err_code);
    }

	nrf_drv_gpiote_in_config_t inConfig = GPIOTE_CONFIG_IN_SENSE_LOTOHI(false);
	inConfig.pull = NRF_GPIO_PIN_NOPULL;

	err_code = nrf_drv_gpiote_in_init(USER_RADIO_STATUS_PIN, &inConfig, st_callback);
	APP_ERROR_CHECK(err_code);

	nrf_drv_gpiote_in_event_enable(USER_RADIO_STATUS_PIN, true);
}                                         

void sx1276_reset(void)
{
    // Set RESET pin to 0
    nrf_gpio_cfg_output(USER_RADIO_RESET_PIN);
    nrf_gpio_pin_clear(USER_RADIO_RESET_PIN);

    // Wait 1 ms
    nrf_delay_ms(1);

    // Configure RESET as input
    nrf_gpio_cfg_input(USER_RADIO_RESET_PIN, NRF_GPIO_PIN_NOPULL);

    // Wait 6 ms
    nrf_delay_ms(6);
}

uint32_t sx1276_get_dio0pin_state(void)
{
    return nrf_gpio_pin_read(USER_RADIO_STATUS_PIN);
}

bool sx1276_check_rf_frequency( uint32_t frequency )
{
    // Implement check. Currently all frequencies are supported
    return true;
}

uint32_t sx1276_get_board_tcxo_wakeup_time( void )
{
    return BOARD_TCXO_WAKEUP_TIME;
}

/******************** Spi driver code.**************************/
#define DEFAULT_JUMP_NUM  10
/**
 * \brief SPI user event handler.
 * \param event
 */
void radio_spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                             void *                    p_context)
{
    radio_spi_xfer_done = true;
}

void radio_spi_init(void)
{
    if(radio_spi_init_flg)
        return;

    spi_radio_config.ss_pin   = SPI_SS_PIN;
    spi_radio_config.miso_pin = SPI_MISO_PIN;
    spi_radio_config.mosi_pin = SPI_MOSI_PIN;
    spi_radio_config.sck_pin  = SPI_SCK_PIN;
    APP_ERROR_CHECK(nrf_drv_spi_init(&radio_spi, &spi_radio_config, radio_spi_event_handler, NULL));
    radio_spi_init_flg = true;
}

static void radio_spi_uninit(void)
{
    nrfx_spim_uninit(&radio_spi.u.spim);
}

ret_code_t radio_spi_transfer(uint8_t *tx_buf, uint8_t tx_buf_len,
                              uint8_t *rx_buf, uint8_t rx_buf_len)
{
    ret_code_t err = NRF_SUCCESS;

    volatile uint8_t delay_count = 0;

    radio_spi_xfer_done = false;

    err = nrf_drv_spi_transfer(&radio_spi, tx_buf, tx_buf_len, rx_buf, rx_buf_len);
    if(err == NRF_SUCCESS)
    {
        while (!radio_spi_xfer_done)
        {
            nrf_delay_ms(1);
            delay_count ++;
            
            if(delay_count > DEFAULT_JUMP_NUM)
            {
                err = NRF_ERROR_TIMEOUT;
                break;
            }
        }
    }

    return err;
}

/******************** Spi driver end.**************************/
void sx1276_io_lowpower_manage(void)
{
    if(radio_spi_init_flg)
        radio_spi_uninit();

    /* Spi_pin low power manage. */
    nrf_gpio_cfg_input(SPI_MISO_PIN, NRF_GPIO_PIN_NOPULL);
    nrf_gpio_cfg_input(SPI_MOSI_PIN, NRF_GPIO_PIN_PULLUP);
    nrf_gpio_cfg_input(SPI_SCK_PIN, NRF_GPIO_PIN_PULLUP);
    nrf_gpio_cfg_input(SPI_SS_PIN, NRF_GPIO_PIN_PULLDOWN);

    radio_spi_init_flg = false;
}
