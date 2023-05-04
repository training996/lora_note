/*!
 * \file      sx1276.c
 *
 * \brief     SX1276 driver implementation
 *
 * \author    ck ( beelinker )
 */
 
#include <math.h>
#include <string.h>
#include "radio.h"
#include "nrf_delay.h"
#include "app_timer.h"
#include "app_error.h"
#include "sx1276.h"
#include "sx1276-board.h"

/*!
 * \brief Internal frequency of the radio
 */
#define SX1276_XTAL_FREQ                            32000000UL

/*!
 * \brief Scaling factor used to perform fixed-point operations
 */
#define SX1276_PLL_STEP_SHIFT_AMOUNT                ( 8 )

/*!
 * \brief PLL step - scaled with SX1276_PLL_STEP_SHIFT_AMOUNT
 */
#define SX1276_PLL_STEP_SCALED                      ( SX1276_XTAL_FREQ >> ( 19 - SX1276_PLL_STEP_SHIFT_AMOUNT ) )

/*!
 * \brief Radio buffer size
 */
#define RX_TX_BUFFER_SIZE                           256

#define RF_TRANSFER_TIMEOUT                         500     //500ms
 
/*!
 * Tx and Rx timers
 */ 
_APP_TIMER_DEF(rx_timeout_timer_id);
_APP_TIMER_DEF(tx_timeout_timer_id);

/*!
 * Radio registers definition
 */
typedef struct
{
    RadioModems_t Modem;
    uint8_t       Addr;
    uint8_t       Value;
}RadioRegisters_t;

/*!
 * FSK bandwidth definition
 */
typedef struct
{
    uint32_t bandwidth;
    uint8_t  RegValue;
}FskBandwidth_t;

static uint8_t       rf_spi_tx_buf[ SPI_TRANSFER_BUF_LEN + 1 ];                /**<Spi TX buffer. */
static uint8_t       rf_spi_rx_buf[ SPI_TRANSFER_BUF_LEN + 1 ];                /**<Spi RX buffer. */
/*
 * Private functions prototypes
 */

/*!
 * Performs the Rx chain calibration for LF and HF bands
 * \remark Must be called just after the reset so all registers are at their
 *         default values
 */
static void rx_chain_calibration( void );

/*!
 * \brief Sets the SX1276 in transmission mode for the given time
 * \param [IN] timeout Transmission timeout [ms] [0: continuous, others timeout]
 */
static void sx1276_set_tx( uint32_t timeout );

/*!
 * \brief Writes the buffer contents to the SX1276 FIFO
 *
 * \param [IN] buffer Buffer containing data to be put on the FIFO.
 * \param [IN] size Number of bytes to be written to the FIFO
 */
static void sx1276_write_fifo( uint8_t *buffer, uint8_t size );

/*!
 * \brief Reads the contents of the SX1276 FIFO
 *
 * \param [OUT] buffer Buffer where to copy the FIFO read data.
 * \param [IN] size Number of bytes to be read from the FIFO
 */
static void sx1276_read_fifo( uint8_t *buffer, uint8_t size );

/*!
 * \brief Sets the SX1276 operating mode
 *
 * \param [IN] opMode New operating mode
 */
static void sx1276_set_opmode( uint8_t opMode );

/*!
 * \brief Get frequency in Hertz for a given number of PLL steps
 *
 * \param [in] pllSteps Number of PLL steps
 *
 * \returns Frequency in Hertz
 */
static uint32_t sx1276_convert_pll_step_to_freq_in_hz( uint32_t pllSteps );

/*!
 * \brief Get the number of PLL steps for a given frequency in Hertz
 *
 * \param [in] freqInHz Frequency in Hertz
 *
 * \returns Number of PLL steps
 */
static uint32_t sx1276_convert_freq_in_hz_to_pll_step( uint32_t freqInHz );

/*!
 * \brief Get the parameter corresponding to a FSK Rx bandwith immediately above the minimum requested one.
 *
 * \param [in] bw Minimum required bandwith in Hz
 *
 * \returns parameter
 */
static uint8_t get_fsk_bandwidth_reg_value( uint32_t bw );

/*!
 * \brief Get the actual value in Hertz of a given LoRa bandwidth
 *
 * \param [in] bw LoRa bandwidth parameter
 *
 * \returns Actual LoRa bandwidth in Hertz
 */
static uint32_t sx1276_get_lora_bandwidth_in_hz( uint32_t bw );

/*!
 * Compute the numerator for LoRa time-on-air computation.
 *
 * \remark To get the actual time-on-air in second, this value has to be divided by the LoRa bandwidth in Hertz.
 *
 * \param [in] bandwidth 
 * \param [in] datarate 
 * \param [in] coderate 
 * \param [in] preambleLen 
 * \param [in] fixLen 
 * \param [in] payloadLen 
 * \param [in] crcOn 
 *
 * \returns LoRa time-on-air numerator
 */
static uint32_t sx1276_get_lora_time_on_air_numerator( uint32_t bandwidth,
                              uint32_t datarate, uint8_t coderate,
                              uint16_t preambleLen, bool fixLen, uint8_t payloadLen,
                              bool crcOn );

/*
 * SX1276 DIO IRQ callback functions prototype
 */

/*!
 * \brief DIO 0 IRQ callback
 */
static void sx1276_ondio0_irq( nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action );

/*!
 * \brief Tx & Rx timeout timer callback
 */
static void sx1276_ontimeout_irq( void* context );

/*!
 * \brief Sets the radio output power.
 *
 * \param [IN] power Sets the RF output power
 */                         
void sx1276_set_rf_tx_power( int8_t power );
/*
 * Private global constants
 */

/*!
 * Radio hardware registers initialization
 *
 * \remark RADIO_INIT_REGISTERS_VALUE is defined in sx1276-board.h file
 */
const RadioRegisters_t RadioRegsInit[] = RADIO_INIT_REGISTERS_VALUE;

/*!
 * Constant values need to compute the RSSI value
 */
#define RSSI_OFFSET_LF                              -164
#define RSSI_OFFSET_HF                              -157

/*!
 * Precomputed FSK bandwidth registers values
 */
const FskBandwidth_t FskBandwidths[] =
{
    { 2600  , 0x17 },
    { 3100  , 0x0F },
    { 3900  , 0x07 },
    { 5200  , 0x16 },
    { 6300  , 0x0E },
    { 7800  , 0x06 },
    { 10400 , 0x15 },
    { 12500 , 0x0D },
    { 15600 , 0x05 },
    { 20800 , 0x14 },
    { 25000 , 0x0C },
    { 31300 , 0x04 },
    { 41700 , 0x13 },
    { 50000 , 0x0B },
    { 62500 , 0x03 },
    { 83333 , 0x12 },
    { 100000, 0x0A },
    { 125000, 0x02 },
    { 166700, 0x11 },
    { 200000, 0x09 },
    { 250000, 0x01 },
    { 300000, 0x00 }, // Invalid Bandwidth
};

/*
 * Private global variables
 */

/*!
 * Radio callbacks variable
 */
static RadioEvents_t *RadioEvents;

/*!
 * Reception buffer
 */
static uint8_t RxTxBuffer[ RX_TX_BUFFER_SIZE ];

/*
 * Public global variables
 */

/*!
 * Radio hardware and global parameters
 */
SX1276_t SX1276;

/*
 * Radio driver functions implementation
 */

void sx1276_init( RadioEvents_t *events )
{             
    uint8_t i, def_val;

    RadioEvents = events;

    // Initialize driver timeout timers
    app_timer_create(&rx_timeout_timer_id, APP_TIMER_MODE_SINGLE_SHOT, sx1276_ontimeout_irq);
    app_timer_create(&tx_timeout_timer_id, APP_TIMER_MODE_SINGLE_SHOT, sx1276_ontimeout_irq);

    sx1276_reset();
    
    //radio_spi_init();
    
	// REMARK: See SX1276 datasheet for modified default values(0x12).
    def_val = sx1276_read( REG_LR_VERSION );
	if(REGVERSION_DEFAULT != def_val)
        return;        

    rx_chain_calibration( );

    sx1276_set_opmode( RF_OPMODE_SLEEP );

    sx1276_status_io_init( sx1276_ondio0_irq );

    for( i = 0; i < sizeof( RadioRegsInit ) / sizeof( RadioRegisters_t ); i++ )
    {
        sx1276_set_modem( RadioRegsInit[i].Modem );
        sx1276_write( RadioRegsInit[i].Addr, RadioRegsInit[i].Value );
    }

    sx1276_set_modem( MODEM_FSK );

    SX1276.Settings.State = RF_IDLE;
}

RadioState_t sx1276_get_status( void )
{
    return SX1276.Settings.State;
}

void sx1276_set_channel( uint32_t freq )
{
    uint32_t freqInPllSteps = sx1276_convert_freq_in_hz_to_pll_step( freq );

    SX1276.Settings.Channel = freq;

    sx1276_write( REG_FRFMSB, ( uint8_t )( ( freqInPllSteps >> 16 ) & 0xFF ) );
    sx1276_write( REG_FRFMID, ( uint8_t )( ( freqInPllSteps >> 8 ) & 0xFF ) );
    sx1276_write( REG_FRFLSB, ( uint8_t )( freqInPllSteps & 0xFF ) );
}

bool sx1276_is_channel_free( uint32_t freq, uint32_t rxBandwidth, int16_t rssiThresh, uint32_t maxCarrierSenseTime )
{
    bool status = true;
    int16_t rssi = 0;
    
    uint32_t carrierSenseTime_from = 0;
    uint32_t carrierSenseTime_to = 0;

    sx1276_set_sleep( );

    sx1276_set_modem( MODEM_FSK );

    sx1276_set_channel( freq );

    sx1276_write( REG_RXBW, get_fsk_bandwidth_reg_value( rxBandwidth ) );
    sx1276_write( REG_AFCBW, get_fsk_bandwidth_reg_value( rxBandwidth ) );

    sx1276_set_opmode( RF_OPMODE_RECEIVER );

    nrf_delay_ms(1);

    carrierSenseTime_from = app_timer_cnt_get( );
    carrierSenseTime_to = carrierSenseTime_from + maxCarrierSenseTime;

    // Perform carrier sense for maxCarrierSenseTime
    while( app_timer_cnt_diff_compute( carrierSenseTime_to, carrierSenseTime_from ) )
    {
        rssi = sx1276_read_rssi( MODEM_FSK );

        if( rssi > rssiThresh )
        {
            status = false;
            break;
        }
        
        carrierSenseTime_from = app_timer_cnt_get( );
    }
    
    sx1276_set_sleep( );
    
    return status;
}

uint32_t sx1276_random( void )
{
    uint8_t i;
    uint32_t rnd = 0;

    /*
     * Radio setup for random number generation
     */
    // Set LoRa modem ON
    sx1276_set_modem( MODEM_LORA );

    // Disable LoRa modem interrupts
    sx1276_write( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                  RFLR_IRQFLAGS_RXDONE |
                  RFLR_IRQFLAGS_PAYLOADCRCERROR |
                  RFLR_IRQFLAGS_VALIDHEADER |
                  RFLR_IRQFLAGS_TXDONE |
                  RFLR_IRQFLAGS_CADDONE |
                  RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                  RFLR_IRQFLAGS_CADDETECTED );

    // Set radio in continuous reception
    sx1276_set_opmode( RF_OPMODE_RECEIVER );

    for( i = 0; i < 32; i++ )
    {
        nrf_delay_ms(1);
        // Unfiltered RSSI value reading. Only takes the LSB value
        rnd |= ( ( uint32_t )sx1276_read( REG_LR_RSSIWIDEBAND ) & 0x01 ) << i;
    }

    sx1276_set_sleep( );

    return rnd;
}

/*!
 * Performs the Rx chain calibration for LF and HF bands
 * \remark Must be called just after the reset so all registers are at their
 *         default values
 */
static void rx_chain_calibration( void )
{
    uint8_t regPaConfigInitVal;
    uint32_t initialFreq;

    // Save context
    regPaConfigInitVal = sx1276_read( REG_PACONFIG );

    initialFreq = sx1276_convert_pll_step_to_freq_in_hz( ( ( ( uint32_t )sx1276_read( REG_FRFMSB ) << 16 ) |
                                                    ( ( uint32_t )sx1276_read( REG_FRFMID ) << 8 ) |
                                                    ( ( uint32_t )sx1276_read( REG_FRFLSB ) ) ) );

    // Cut the PA just in case, RFO output, power = -1 dBm
    sx1276_write( REG_PACONFIG, 0x00 );

    // Launch Rx chain calibration for LF band
    sx1276_write( REG_IMAGECAL, ( sx1276_read( REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_MASK ) | RF_IMAGECAL_IMAGECAL_START );
    while( ( sx1276_read( REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_RUNNING ) == RF_IMAGECAL_IMAGECAL_RUNNING )
    {
    }

    // Sets a Frequency in HF band
    sx1276_set_channel( 868000000 );

    // Launch Rx chain calibration for HF band
    sx1276_write( REG_IMAGECAL, ( sx1276_read( REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_MASK ) | RF_IMAGECAL_IMAGECAL_START );
    while( ( sx1276_read( REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_RUNNING ) == RF_IMAGECAL_IMAGECAL_RUNNING )
    {
    }

    // Restore context
    sx1276_write( REG_PACONFIG, regPaConfigInitVal );
    sx1276_set_channel( initialFreq );
}

void sx1276_set_rx_config( RadioModems_t modem, uint32_t bandwidth,
                         uint32_t datarate, uint8_t coderate,
                         uint32_t bandwidthAfc, uint16_t preambleLen,
                         uint16_t symbTimeout, bool fixLen,
                         uint8_t payloadLen,
                         bool crcOn, bool freqHopOn, uint8_t hopPeriod,
                         bool iqInverted, bool rxContinuous )
{
    sx1276_set_modem( modem );

    sx1276_set_stby( );

    switch( modem )
    {
        default:
        case MODEM_LORA:
        {
            if( bandwidth > 2 )
            {
                // Fatal error: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
                while( 1 );
            }
            bandwidth += 7;
            SX1276.Settings.LoRa.Bandwidth = bandwidth;
            SX1276.Settings.LoRa.Datarate = datarate;
            SX1276.Settings.LoRa.Coderate = coderate;
            SX1276.Settings.LoRa.PreambleLen = preambleLen;
            SX1276.Settings.LoRa.FixLen = fixLen;
            SX1276.Settings.LoRa.PayloadLen = payloadLen;
            SX1276.Settings.LoRa.CrcOn = crcOn;
            SX1276.Settings.LoRa.FreqHopOn = freqHopOn;
            SX1276.Settings.LoRa.HopPeriod = hopPeriod;
            SX1276.Settings.LoRa.IqInverted = iqInverted;
            SX1276.Settings.LoRa.RxContinuous = rxContinuous;

            if( datarate > 12 )
            {
                datarate = 12;
            }
            else if( datarate < 6 )
            {
                datarate = 6;
            }

            if( ( ( bandwidth == 7 ) && ( ( datarate == 11 ) || ( datarate == 12 ) ) ) ||
                ( ( bandwidth == 8 ) && ( datarate == 12 ) ) )
            {
                SX1276.Settings.LoRa.LowDatarateOptimize = 0x01;
            }
            else
            {
                SX1276.Settings.LoRa.LowDatarateOptimize = 0x00;
            }

            sx1276_write( REG_LR_MODEMCONFIG1,
                         ( sx1276_read( REG_LR_MODEMCONFIG1 ) &
                           RFLR_MODEMCONFIG1_BW_MASK &
                           RFLR_MODEMCONFIG1_CODINGRATE_MASK &
                           RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK ) |
                           ( bandwidth << 4 ) | ( coderate << 1 ) |
                           fixLen );

            sx1276_write( REG_LR_MODEMCONFIG2,
                         ( sx1276_read( REG_LR_MODEMCONFIG2 ) &
                           RFLR_MODEMCONFIG2_SF_MASK &
                           RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK &
                           RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) |
                           ( datarate << 4 ) | ( crcOn << 2 ) |
                           ( ( symbTimeout >> 8 ) & ~RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) );

            sx1276_write( REG_LR_MODEMCONFIG3,
                         ( sx1276_read( REG_LR_MODEMCONFIG3 ) &
                           RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK ) |
                           ( SX1276.Settings.LoRa.LowDatarateOptimize << 3 ) );

            sx1276_write( REG_LR_SYMBTIMEOUTLSB, ( uint8_t )( symbTimeout & 0xFF ) );

            sx1276_write( REG_LR_PREAMBLEMSB, ( uint8_t )( ( preambleLen >> 8 ) & 0xFF ) );
            sx1276_write( REG_LR_PREAMBLELSB, ( uint8_t )( preambleLen & 0xFF ) );

            if( fixLen == 1 )
            {
                sx1276_write( REG_LR_PAYLOADLENGTH, payloadLen );
            }

            if( SX1276.Settings.LoRa.FreqHopOn == true )
            {
                sx1276_write( REG_LR_PLLHOP, ( sx1276_read( REG_LR_PLLHOP ) & RFLR_PLLHOP_FASTHOP_MASK ) | RFLR_PLLHOP_FASTHOP_ON );
                sx1276_write( REG_LR_HOPPERIOD, SX1276.Settings.LoRa.HopPeriod );
            }

            if( ( bandwidth == 9 ) && ( SX1276.Settings.Channel > RF_MID_BAND_THRESH ) )
            {
                // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
                sx1276_write( REG_LR_HIGHBWOPTIMIZE1, 0x02 );
                sx1276_write( REG_LR_HIGHBWOPTIMIZE2, 0x64 );
            }
            else if( bandwidth == 9 )
            {
                // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
                sx1276_write( REG_LR_HIGHBWOPTIMIZE1, 0x02 );
                sx1276_write( REG_LR_HIGHBWOPTIMIZE2, 0x7F );
            }
            else
            {
                // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
                sx1276_write( REG_LR_HIGHBWOPTIMIZE1, 0x03 );
            }

            if( datarate == 6 )
            {
                sx1276_write( REG_LR_DETECTOPTIMIZE,
                             ( sx1276_read( REG_LR_DETECTOPTIMIZE ) &
                               RFLR_DETECTIONOPTIMIZE_MASK ) |
                               RFLR_DETECTIONOPTIMIZE_SF6 );
                sx1276_write( REG_LR_DETECTIONTHRESHOLD,
                             RFLR_DETECTIONTHRESH_SF6 );
            }
            else
            {
                sx1276_write( REG_LR_DETECTOPTIMIZE,
                             ( sx1276_read( REG_LR_DETECTOPTIMIZE ) &
                             RFLR_DETECTIONOPTIMIZE_MASK ) |
                             RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12 );
                sx1276_write( REG_LR_DETECTIONTHRESHOLD,
                             RFLR_DETECTIONTHRESH_SF7_TO_SF12 );
            }
        }
        break;
    }
}

void sx1276_set_tx_config( RadioModems_t modem, int8_t power, uint32_t fdev,
                        uint32_t bandwidth, uint32_t datarate,
                        uint8_t coderate, uint16_t preambleLen,
                        bool fixLen, bool crcOn, bool freqHopOn,
                        uint8_t hopPeriod, bool iqInverted, uint32_t timeout )
{
    sx1276_set_modem( modem );

    sx1276_set_stby( );

    sx1276_set_rf_tx_power( power );
    
    sx1276_write( REG_LR_OCP, 0x3B );

    switch( modem )
    {
        default:
        case MODEM_LORA:
        {
            SX1276.Settings.LoRa.Power = power;
            if( bandwidth > 2 )
            {
                // Fatal error: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
                while( 1 );
            }
            bandwidth += 7;
            SX1276.Settings.LoRa.Bandwidth = bandwidth;
            SX1276.Settings.LoRa.Datarate = datarate;
            SX1276.Settings.LoRa.Coderate = coderate;
            SX1276.Settings.LoRa.PreambleLen = preambleLen;
            SX1276.Settings.LoRa.FixLen = fixLen;
            SX1276.Settings.LoRa.FreqHopOn = freqHopOn;
            SX1276.Settings.LoRa.HopPeriod = hopPeriod;
            SX1276.Settings.LoRa.CrcOn = crcOn;
            SX1276.Settings.LoRa.IqInverted = iqInverted;
            SX1276.Settings.LoRa.TxTimeout = timeout;

            if( datarate > 12 )
            {
                datarate = 12;
            }
            else if( datarate < 6 )
            {
                datarate = 6;
            }
            if( ( ( bandwidth == 7 ) && ( ( datarate == 11 ) || ( datarate == 12 ) ) ) ||
                ( ( bandwidth == 8 ) && ( datarate == 12 ) ) )
            {
                SX1276.Settings.LoRa.LowDatarateOptimize = 0x01;
            }
            else
            {
                SX1276.Settings.LoRa.LowDatarateOptimize = 0x00;
            }

            if( SX1276.Settings.LoRa.FreqHopOn == true )
            {
                sx1276_write( REG_LR_PLLHOP, ( sx1276_read( REG_LR_PLLHOP ) & RFLR_PLLHOP_FASTHOP_MASK ) | RFLR_PLLHOP_FASTHOP_ON );
                sx1276_write( REG_LR_HOPPERIOD, SX1276.Settings.LoRa.HopPeriod );
            }

            sx1276_write( REG_LR_MODEMCONFIG1,
                         ( sx1276_read( REG_LR_MODEMCONFIG1 ) &
                           RFLR_MODEMCONFIG1_BW_MASK &
                           RFLR_MODEMCONFIG1_CODINGRATE_MASK &
                           RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK ) |
                           ( bandwidth << 4 ) | ( coderate << 1 ) |
                           fixLen );

            sx1276_write( REG_LR_MODEMCONFIG2,
                         ( sx1276_read( REG_LR_MODEMCONFIG2 ) &
                           RFLR_MODEMCONFIG2_SF_MASK &
                           RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK ) |
                           ( datarate << 4 ) | ( crcOn << 2 ) );

            sx1276_write( REG_LR_MODEMCONFIG3,
                         ( sx1276_read( REG_LR_MODEMCONFIG3 ) &
                           RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK ) |
                           ( SX1276.Settings.LoRa.LowDatarateOptimize << 3 ) );

            sx1276_write( REG_LR_PREAMBLEMSB, ( preambleLen >> 8 ) & 0x00FF );
            sx1276_write( REG_LR_PREAMBLELSB, preambleLen & 0xFF );

            if( datarate == 6 )
            {
                sx1276_write( REG_LR_DETECTOPTIMIZE,
                             ( sx1276_read( REG_LR_DETECTOPTIMIZE ) &
                               RFLR_DETECTIONOPTIMIZE_MASK ) |
                               RFLR_DETECTIONOPTIMIZE_SF6 );
                sx1276_write( REG_LR_DETECTIONTHRESHOLD,
                             RFLR_DETECTIONTHRESH_SF6 );
            }
            else
            {
                sx1276_write( REG_LR_DETECTOPTIMIZE,
                             ( sx1276_read( REG_LR_DETECTOPTIMIZE ) &
                             RFLR_DETECTIONOPTIMIZE_MASK ) |
                             RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12 );
                sx1276_write( REG_LR_DETECTIONTHRESHOLD,
                             RFLR_DETECTIONTHRESH_SF7_TO_SF12 );
            }
        }
        break;
    }
}

uint32_t sx1276_get_time_on_air( RadioModems_t modem, uint32_t bandwidth,
                              uint32_t datarate, uint8_t coderate,
                              uint16_t preambleLen, bool fixLen, uint8_t payloadLen,
                              bool crcOn )
{
    uint32_t numerator = 0;
    uint32_t denominator = 1;

    switch( modem )
    {
        default:
        case MODEM_LORA:
        {
            numerator   = 1000U * sx1276_get_lora_time_on_air_numerator( bandwidth, datarate, coderate, preambleLen, fixLen,
                                                                   payloadLen, crcOn );
            denominator = sx1276_get_lora_bandwidth_in_hz( bandwidth );
        }
        break;
    }
    // Perform integral ceil()
    return ( numerator + denominator - 1 ) / denominator;
}

void sx1276_send( uint8_t *buffer, uint8_t size )
{
    uint32_t txTimeout = 0;

    switch( SX1276.Settings.Modem )
    {
        default:
        case MODEM_LORA:
        {
            if( SX1276.Settings.LoRa.IqInverted == true )//载波翻转
            {
                sx1276_write( REG_LR_INVERTIQ, ( ( sx1276_read( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_ON ) );
                sx1276_write( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON );
            }
            else
            {
                sx1276_write( REG_LR_INVERTIQ, ( ( sx1276_read( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF ) );
                sx1276_write( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF );
            }

            SX1276.Settings.LoRaPacketHandler.Size = size;

            // Initializes the payload size
            sx1276_write( REG_LR_PAYLOADLENGTH, size );

            // Full buffer used for Tx
            sx1276_write( REG_LR_FIFOTXBASEADDR, 0 );
            sx1276_write( REG_LR_FIFOADDRPTR, 0 );

            // FIFO operations can not take place in Sleep mode
            if( ( sx1276_read( REG_OPMODE ) & ~RF_OPMODE_MASK ) == RF_OPMODE_SLEEP )//后三位不等于0
            {
                sx1276_set_stby( );
                nrf_delay_ms(2);
            }
            
            // Write payload buffer
            sx1276_write_fifo( buffer, size );
            txTimeout = SX1276.Settings.LoRa.TxTimeout;
        }
        break;
    }

    sx1276_set_tx( txTimeout );
}

void sx1276_set_sleep( void )
{
    app_timer_stop( rx_timeout_timer_id );
    app_timer_stop( tx_timeout_timer_id );

    sx1276_set_opmode( RF_OPMODE_SLEEP );

    // Disable TCXO radio is in SLEEP mode
    //SX1276SetBoardTcxo( false );

    SX1276.Settings.State = RF_IDLE;
}

void sx1276_set_stby( void )
{
    app_timer_stop( rx_timeout_timer_id );
    app_timer_stop( tx_timeout_timer_id );

    sx1276_set_opmode( RF_OPMODE_STANDBY );
    SX1276.Settings.State = RF_IDLE;
}

void sx1276_set_rx( uint32_t timeout )
{
    bool rxContinuous = false;
    
    app_timer_stop(rx_timeout_timer_id);

    switch( SX1276.Settings.Modem )
    {
        default:    
        case MODEM_LORA:
        {
            if( SX1276.Settings.LoRa.IqInverted == true )
            {
                sx1276_write( REG_LR_INVERTIQ, ( ( sx1276_read( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_ON | RFLR_INVERTIQ_TX_OFF ) );
                sx1276_write( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON );
            }
            else
            {
                sx1276_write( REG_LR_INVERTIQ, ( ( sx1276_read( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF ) );
                sx1276_write( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF );
            }

            // ERRATA 2.3 - Receiver Spurious Reception of a LoRa Signal
            if( SX1276.Settings.LoRa.Bandwidth < 9 )
            {
                sx1276_write( REG_LR_DETECTOPTIMIZE, sx1276_read( REG_LR_DETECTOPTIMIZE ) & 0x7F );
                sx1276_write( REG_LR_IFFREQ2, 0x00 );
                switch( SX1276.Settings.LoRa.Bandwidth )
                {
                case 0: // 7.8 kHz
                    sx1276_write( REG_LR_IFFREQ1, 0x48 );
                    sx1276_set_channel(SX1276.Settings.Channel + 7810 );
                    break;
                case 1: // 10.4 kHz
                    sx1276_write( REG_LR_IFFREQ1, 0x44 );
                    sx1276_set_channel(SX1276.Settings.Channel + 10420 );
                    break;
                case 2: // 15.6 kHz
                    sx1276_write( REG_LR_IFFREQ1, 0x44 );
                    sx1276_set_channel(SX1276.Settings.Channel + 15620 );
                    break;
                case 3: // 20.8 kHz
                    sx1276_write( REG_LR_IFFREQ1, 0x44 );
                    sx1276_set_channel(SX1276.Settings.Channel + 20830 );
                    break;
                case 4: // 31.2 kHz
                    sx1276_write( REG_LR_IFFREQ1, 0x44 );
                    sx1276_set_channel(SX1276.Settings.Channel + 31250 );
                    break;
                case 5: // 41.4 kHz
                    sx1276_write( REG_LR_IFFREQ1, 0x44 );
                    sx1276_set_channel(SX1276.Settings.Channel + 41670 );
                    break;
                case 6: // 62.5 kHz
                    sx1276_write( REG_LR_IFFREQ1, 0x40 );
                    break;
                case 7: // 125 kHz
                    sx1276_write( REG_LR_IFFREQ1, 0x40 );
                    break;
                case 8: // 250 kHz
                    sx1276_write( REG_LR_IFFREQ1, 0x40 );
                    break;
                }
            }
            else
            {
                sx1276_write( REG_LR_DETECTOPTIMIZE, sx1276_read( REG_LR_DETECTOPTIMIZE ) | 0x80 );
            }

            rxContinuous = SX1276.Settings.LoRa.RxContinuous;

            if( SX1276.Settings.LoRa.FreqHopOn == true )
            {
                sx1276_write( REG_LR_IRQFLAGSMASK, //RFLR_IRQFLAGS_RXTIMEOUT |
                                                  //RFLR_IRQFLAGS_RXDONE |
                                                  //RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                                  RFLR_IRQFLAGS_VALIDHEADER |
                                                  RFLR_IRQFLAGS_TXDONE |
                                                  RFLR_IRQFLAGS_CADDONE |
                                                  //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                                  RFLR_IRQFLAGS_CADDETECTED );

                // DIO0=RxDone, DIO2=FhssChangeChannel
                sx1276_write( REG_DIOMAPPING1, ( sx1276_read( REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK & RFLR_DIOMAPPING1_DIO2_MASK  ) | RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO2_00 );
            }
            else
            {
                sx1276_write( REG_LR_IRQFLAGSMASK, //RFLR_IRQFLAGS_RXTIMEOUT |
                                                  //RFLR_IRQFLAGS_RXDONE |
                                                  //RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                                  RFLR_IRQFLAGS_VALIDHEADER |
                                                  RFLR_IRQFLAGS_TXDONE |
                                                  RFLR_IRQFLAGS_CADDONE |
                                                  RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                                  RFLR_IRQFLAGS_CADDETECTED );

                // DIO0=RxDone
                sx1276_write( REG_DIOMAPPING1, ( sx1276_read( REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK ) | RFLR_DIOMAPPING1_DIO0_00 );
            }
            sx1276_write( REG_LR_FIFORXBASEADDR, 0 );
            sx1276_write( REG_LR_FIFOADDRPTR, 0 );
        }
        break;
    }

    memset( RxTxBuffer, 0, ( size_t )RX_TX_BUFFER_SIZE );

    SX1276.Settings.State = RF_RX_RUNNING;
    if(timeout != 0)
    {
        app_timer_start(rx_timeout_timer_id, APP_TIMER_TICKS(timeout), NULL);  
    }

    if( SX1276.Settings.Modem == MODEM_FSK )
    {
        sx1276_set_opmode( RF_OPMODE_RECEIVER );

        if( rxContinuous == false )
        {
            //set RxTimeoutSyncWord
        }
    }
    else
    {
        if( rxContinuous == true )
        {
            sx1276_set_opmode( RFLR_OPMODE_RECEIVER );
        }
        else
        {
            sx1276_set_opmode( RFLR_OPMODE_RECEIVER_SINGLE );
        }
    }
}

static void sx1276_set_tx( uint32_t timeout )
{
    app_timer_stop( rx_timeout_timer_id );

    switch( SX1276.Settings.Modem )
    {
        default:
        case MODEM_LORA:
        {
            if( SX1276.Settings.LoRa.FreqHopOn == true )
            {
                sx1276_write( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                                                  RFLR_IRQFLAGS_RXDONE |
                                                  RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                                  RFLR_IRQFLAGS_VALIDHEADER |
                                                  //RFLR_IRQFLAGS_TXDONE |
                                                  RFLR_IRQFLAGS_CADDONE |
                                                  //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                                  RFLR_IRQFLAGS_CADDETECTED );

                // DIO0=TxDone, DIO2=FhssChangeChannel
                sx1276_write( REG_DIOMAPPING1, ( sx1276_read( REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK & RFLR_DIOMAPPING1_DIO2_MASK ) | RFLR_DIOMAPPING1_DIO0_01 | RFLR_DIOMAPPING1_DIO2_00 );
            }
            else
            {
                sx1276_write( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                                                  RFLR_IRQFLAGS_RXDONE |
                                                  RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                                  RFLR_IRQFLAGS_VALIDHEADER |
                                                  //RFLR_IRQFLAGS_TXDONE |
                                                  RFLR_IRQFLAGS_CADDONE |
                                                  RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                                  RFLR_IRQFLAGS_CADDETECTED );

                // DIO0=TxDone
                sx1276_write( REG_DIOMAPPING1, ( sx1276_read( REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK ) | RFLR_DIOMAPPING1_DIO0_01 );
            }
        }
        break;
    }

    SX1276.Settings.State = RF_TX_RUNNING;
    
    app_timer_start( tx_timeout_timer_id, APP_TIMER_TICKS(timeout), NULL );
    
    sx1276_set_opmode( RF_OPMODE_TRANSMITTER );
}

void sx1276_start_cad( void )
{
    switch( SX1276.Settings.Modem )
    {
        case MODEM_LORA:
        {
            sx1276_write( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                                        RFLR_IRQFLAGS_RXDONE |
                                        RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                        RFLR_IRQFLAGS_VALIDHEADER |
                                        RFLR_IRQFLAGS_TXDONE |
                                        //RFLR_IRQFLAGS_CADDONE |
                                        RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL // |
                                        //RFLR_IRQFLAGS_CADDETECTED
                                        );

            // DIO3=CADDone
            sx1276_write( REG_DIOMAPPING1, ( sx1276_read( REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO3_MASK ) | RFLR_DIOMAPPING1_DIO3_00 );

            SX1276.Settings.State = RF_CAD;
            sx1276_set_opmode( RFLR_OPMODE_CAD );
        }
        break;
        
        default:
        break;
    }
}

void sx1276_set_tx_continuous_wave( uint32_t freq, int8_t power, uint16_t time )
{
    uint32_t timeout = ( uint32_t )time * 1000;

    sx1276_set_channel( freq );

    sx1276_set_tx_config( MODEM_FSK, power, 0, 0, 4800, 0, 5, false, false, 0, 0, 0, timeout );
		//sx1276_set_tx_config( RadioModems_t modem, int8_t power, uint32_t fdev,uint32_t bandwidth, uint32_t datarate,
	  //uint8_t coderate, uint16_t preambleLen,bool fixLen, bool crcOn, bool freqHopOn,uint8_t hopPeriod, bool iqInverted, uint32_t timeout )

    sx1276_write( REG_PACKETCONFIG2, ( sx1276_read( REG_PACKETCONFIG2 ) & RF_PACKETCONFIG2_DATAMODE_MASK ) );
    // Disable radio interrupts
    sx1276_write( REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_11 | RF_DIOMAPPING1_DIO1_11 );
    sx1276_write( REG_DIOMAPPING2, RF_DIOMAPPING2_DIO4_10 | RF_DIOMAPPING2_DIO5_10 );

    SX1276.Settings.State = RF_TX_RUNNING;
    
    app_timer_start( tx_timeout_timer_id, APP_TIMER_TICKS(timeout), NULL );
    
    sx1276_set_opmode( RF_OPMODE_TRANSMITTER );
}

int16_t sx1276_read_rssi( RadioModems_t modem )
{
    int16_t rssi = 0;

    switch( modem )
    {
        case MODEM_LORA:
            if( SX1276.Settings.Channel > RF_MID_BAND_THRESH )
            {
                rssi = RSSI_OFFSET_HF + sx1276_read( REG_LR_RSSIVALUE );
            }
            else
            {
                rssi = RSSI_OFFSET_LF + sx1276_read( REG_LR_RSSIVALUE );
            }
        break;
            
        default:
            rssi = -1;
        break;
    }
    
    return rssi;
}

static void sx1276_set_opmode( uint8_t opMode )
{
//    if( opMode == RF_OPMODE_SLEEP )
//    {
//        SX1276SetAntSwLowPower( true );
//    }
//    else
//    {
//        // Enable TCXO if operating mode different from SLEEP.
//        SX1276SetBoardTcxo( true );
//        SX1276SetAntSwLowPower( false );
//        SX1276SetAntSw( opMode );
//    }
    sx1276_write( REG_OPMODE, ( sx1276_read( REG_OPMODE ) & RF_OPMODE_MASK ) | opMode );
}

void sx1276_set_modem( RadioModems_t modem )
{
    if( ( sx1276_read( REG_OPMODE ) & RFLR_OPMODE_LONGRANGEMODE_ON ) != 0 )
    {
        SX1276.Settings.Modem = MODEM_LORA;
    }
    else
    {
        SX1276.Settings.Modem = MODEM_FSK;
    }

    if( SX1276.Settings.Modem == modem )
    {
        return;
    }

    SX1276.Settings.Modem = modem;
    
    switch( SX1276.Settings.Modem )
    {
        default:
        case MODEM_LORA:
            sx1276_set_opmode( RF_OPMODE_SLEEP );
            sx1276_write( REG_OPMODE, ( sx1276_read( REG_OPMODE ) & RFLR_OPMODE_LONGRANGEMODE_MASK ) | RFLR_OPMODE_LONGRANGEMODE_ON );

            sx1276_write( REG_DIOMAPPING1, 0x00 );
            sx1276_write( REG_DIOMAPPING2, 0x00 );
        break;
    }
}

void sx1276_set_rf_tx_power(int8_t power)
{
    uint8_t paConfig = 0;
    uint8_t paDac = 0;

    paConfig = sx1276_read( REG_PACONFIG );
    paDac = sx1276_read( REG_PADAC );

    paConfig = ( paConfig & RF_PACONFIG_PASELECT_MASK ) | RF_PACONFIG_PASELECT_PABOOST;

    if( ( paConfig & RF_PACONFIG_PASELECT_PABOOST ) == RF_PACONFIG_PASELECT_PABOOST )
    {
        if( power > 17 )
        {
            paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_ON;
        }
        else
        {
            paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_OFF;
        }
        if( ( paDac & RF_PADAC_20DBM_ON ) == RF_PADAC_20DBM_ON )
        {
            if( power < 5 )
            {
                power = 5;
            }
            if( power > 20 )
            {
                power = 20;
            }
            paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 5 ) & 0x0F );
        }
        else
        {
            if( power < 2 )
            {
                power = 2;
            }
            if( power > 17 )
            {
                power = 17;
            }
            paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 2 ) & 0x0F );
        }
    }
    else
    {
        if( power > 0 )
        {
            if( power > 15 )
            {
                power = 15;
            }
            paConfig = ( paConfig & RF_PACONFIG_MAX_POWER_MASK & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( 7 << 4 ) | ( power );
        }
        else
        {
            if( power < -4 )
            {
                power = -4;
            }
            paConfig = ( paConfig & RF_PACONFIG_MAX_POWER_MASK & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( 0 << 4 ) | ( power + 4 );
        }
    }
    sx1276_write( REG_PACONFIG, paConfig );
    sx1276_write( REG_PADAC, paDac );
}

void sx1276_write( uint32_t addr, uint8_t data )
{
    sx1276_write_buffer( addr, &data, 1 );
}

uint8_t sx1276_read( uint32_t addr )
{
    uint8_t data;
    
    sx1276_read_buffer( addr, &data, 1 );
    return data;
}

void sx1276_write_buffer( uint32_t addr, uint8_t *buffer, uint8_t size )
{ 
    if( ( size > SPI_TRANSFER_BUF_LEN ) || ( size == 0 ) )
        return;
    
    //memset( rf_spi_tx_buf, 0, sizeof( rf_spi_tx_buf ) );
    
    ASSERT( buffer );
    
    rf_spi_tx_buf[0] = addr | 0x80;
    
    memcpy( &rf_spi_tx_buf[1], buffer, size );
    
    radio_spi_transfer( rf_spi_tx_buf, size + 1, NULL, 0 );
}

void sx1276_read_buffer( uint32_t addr, uint8_t *buffer, uint8_t size )
{
    if( ( size > SPI_TRANSFER_BUF_LEN ) || ( size == 0 ) )
        return;
    
    memset(rf_spi_tx_buf, 0, sizeof(rf_spi_tx_buf));
    memset(rf_spi_rx_buf, 0, sizeof(rf_spi_rx_buf));
    
    ASSERT( buffer );
    
    rf_spi_tx_buf[0] = addr & 0x7F;
    
    radio_spi_transfer( rf_spi_tx_buf, size + 1, rf_spi_rx_buf, size + 1 );
    
    memcpy(buffer, &rf_spi_rx_buf[1], size);
}

static void sx1276_write_fifo( uint8_t *buffer, uint8_t size )
{
    sx1276_write_buffer( 0, buffer, size );
}

static void sx1276_read_fifo( uint8_t *buffer, uint8_t size )
{
    sx1276_read_buffer( 0, buffer, size );
}

void sx1276_set_max_payload_length( RadioModems_t modem, uint8_t max )
{
    sx1276_set_modem( modem );

    switch( modem )
    {
        default :
        case MODEM_LORA:
            sx1276_write( REG_LR_PAYLOADMAXLENGTH, max );
        break;
    }
}

void sx1276_set_public_network( bool enable )
{
    sx1276_set_modem( MODEM_LORA );
    SX1276.Settings.LoRa.PublicNetwork = enable;
    if( enable == true )
    {
        // Change LoRa modem SyncWord
        sx1276_write( REG_LR_SYNCWORD, LORA_MAC_PUBLIC_SYNCWORD );
    }
    else
    {
        // Change LoRa modem SyncWord
        sx1276_write( REG_LR_SYNCWORD, LORA_MAC_PRIVATE_SYNCWORD );
    }
}

uint32_t sx1276_get_wakeup_time( void )
{
    return sx1276_get_board_tcxo_wakeup_time( ) + RADIO_WAKEUP_TIME;
}

static uint32_t sx1276_convert_pll_step_to_freq_in_hz( uint32_t pllSteps )
{
    uint32_t freqInHzInt;
    uint32_t freqInHzFrac;
    
    // freqInHz = pllSteps * ( SX1276_XTAL_FREQ / 2^19 )
    // Get integer and fractional parts of the frequency computed with a PLL step scaled value
    freqInHzInt = pllSteps >> SX1276_PLL_STEP_SHIFT_AMOUNT;
    freqInHzFrac = pllSteps - ( freqInHzInt << SX1276_PLL_STEP_SHIFT_AMOUNT );
    
    // Apply the scaling factor to retrieve a frequency in Hz (+ ceiling)
    return freqInHzInt * SX1276_PLL_STEP_SCALED + 
           ( ( freqInHzFrac * SX1276_PLL_STEP_SCALED + ( 128 ) ) >> SX1276_PLL_STEP_SHIFT_AMOUNT );
}

static uint32_t sx1276_convert_freq_in_hz_to_pll_step( uint32_t freqInHz )
{
    uint32_t stepsInt;
    uint32_t stepsFrac;

    // pllSteps = freqInHz / (SX1276_XTAL_FREQ / 2^19 )
    // Get integer and fractional parts of the frequency computed with a PLL step scaled value
    stepsInt = freqInHz / SX1276_PLL_STEP_SCALED;
    stepsFrac = freqInHz - ( stepsInt * SX1276_PLL_STEP_SCALED );
    
    // Apply the scaling factor to retrieve a frequency in Hz (+ ceiling)
    return ( stepsInt << SX1276_PLL_STEP_SHIFT_AMOUNT ) + 
           ( ( ( stepsFrac << SX1276_PLL_STEP_SHIFT_AMOUNT ) + ( SX1276_PLL_STEP_SCALED >> 1 ) ) /
             SX1276_PLL_STEP_SCALED );
}

static uint8_t get_fsk_bandwidth_reg_value( uint32_t bw )
{
    uint8_t i;

    for( i = 0; i < ( sizeof( FskBandwidths ) / sizeof( FskBandwidth_t ) ) - 1; i++ )
    {
        if( ( bw >= FskBandwidths[i].bandwidth ) && ( bw < FskBandwidths[i + 1].bandwidth ) )
        {
            return FskBandwidths[i].RegValue;
        }
    }
    // ERROR: Value not found
    while( 1 );
}

static uint32_t sx1276_get_lora_bandwidth_in_hz(uint32_t bw)
{
    uint32_t bandwidthInHz = 0;

    switch(bw)
    {
        case 0: // 125 kHz
            bandwidthInHz = 125000UL;
            break;
        case 1: // 250 kHz
            bandwidthInHz = 250000UL;
            break;
        case 2: // 500 kHz
            bandwidthInHz = 500000UL;
            break;
    }

    return bandwidthInHz;
}

static uint32_t sx1276_get_lora_time_on_air_numerator( uint32_t bandwidth,
                              uint32_t datarate, uint8_t coderate,
                              uint16_t preambleLen, bool fixLen, uint8_t payloadLen,
                              bool crcOn )
{
    int32_t crDenom           = coderate + 4;
    bool    lowDatareOptimize = false;

    // Ensure that the preamble length is at least 12 symbols when using SF5 or
    // SF6
    if( ( datarate == 5 ) || ( datarate == 6 ) )
    {
        if( preambleLen < 12 )
        {
            preambleLen = 12;
        }
    }

    if( ( ( bandwidth == 0 ) && ( ( datarate == 11 ) || ( datarate == 12 ) ) ) ||
        ( ( bandwidth == 1 ) && ( datarate == 12 ) ) )
    {
        lowDatareOptimize = true;
    }

    int32_t ceilDenominator;
    int32_t ceilNumerator = ( payloadLen << 3 ) +
                            ( crcOn ? 16 : 0 ) -
                            ( 4 * datarate ) +
                            ( fixLen ? 0 : 20 );

    if( datarate <= 6 )
    {
        ceilDenominator = 4 * datarate;
    }
    else
    {
        ceilNumerator += 8;

        if( lowDatareOptimize == true )
        {
            ceilDenominator = 4 * ( datarate - 2 );
        }
        else
        {
            ceilDenominator = 4 * datarate;
        }
    }

    if( ceilNumerator < 0 )
    {
        ceilNumerator = 0;
    }

    // Perform integral ceil()
    int32_t intermediate =
        ( ( ceilNumerator + ceilDenominator - 1 ) / ceilDenominator ) * crDenom + preambleLen + 12;

    if( datarate <= 6 )
    {
        intermediate += 2;
    }

    return ( uint32_t )( ( 4 * intermediate + 1 ) * ( 1 << ( datarate - 2 ) ) );
}

static void sx1276_ontimeout_irq( void* context )
{
    switch( SX1276.Settings.State )
    {
        case RF_RX_RUNNING:
        if( ( RadioEvents != NULL ) && ( RadioEvents->RxTimeout != NULL ) )
        {
            RadioEvents->RxTimeout( );
        }
        break;
        
        case RF_TX_RUNNING:
        // Tx timeout shouldn't happen.
        // Reported issue of SPI data corruption resulting in TX TIMEOUT 
        // is NOT related to a bug in radio transceiver.
        // It is mainly caused by improper PCB routing of SPI lines and/or
        // violation of SPI specifications.
        // To mitigate redesign, Semtech offers a workaround which resets
        // the radio transceiver and putting it into a known state.

        // BEGIN WORKAROUND

        // Reset the radio
        sx1276_reset( );

        // Calibrate Rx chain
        rx_chain_calibration( );

        // Initialize radio default values
        sx1276_set_opmode( RF_OPMODE_SLEEP );

        for( uint8_t i = 0; i < sizeof( RadioRegsInit ) / sizeof( RadioRegisters_t ); i++ )
        {
            sx1276_set_modem( RadioRegsInit[i].Modem );
            sx1276_write( RadioRegsInit[i].Addr, RadioRegsInit[i].Value );
        }
        sx1276_set_modem( MODEM_FSK );

        // Restore previous network type setting.
        sx1276_set_public_network( SX1276.Settings.LoRa.PublicNetwork );
        // END WORKAROUND

        SX1276.Settings.State = RF_IDLE;
        if( ( RadioEvents != NULL ) && ( RadioEvents->TxTimeout != NULL ) )
        {
            RadioEvents->TxTimeout( );
        }
        break;
    default:
        break;
    }
}

static void sx1276_ondio0_irq(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    volatile uint8_t irqFlags = 0;

    switch( SX1276.Settings.State )
    {
        case RF_RX_RUNNING:
            // RxDone interrupt
            switch( SX1276.Settings.Modem )
            {
                case MODEM_LORA:
                {
                    // Clear Irq
                    sx1276_write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE );

                    irqFlags = sx1276_read( REG_LR_IRQFLAGS );
                    if( ( irqFlags & RFLR_IRQFLAGS_PAYLOADCRCERROR_MASK ) == RFLR_IRQFLAGS_PAYLOADCRCERROR )
                    {
                        // Clear Irq
                        sx1276_write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_PAYLOADCRCERROR );

                        if( SX1276.Settings.LoRa.RxContinuous == false )
                        {
                            SX1276.Settings.State = RF_IDLE;
                        }
                        
                        app_timer_stop( rx_timeout_timer_id );

                        if( ( RadioEvents != NULL ) && ( RadioEvents->RxError != NULL ) )
                        {
                            RadioEvents->RxError( );
                        }
                        break;
                    }

                    // Returns SNR value [dB] rounded to the nearest integer value
                    SX1276.Settings.LoRaPacketHandler.SnrValue = ( ( ( int8_t )sx1276_read( REG_LR_PKTSNRVALUE ) ) + 2 ) >> 2;

                    int16_t rssi = sx1276_read( REG_LR_PKTRSSIVALUE );
                    if( SX1276.Settings.LoRaPacketHandler.SnrValue < 0 )
                    {
                        if( SX1276.Settings.Channel > RF_MID_BAND_THRESH )
                        {
                            SX1276.Settings.LoRaPacketHandler.RssiValue = RSSI_OFFSET_HF + rssi + ( rssi >> 4 ) +
                                                                          SX1276.Settings.LoRaPacketHandler.SnrValue;
                        }
                        else
                        {
                            SX1276.Settings.LoRaPacketHandler.RssiValue = RSSI_OFFSET_LF + rssi + ( rssi >> 4 ) +
                                                                          SX1276.Settings.LoRaPacketHandler.SnrValue;
                        }
                    }
                    else
                    {
                        if( SX1276.Settings.Channel > RF_MID_BAND_THRESH )
                        {
                            SX1276.Settings.LoRaPacketHandler.RssiValue = RSSI_OFFSET_HF + rssi + ( rssi >> 4 );
                        }
                        else
                        {
                            SX1276.Settings.LoRaPacketHandler.RssiValue = RSSI_OFFSET_LF + rssi + ( rssi >> 4 );
                        }
                    }

                    SX1276.Settings.LoRaPacketHandler.Size = sx1276_read( REG_LR_RXNBBYTES );
                    sx1276_write( REG_LR_FIFOADDRPTR, sx1276_read( REG_LR_FIFORXCURRENTADDR ) );
                    sx1276_read_fifo( RxTxBuffer, SX1276.Settings.LoRaPacketHandler.Size );

                    if( SX1276.Settings.LoRa.RxContinuous == false )
                    {
                        SX1276.Settings.State = RF_IDLE;
                    }
                    
                    app_timer_stop( rx_timeout_timer_id );

                    if( ( RadioEvents != NULL ) && ( RadioEvents->RxDone != NULL ) )
                    {
                        RadioEvents->RxDone( RxTxBuffer, SX1276.Settings.LoRaPacketHandler.Size, SX1276.Settings.LoRaPacketHandler.RssiValue, SX1276.Settings.LoRaPacketHandler.SnrValue );
                    }
                }
                break;
                
                default:
                break;
            }
            break;
        case RF_TX_RUNNING:
            app_timer_stop( tx_timeout_timer_id );
            // TxDone interrupt
            switch( SX1276.Settings.Modem )
            {
                case MODEM_LORA:
                    // Clear Irq
                    sx1276_write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE );
                // Intentional fall through
                case MODEM_FSK:
                default:
                    SX1276.Settings.State = RF_IDLE;
                    if( ( RadioEvents != NULL ) && ( RadioEvents->TxDone != NULL ) )
                    {
                        RadioEvents->TxDone( );
                    }
                break;
            }
            break;
            
        default:
            break;
    }
}

