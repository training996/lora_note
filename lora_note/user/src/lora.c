/**
 * lora.c
 */
#include "lora.h"

typedef enum
{
    LOWPOWER,
    RX,
    RX_TIMEOUT,
    RX_ERROR,
    TX,
    TX_TIMEOUT,
}States_t;

uint16_t BufferSize = BUFFER_SIZE;
uint8_t Buffer[BUFFER_SIZE];

States_t State = LOWPOWER;

int8_t RssiValue = 0;
int8_t SnrValue = 0;

/**
 * Radio events function pointer
 */
static RadioEvents_t RadioEvents;

/**
 * Radio initialization.
 */
void Lora_init( void )
{
    Radio.SpiInit ( );

    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError = OnRxError;

    Radio.Init( &RadioEvents );

    Radio.SetChannel( RF_FREQUENCY );

    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );

    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                   LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                   LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   0, true, 0, 0, LORA_IQ_INVERSION_ON, true );

    Radio.SetMaxPayloadLength( MODEM_LORA, BUFFER_SIZE );

    nrf_delay_ms(2);
    Radio.Standby( );
    nrf_delay_ms(2);
}


/**@brief Function to be executed on Radio Tx Done event
 */
void OnTxDone( void )
{
    Radio.Sleep( );
    NRF_LOG_INFO("TxDone");
    State = TX;
}

/**@brief Function to be executed on Radio Rx Done event
 */
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    Radio.Sleep( );
    memset(Buffer,0,BUFFER_SIZE);
    BufferSize = size;
    memcpy( Buffer, payload, BufferSize );
    RssiValue = rssi;
    SnrValue = snr;
    State = RX;
    NRF_LOG_INFO("RxDone\r\nrssi:%d\r\nsnr:%d\r\nsize:%d\r\npayload:%s\r\n",RssiValue,SnrValue,BufferSize,Buffer);
    Radio.Rx( RX_TIMEOUT_VALUE );
}

/**@brief Function executed on Radio Tx Timeout event
 */
void OnTxTimeout( void )
{
    Radio.Sleep( );
    NRF_LOG_INFO("TxTIMEOUT");
    State = TX_TIMEOUT;
}

/**@brief Function executed on Radio Rx Timeout event
 */
void OnRxTimeout( void )
{
    Radio.Sleep( );
    State = RX_TIMEOUT;
    NRF_LOG_INFO("RxTIMEOUT");
    Radio.Rx( RX_TIMEOUT_VALUE );
}

/**@brief Function executed on Radio Rx Error event
 */
void OnRxError( void )
{
    Radio.Sleep( );
    State = RX_ERROR;
    NRF_LOG_INFO("RxError");
    Radio.Rx( RX_TIMEOUT_VALUE );
}
