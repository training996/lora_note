/**
 * Reads the raw tempexature
 */
#include "sx1278-temp.h"

int8_t RadioGetRawTenp( void )
{
    int8_t temp = 0;
    uint8_t previousOpMode;
    // Save current Operation Mode
    previousOpMode = sx1276_read( REG_OPMODE);

    // Pass through LoRa sleep only necessary if reading temperature while in LoRa Mode
    if(( previousOpMode & RFLR_OPMODE_LONGRANGEMODE_ON ) == RFLR_OPMODE_LONGRANGEMODE_ON )
    {
       sx1276_write( REG_OPMODE, RFLR_OPMODE_SLEEP );// put device in LoRa sleep Mode
    }
    // Put device in FSK Sleep Mode
    sx1276_write( REG_OPMODE, RF_OPMODE_SLEEP );

    // Put device in FSK RxSynth
    sx1276_write( REG_OPMODE , RF_OPMODE_SYNTHESIZER_RX );

    // Enable Temperature reading
    uint8_t EnableTempReg;
    EnableTempReg = ( sx1276_read( REG_IMAGECAL) & RF_IMAGECAL_TEMPMONITOR_MASK ) | RF_IMAGECAL_TEMPMONITOR_ON;
    sx1276_write( REG_IMAGECAL,EnableTempReg );

    // Wait 150us
    nrf_delay_us( 150 );

    // Disable Temperature reading
    uint8_t DisableTempReg;
    DisableTempReg = ( sx1276_read( REG_IMAGECAL) & RF_IMAGECAL_TEMPMONITOR_MASK ) | RF_IMAGECAL_TEMPMONITOR_OFF;
    sx1276_write( REG_IMAGECAL,DisableTempReg );

    // Put device in FSK Sleep Mode
    sx1276_write( REG_OPMODE,RF_OPMODE_SLEEP );
		
    // Read temperature
    if( ( sx1276_read( REG_TEMP) & 0x80 )== 0x80 )
    {
      temp= 255 - sx1276_read( REG_TEMP);
    }
    else
    {
      temp = sx1276_read( REG_TEMP);
      temp *= -1;
    }
    //We were in LoRa Mode prior to the temperature reading
    if( ( previousOpMode & RFLR_OPMODE_LONGRANGEMODE_ON ) | RFLR_OPMODE_LONGRANGEMODE_ON )
    {
       sx1276_write( REG_OPMODE, RFLR_OPMODE_SLEEP );
    }
		// Reload previous Op Mode
    sx1276_write( REG_OPMODE,previousOpMode );
    return (temp + 15);
}
