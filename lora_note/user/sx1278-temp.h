/**
 * sx1278-temp.h
 */
#ifndef SX1278_TEMP_H__
#define SX1278_TEMP_H__

#include "sx1276.h"
#include "sx1276-board.h"
#include "radio.h"
#include "sx1276Regs-Fsk.h"
#include "sx1276Regs-LoRa.h"
#include "nrf_delay.h"

int8_t RadioGetRawTenp(void);

#endif
