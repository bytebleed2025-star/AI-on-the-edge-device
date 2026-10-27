/**
 * @file lora_link.h
 * @brief LoRa slave (meter side): sends each accepted reading to the master.
 *
 * Wiring defaults (see also SETUP.md, section 8):
 *
 *   RA-02   ESP32-S3 CAM        note
 *   ------  ------------------  -----------------------------------------
 *   SCK      GPIO 14
 *   MISO     GPIO 21
 *   MOSI     GPIO 41
 *   NSS      GPIO 42
 *   RST      GPIO 47
 *   DIO0     -                  not needed, the driver polls the IRQ flags
 *   3.3V     3V3                never 5 V
 *   GND      GND
 *
 * All pins can be changed in /sdcard/lora.ini without recompiling.
 */
#pragma once

#ifndef LORA_LINK_H
#define LORA_LINK_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Read /sdcard/lora.ini, initialise SPI + radio and start the link.
 *
 * Safe to call when no module is connected: the function only logs an error
 * and the rest of the firmware keeps working.
 *
 * @return true if the radio answered and is ready to send.
 */
bool LoRaLink_Init(void);

/** true when the radio was initialised successfully */
bool LoRaLink_IsReady(void);

/**
 * @brief Send one meter reading to the master (blocking, includes ACK wait).
 *
 * @param sensor     number name, e.g. "default"
 * @param value      formatted value, e.g. "123.456" (may be empty on error)
 * @param timestamp  unix time of the reading, 0 if unknown
 * @param error      true if the post processing reported an error
 * @return true if the packet was transmitted (and acknowledged when ACK is on)
 */
bool LoRaLink_SendReading(const char *sensor, const char *value, uint32_t timestamp, bool error);

/** Statistics for logging / debugging */
typedef struct
{
    uint32_t sent;         /**< packets handed to the radio            */
    uint32_t acked;        /**< packets acknowledged by the master     */
    uint32_t failed;       /**< packets that were not acknowledged     */
    uint32_t retries;      /**< extra transmissions                    */
    int8_t   last_rssi;    /**< RSSI of the last ACK in dBm            */
    int8_t   last_snr;     /**< SNR of the last ACK in dB              */
} lora_link_stats_t;

void LoRaLink_GetStats(lora_link_stats_t *out);

/** Put the radio into sleep mode (e.g. before deep sleep) */
void LoRaLink_Sleep(void);

#ifdef __cplusplus
}
#endif

#endif // LORA_LINK_H
