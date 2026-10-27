/**
 * @file sx127x.h
 * @brief Minimal, platform independent driver for the Semtech SX1276/77/78/79
 *        LoRa modem (used by the "LoRa RA-02" 433 MHz module).
 *
 * The driver only speaks registers over SPI. All platform specific parts
 * (SPI transfer, chip select, reset pin, optional DIO0 pin, delay) are
 * injected through sx127x_hal_t, so the very same driver file runs on
 * ESP-IDF (meter / slave) and on Arduino (gateway / master).
 *
 * Not implemented: FSK/OOK mode, CAD, temperature compensation, explicit
 * frequency hopping. Only plain LoRa point to point, which is all this
 * project needs.
 */
#pragma once

#ifndef SX127X_H
#define SX127X_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------- */
/* Register map (only the ones we use)                                       */
/* ------------------------------------------------------------------------- */
#define SX127X_REG_FIFO                 0x00
#define SX127X_REG_OPMODE               0x01
#define SX127X_REG_FRF_MSB              0x06
#define SX127X_REG_FRF_MID              0x07
#define SX127X_REG_FRF_LSB              0x08
#define SX127X_REG_PACONFIG             0x09
#define SX127X_REG_PARAMP               0x0A
#define SX127X_REG_OCP                  0x0B
#define SX127X_REG_LNA                  0x0C
#define SX127X_REG_FIFO_ADDR_PTR        0x0D
#define SX127X_REG_FIFO_TX_BASE_ADDR    0x0E
#define SX127X_REG_FIFO_RX_BASE_ADDR    0x0F
#define SX127X_REG_FIFO_RX_CURRENT_ADDR 0x10
#define SX127X_REG_IRQ_FLAGS_MASK       0x11
#define SX127X_REG_IRQ_FLAGS            0x12
#define SX127X_REG_RX_NB_BYTES          0x13
#define SX127X_REG_PKT_SNR_VALUE        0x19
#define SX127X_REG_PKT_RSSI_VALUE       0x1A
#define SX127X_REG_RSSI_VALUE           0x1B
#define SX127X_REG_MODEM_CONFIG_1       0x1D
#define SX127X_REG_MODEM_CONFIG_2       0x1E
#define SX127X_REG_SYMB_TIMEOUT_LSB     0x1F
#define SX127X_REG_PREAMBLE_MSB         0x20
#define SX127X_REG_PREAMBLE_LSB         0x21
#define SX127X_REG_PAYLOAD_LENGTH       0x22
#define SX127X_REG_MODEM_CONFIG_3       0x26
#define SX127X_REG_DETECT_OPTIMIZE      0x31
#define SX127X_REG_INVERT_IQ            0x33
#define SX127X_REG_DETECTION_THRESHOLD  0x37
#define SX127X_REG_SYNC_WORD            0x39
#define SX127X_REG_DIO_MAPPING_1        0x40
#define SX127X_REG_VERSION              0x42
#define SX127X_REG_PA_DAC               0x4D

/* IRQ flags */
#define SX127X_IRQ_RX_TIMEOUT           0x80
#define SX127X_IRQ_RX_DONE               0x40
#define SX127X_IRQ_CRC_ERROR            0x20
#define SX127X_IRQ_VALID_HEADER         0x10
#define SX127X_IRQ_TX_DONE              0x08
#define SX127X_IRQ_CAD_DONE             0x04
#define SX127X_IRQ_FHSS_CHANGE          0x02
#define SX127X_IRQ_CAD_DETECTED         0x01
#define SX127X_IRQ_ALL                  0xFF

/* Operation modes (combine with SX127X_LONG_RANGE_MODE) */
#define SX127X_LONG_RANGE_MODE          0x80
#define SX127X_MODE_SLEEP               0x00
#define SX127X_MODE_STDBY               0x01
#define SX127X_MODE_TX                  0x03
#define SX127X_MODE_RX_CONT             0x05
#define SX127X_MODE_RX_SINGLE           0x06

/** Result codes */
typedef enum
{
    SX127X_OK = 0,
    SX127X_ERR_SPI = -1,        /**< SPI / HAL failed                        */
    SX127X_ERR_NOT_SX127X = -2, /**< RegVersion is not 0x12                  */
    SX127X_ERR_PARAM = -3,      /**< invalid argument                        */
    SX127X_ERR_TIMEOUT = -4,    /**< TX or RX did not finish in time         */
    SX127X_ERR_CRC = -5         /**< received packet failed the radio CRC    */
} sx127x_result_t;

/**
 * @brief Platform hooks. Fill this in once per platform.
 *
 * spi_xfer() must send @p len bytes from @p tx and store the received bytes in
 * @p rx (rx may be NULL if the caller is not interested). Chip select is
 * handled by cs(), not by the SPI peripheral.
 */
typedef struct
{
    void *ctx;                                                         /**< your context (SPI device handle, ...) */
    int  (*spi_xfer)(void *ctx, const uint8_t *tx, uint8_t *rx, size_t len);
    void (*cs)(void *ctx, int level);                                  /**< 0 = selected, 1 = idle */
    void (*reset)(void *ctx, int level);                               /**< 0 = reset asserted    */
    int  (*dio0)(void *ctx);                                           /**< may be NULL (then polled) */
    void (*delay_ms)(void *ctx, uint32_t ms);
} sx127x_hal_t;

/** Radio settings */
typedef struct
{
    uint32_t frequency_hz;      /**< 433000000 / 868000000 / 915000000 ...   */
    uint8_t  spreading_factor;  /**< 6..12, higher = slower but longer range */
    uint32_t bandwidth_hz;      /**< 7800 .. 500000 (125000 is a good default) */
    uint8_t  coding_rate;       /**< 5..8  -> 4/5 .. 4/8                     */
    int8_t   tx_power_dbm;      /**< 2..20 (RA-02: use <= 17 for a safe start) */
    uint8_t  sync_word;         /**< 0x12 private network (default), 0x34 LoRaWAN */
    uint16_t preamble_len;      /**< symbols, 8 is the SX127x default         */
    bool     crc_on;            /**< payload CRC generated by the radio       */
    uint16_t tx_timeout_ms;     /**< max time to wait for TxDone              */
} sx127x_config_t;

/** Driver instance */
typedef struct
{
    sx127x_hal_t hal;
    uint32_t frequency_hz;
    uint8_t  spreading_factor;
    uint8_t  bandwidth_code;
    uint8_t  coding_rate;
    int8_t   tx_power_dbm;
    uint16_t tx_timeout_ms;
    int8_t   last_rssi;         /**< RSSI of the last received packet in dBm  */
    int8_t   last_snr;          /**< SNR of the last received packet in dB    */
    bool     initialised;
} sx127x_t;

/** Default values that work between two RA-02 modules on 433 MHz */
#define SX127X_DEFAULT_CONFIG                     \
    {                                             \
        433000000UL, /* frequency_hz      */      \
        9,           /* spreading_factor  */      \
        125000UL,    /* bandwidth_hz      */      \
        5,           /* coding_rate       */      \
        17,          /* tx_power_dbm      */      \
        0x12,        /* sync_word         */      \
        8,           /* preamble_len      */      \
        true,        /* crc_on            */      \
        5000         /* tx_timeout_ms     */      \
    }

/* ------------------------------------------------------------------------- */
/* API                                                                        */
/* ------------------------------------------------------------------------- */

/**
 * @brief Reset the radio and apply @p cfg.
 * @return SX127X_OK on success.
 */
sx127x_result_t sx127x_init(sx127x_t *dev, const sx127x_hal_t *hal, const sx127x_config_t *cfg);

/** Re-apply all modem settings from the instance (after a mode change) */
sx127x_result_t sx127x_apply_config(sx127x_t *dev);

/** Put the radio into sleep mode (lowest current, SPI still works) */
sx127x_result_t sx127x_sleep(sx127x_t *dev);
/** Put the radio into standby */
sx127x_result_t sx127x_standby(sx127x_t *dev);

/** Change the frequency at runtime (e.g. channel hopping) */
sx127x_result_t sx127x_set_frequency(sx127x_t *dev, uint32_t frequency_hz);
/** Change the output power at runtime */
sx127x_result_t sx127x_set_tx_power(sx127x_t *dev, int8_t dbm);

/**
 * @brief Transmit one packet (blocking).
 * @return SX127X_OK when TxDone was seen, SX127X_ERR_TIMEOUT otherwise.
 */
sx127x_result_t sx127x_transmit(sx127x_t *dev, const uint8_t *data, uint8_t len);

/** Start continuous receive mode */
sx127x_result_t sx127x_start_rx(sx127x_t *dev);

/**
 * @brief Non blocking poll for a received packet.
 * @return 1 = packet in @p buf (length in @p out_len), 0 = nothing yet,
 *         negative = sx127x_result_t error.
 */
int sx127x_poll_rx(sx127x_t *dev, uint8_t *buf, uint8_t max_len, uint8_t *out_len);

/**
 * @brief Wait up to @p timeout_ms for a packet.
 * @return same as sx127x_poll_rx()
 */
int sx127x_wait_rx(sx127x_t *dev, uint8_t *buf, uint8_t max_len, uint8_t *out_len,
                   uint32_t timeout_ms);

/** Low level register access (exposed for debugging / tests) */
uint8_t sx127x_read_reg(sx127x_t *dev, uint8_t addr);
void    sx127x_write_reg(sx127x_t *dev, uint8_t addr, uint8_t value);
void    sx127x_read_burst(sx127x_t *dev, uint8_t addr, uint8_t *buf, size_t len);
void    sx127x_write_burst(sx127x_t *dev, uint8_t addr, const uint8_t *buf, size_t len);

/** Estimated time on air of a packet in milliseconds */
uint32_t sx127x_time_on_air_ms(const sx127x_t *dev, uint8_t payload_len);

/** Map a bandwidth in Hz to the modem config code (returns 0xFF if unsupported) */
uint8_t sx127x_bandwidth_code(uint32_t bandwidth_hz);

/** Human readable name of the result code */
const char *sx127x_result_str(sx127x_result_t res);

#ifdef __cplusplus
}
#endif

#endif // SX127X_H
