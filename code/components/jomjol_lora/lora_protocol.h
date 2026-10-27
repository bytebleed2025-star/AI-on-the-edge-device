/**
 * @file lora_protocol.h
 * @brief Wire format of the LoRa link between the meter (slave) and the gateway (master).
 *
 * This file is shared by both firmwares:
 *   - slave : AI-on-the-edge-device / code/components/jomjol_lora/
 *   - master: lora-master/ (Arduino, ESP32 without camera)
 *
 * Keep the two copies identical!
 *
 * All multi byte fields are little endian (ESP32 and ESP32-S3 are both LE).
 * A packet is always sizeof(lora_packet_t) bytes, the structure is packed so
 * no padding is inserted. Do not reorder or resize fields without bumping
 * LORA_PROTO_VERSION on both sides.
 */
#pragma once

#ifndef LORA_PROTOCOL_H
#define LORA_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#define LORA_PROTO_MAGIC       0xA5
#define LORA_PROTO_VERSION     1

/* Largest payload the SX127x accepts in one packet (FIFO is 256 bytes) */
#define LORA_MAX_PAYLOAD       255

/* Meter id / sensor name length including the terminating NUL */
#define LORA_ID_LEN            16
#define LORA_SENSOR_LEN        12

/** Message types */
enum
{
    LORA_TYPE_READING = 1,   /**< slave -> master, a meter reading            */
    LORA_TYPE_ACK     = 2,   /**< master -> slave, acknowledges a reading     */
    LORA_TYPE_STATUS  = 3,   /**< slave -> master, alive / no reading yet     */
    LORA_TYPE_CMD     = 4    /**< master -> slave, reserved (e.g. force run)  */
};

/** Flags of lora_packet_t::flags */
enum
{
    LORA_FLAG_ERROR    = 0x01,  /**< reading contains an error / was not validated */
    LORA_FLAG_PREVALUE = 0x02,  /**< pre value was used to replace unknown digits  */
    LORA_FLAG_ACK_REQ  = 0x04   /**< sender wants an ACK                            */
};

/**
 * @brief One LoRa packet.
 *
 * size = 1 + 1 + 1 + 1 + 16 + 12 + 4 + 4 + 1 + 1 + 2 = 44 bytes
 */
typedef struct __attribute__((packed))
{
    uint8_t  magic;                     /**< LORA_PROTO_MAGIC                       */
    uint8_t  version;                   /**< LORA_PROTO_VERSION                     */
    uint8_t  type;                      /**< LORA_TYPE_*                            */
    uint8_t  seq;                       /**< sequence number, echoed in the ACK     */
    char     id[LORA_ID_LEN];           /**< device / meter id, NUL padded           */
    char     sensor[LORA_SENSOR_LEN];   /**< number name, e.g. "default"            */
    float    value;                     /**< meter reading                          */
    uint32_t timestamp;                 /**< unix time of the reading, 0 = unknown  */
    uint8_t  decimals;                  /**< decimal places of value                */
    uint8_t  flags;                     /**< LORA_FLAG_*                            */
    uint16_t crc16;                     /**< CRC16-CCITT over all bytes before this */
} lora_packet_t;

/** CRC16-CCITT (0x1021, init 0xFFFF) - used for the packet checksum */
static inline uint16_t lora_crc16(const void *data, size_t len)
{
    const uint8_t *p = (const uint8_t *)data;
    uint16_t crc = 0xFFFF;

    for (size_t i = 0; i < len; ++i)
    {
        crc ^= (uint16_t)p[i] << 8;
        for (int b = 0; b < 8; ++b)
        {
            if (crc & 0x8000)
                crc = (uint16_t)((crc << 1) ^ 0x1021);
            else
                crc = (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/** Fill in magic / version / crc16 of a packet before sending */
static inline void lora_packet_seal(lora_packet_t *p)
{
    p->magic = LORA_PROTO_MAGIC;
    p->version = LORA_PROTO_VERSION;
    p->crc16 = lora_crc16(p, offsetof(lora_packet_t, crc16));
}

/**
 * @brief Validate a received buffer and copy it into a packet structure.
 * @return true if magic, version, length and CRC are all correct.
 */
static inline int lora_packet_open(const uint8_t *buf, size_t len, lora_packet_t *out)
{
    if (len != sizeof(lora_packet_t))
        return 0;

    lora_packet_t tmp;
    memcpy(&tmp, buf, sizeof(tmp));

    if (tmp.magic != LORA_PROTO_MAGIC || tmp.version != LORA_PROTO_VERSION)
        return 0;

    uint16_t crc = lora_crc16(&tmp, offsetof(lora_packet_t, crc16));
    if (crc != tmp.crc16)
        return 0;

    /* make sure the string fields are terminated */
    tmp.id[LORA_ID_LEN - 1] = '\0';
    tmp.sensor[LORA_SENSOR_LEN - 1] = '\0';

    *out = tmp;
    return 1;
}

#endif // LORA_PROTOCOL_H
