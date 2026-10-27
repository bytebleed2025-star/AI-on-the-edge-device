# jomjol_lora – LoRa slave (meter → master)

Adds a **LoRa RA-02 / SX1278** link to the AI-on-the-edge firmware: after every
successful digitization round the accepted reading(s) are sent to the master
(gateway) as a small binary packet. The master acknowledges each packet; when the
ACK is missing the packet is retried.

The REST API, MQTT, InfluxDB and the web UI are **not** affected by this
component – it works in parallel.

Compiled in only when the build flag `ENABLE_LORA` is set (it is set in the
`esp32s3-wroom-cam` environment).

---

## 1. Wiring (ESP32-S3-WROOM N16R8 CAM board)

| RA-02 | ESP32-S3 CAM | Notes |
| ----- | ------------ | ----- |
| VCC | 3V3 | **3.3 V only** – never 5 V |
| GND | GND | |
| SCK | GPIO 14 | |
| MISO | GPIO 21 | |
| MOSI | GPIO 41 | |
| NSS | GPIO 42 | chip select |
| RST | GPIO 47 | hard reset |
| DIO0 | – | not needed, the driver polls the IRQ flags (can be set to 48) |

These pins avoid the camera bus (4–13, 15–18), the SD card (38–40), UART0 (43/44),
USB (19/20), the octal PSRAM (33–37) and the strapping pins (0, 3, 45, 46).

**Always connect an antenna before powering up**, otherwise the PA can be damaged.

All pins and radio settings can be changed in `/sdcard/lora.ini` – no recompile
needed.

---

## 2. `/sdcard/lora.ini`

Copy `lora.ini.example` from this folder to the root of the SD card and rename it
to `lora.ini`. Without the file the built-in defaults are used (which match the
defaults of `lora-master`).

```ini
[LoRa]
enabled         = true
frequency       = 433000000     ; Hz
spreading_factor = 9            ; 6..12, higher = longer range / slower
bandwidth       = 125000        ; Hz
coding_rate     = 5             ; 4/5
tx_power        = 17            ; dBm, max 20 (RA-02)
sync_word       = 0x12          ; private network, must match the master
preamble        = 8
ack             = true
ack_timeout_ms  = 1500
retries         = 2
pin_sck         = 14
pin_miso        = 21
pin_mosi        = 41
pin_nss         = 42
pin_reset       = 47
pin_dio0        = -1            ; -1 = not wired (polled)
```

The settings must match `include/config.h` of the `lora-master` project.

---

## 3. Behaviour

* The packet is sent from `ClassFlowPostProcessing::doFlow()` right after the
  values were written to the data log, i.e. once per digitization round.
* Only values that passed the post processing are sent (empty value = the round
  had an error → nothing is sent).
* The slave waits up to `ack_timeout_ms` for the ACK of the master and retries
  `retries` times.
* A failing radio link is logged only; it never blocks the meter firmware.

Log examples:

```
I (3120) LORA: RA-02 ready: 433.000 MHz, SF9, BW125 kHz, CR4/5, 17 dBm, sync 0x12, ACK on
I (60123) LORA: sent default=123.456 (seq 1, rssi -64 dBm, snr 8 dB)
W (61234) LORA: no ACK for seq 2 (attempt 1/3)
I (62987) LORA: sent default=123.456 (seq 2, rssi -63 dBm, snr 9 dB)
```

---

## 4. Packet format

```
byte 0   magic  0xA5
byte 1   version 1
byte 2   type   1 = reading, 2 = ACK, 3 = status
byte 3   seq    sequence number, echoed by the ACK
byte 4   id[16] device name (hostname from wlan.ini), NUL padded
byte 20  sensor[12] number name ("default"), NUL padded
byte 32  value  float32  (little endian)
byte 36  timestamp uint32 (unix time, 0 = unknown)
byte 40  decimals uint8
byte 41  flags  bit0 error, bit1 prevalue used, bit2 ACK requested
byte 42  crc16  CRC16-CCITT over bytes 0..41
```

Total: 44 bytes, i.e. ~150 ms airtime at SF9 / BW125.

The same header is used by the master (`lora-master/include/lora_protocol.h`).

---

## 5. Files

| File | Purpose |
| ---- | ------- |
| `sx127x.h` / `sx127x.c` | platform independent SX1276/77/78/79 LoRa driver |
| `lora_protocol.h` | packet format + CRC (shared with the master) |
| `lora_link.h` / `lora_link.cpp` | ESP-IDF glue: SPI, `lora.ini`, ACK handling |
| `lora.ini.example` | template for the SD card |

## 6. Troubleshooting

| Log message | Cause |
| ----------- | ----- |
| `SX127x init failed (not an SX127x (bad version))` | SPI wiring wrong, module not powered, or 5 V logic. Check MISO/MOSI/SCK/NSS/RST. |
| `no ACK for seq …` | master off / out of range / different frequency, SF, BW or sync word / no antenna. |
| `LoRa slave not available` | `/sdcard/lora.ini` contains `enabled = false` or the module did not answer. |
| `sd card` and LoRa both dead | power supply too weak – the PA draws up to ~120 mA while transmitting. |
