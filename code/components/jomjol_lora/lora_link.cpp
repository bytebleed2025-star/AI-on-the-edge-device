/**
 * @file lora_link.cpp
 * @brief ESP-IDF glue for the LoRa slave: SPI setup, /sdcard/lora.ini,
 *        packet building, ACK handling.
 */
#ifdef ENABLE_LORA

#include "lora_link.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cctype>
#include <string>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "sx127x.h"
#include "lora_protocol.h"
#include "read_wlanini.h"

static const char *TAG = "LORA";

#define LORA_INI_FILE   "/sdcard/lora.ini"
#define LORA_SPI_HOST   SPI2_HOST
#define LORA_SPI_CLOCK  8000000

/* ------------------------------------------------------------------------- */
/* settings                                                                   */
/* ------------------------------------------------------------------------- */

typedef struct
{
    bool     enabled;

    uint32_t frequency_hz;
    uint8_t  spreading_factor;
    uint32_t bandwidth_hz;
    uint8_t  coding_rate;
    int8_t   tx_power_dbm;
    uint8_t  sync_word;
    uint16_t preamble_len;

    bool     ack;
    uint16_t ack_timeout_ms;
    uint8_t  retries;

    int8_t   pin_sck;
    int8_t   pin_miso;
    int8_t   pin_mosi;
    int8_t   pin_nss;
    int8_t   pin_reset;
    int8_t   pin_dio0;      /* -1 = not wired, IRQ flags are polled */
} lora_settings_t;

static lora_settings_t s_cfg;
static sx127x_t s_radio;
static bool s_ready = false;
static bool s_ini_loaded = false;
static spi_device_handle_t s_spi = NULL;
static SemaphoreHandle_t s_mutex = NULL;
static uint8_t s_seq = 0;
static lora_link_stats_t s_stats = {};

/* ------------------------------------------------------------------------- */
/* small helpers                                                              */
/* ------------------------------------------------------------------------- */

static void settings_defaults(lora_settings_t *s)
{
    memset(s, 0, sizeof(*s));
    s->enabled = true;

    s->frequency_hz = 433000000UL;
    s->spreading_factor = 9;
    s->bandwidth_hz = 125000;
    s->coding_rate = 5;
    s->tx_power_dbm = 17;
    s->sync_word = 0x12;
    s->preamble_len = 8;

    s->ack = true;
    s->ack_timeout_ms = 1500;
    s->retries = 2;

    s->pin_sck = 14;
    s->pin_miso = 21;
    s->pin_mosi = 41;
    s->pin_nss = 42;
    s->pin_reset = 47;
    s->pin_dio0 = -1;
}

static std::string trim(const std::string &in)
{
    size_t b = in.find_first_not_of(" \t\r\n\"");
    size_t e = in.find_last_not_of(" \t\r\n\"");

    if (b == std::string::npos)
        return "";
    return in.substr(b, e - b + 1);
}

static bool parse_bool(const std::string &v)
{
    std::string s;
    for (char c : v)
        s += (char)tolower((unsigned char)c);

    return (s == "1" || s == "true" || s == "yes" || s == "on");
}

static void load_ini(lora_settings_t *s)
{
    FILE *f = fopen(LORA_INI_FILE, "r");
    if (!f)
    {
        ESP_LOGW(TAG, "%s not found - using built in defaults", LORA_INI_FILE);
        return;
    }

    char line[256];
    while (fgets(line, sizeof(line), f))
    {
        std::string raw(line);

        size_t comment = raw.find_first_of(";#");
        if (comment != std::string::npos)
            raw = raw.substr(0, comment);

        size_t eq = raw.find('=');
        if (eq == std::string::npos)
            continue;

        std::string key = trim(raw.substr(0, eq));
        std::string val = trim(raw.substr(eq + 1));

        for (char &c : key)
            c = (char)toupper((unsigned char)c);

        if (key.empty() || val.empty())
            continue;

        long num = strtol(val.c_str(), NULL, 0);

        if (key == "ENABLED")                   s->enabled = parse_bool(val);
        else if (key == "FREQUENCY")            s->frequency_hz = (uint32_t)num;
        else if (key == "SPREADING_FACTOR")     s->spreading_factor = (uint8_t)num;
        else if (key == "BANDWIDTH")            s->bandwidth_hz = (uint32_t)num;
        else if (key == "CODING_RATE")          s->coding_rate = (uint8_t)num;
        else if (key == "TX_POWER")             s->tx_power_dbm = (int8_t)num;
        else if (key == "SYNC_WORD")            s->sync_word = (uint8_t)num;
        else if (key == "PREAMBLE")             s->preamble_len = (uint16_t)num;
        else if (key == "ACK")                  s->ack = parse_bool(val);
        else if (key == "ACK_TIMEOUT_MS")       s->ack_timeout_ms = (uint16_t)num;
        else if (key == "RETRIES")              s->retries = (uint8_t)num;
        else if (key == "PIN_SCK")              s->pin_sck = (int8_t)num;
        else if (key == "PIN_MISO")             s->pin_miso = (int8_t)num;
        else if (key == "PIN_MOSI")             s->pin_mosi = (int8_t)num;
        else if (key == "PIN_NSS")              s->pin_nss = (int8_t)num;
        else if (key == "PIN_RESET")            s->pin_reset = (int8_t)num;
        else if (key == "PIN_DIO0")             s->pin_dio0 = (int8_t)num;
        else
            ESP_LOGW(TAG, "unknown key in %s: %s", LORA_INI_FILE, key.c_str());
    }

    fclose(f);
    s_ini_loaded = true;
}

/* ------------------------------------------------------------------------- */
/* HAL implementation                                                         */
/* ------------------------------------------------------------------------- */

static int hal_spi_xfer(void *ctx, const uint8_t *tx, uint8_t *rx, size_t len)
{
    (void)ctx;

    if (!s_spi || len == 0)
        return -1;

    spi_transaction_t t = {};
    t.length = len * 8;
    t.tx_buffer = tx;       /* NULL sends zeros, which is what reads need */
    t.rx_buffer = rx;

    return (spi_device_polling_transmit(s_spi, &t) == ESP_OK) ? 0 : -1;
}

static void hal_cs(void *ctx, int level)
{
    (void)ctx;
    gpio_set_level((gpio_num_t)s_cfg.pin_nss, level ? 1 : 0);
}

static void hal_reset(void *ctx, int level)
{
    (void)ctx;
    gpio_set_level((gpio_num_t)s_cfg.pin_reset, level ? 1 : 0);
}

static int hal_dio0(void *ctx)
{
    (void)ctx;
    return gpio_get_level((gpio_num_t)s_cfg.pin_dio0);
}

static void hal_delay_ms(void *ctx, uint32_t ms)
{
    (void)ctx;
    vTaskDelay(pdMS_TO_TICKS(ms ? ms : 1));
}

/* ------------------------------------------------------------------------- */
/* init                                                                       */
/* ------------------------------------------------------------------------- */

static bool setup_gpio(void)
{
    gpio_config_t io = {};
    io.mode = GPIO_MODE_OUTPUT;
    io.pull_up_en = GPIO_PULLUP_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type = GPIO_INTR_DISABLE;

    io.pin_bit_mask = (1ULL << s_cfg.pin_nss);
    if (s_cfg.pin_reset >= 0)
        io.pin_bit_mask |= (1ULL << s_cfg.pin_reset);

    esp_err_t err = gpio_config(&io);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "gpio_config failed: %s", esp_err_to_name(err));
        return false;
    }

    gpio_set_level((gpio_num_t)s_cfg.pin_nss, 1);      /* CS idle high */
    if (s_cfg.pin_reset >= 0)
        gpio_set_level((gpio_num_t)s_cfg.pin_reset, 1);

    if (s_cfg.pin_dio0 >= 0)
    {
        gpio_config_t dio = {};
        dio.mode = GPIO_MODE_INPUT;
        dio.pin_bit_mask = (1ULL << s_cfg.pin_dio0);
        dio.pull_up_en = GPIO_PULLUP_DISABLE;
        dio.pull_down_en = GPIO_PULLDOWN_ENABLE;
        dio.intr_type = GPIO_INTR_DISABLE;
        gpio_config(&dio);
    }

    return true;
}

bool LoRaLink_Init(void)
{
    if (s_ready)
        return true;

    settings_defaults(&s_cfg);
    load_ini(&s_cfg);

    if (!s_cfg.enabled)
    {
        ESP_LOGI(TAG, "LoRa disabled in %s", LORA_INI_FILE);
        return false;
    }

    if (!setup_gpio())
        return false;

    /* --- SPI bus ------------------------------------------------------- */
    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = s_cfg.pin_mosi;
    buscfg.miso_io_num = s_cfg.pin_miso;
    buscfg.sclk_io_num = s_cfg.pin_sck;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 64;

    esp_err_t err = spi_bus_initialize(LORA_SPI_HOST, &buscfg, SPI_DMA_DISABLED);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(err));
        return false;
    }

    spi_device_interface_config_t devcfg = {};
    devcfg.clock_speed_hz = LORA_SPI_CLOCK;
    devcfg.mode = 0;
    devcfg.spics_io_num = -1;          /* chip select is driven in hal_cs() */
    devcfg.queue_size = 1;

    err = spi_bus_add_device(LORA_SPI_HOST, &devcfg, &s_spi);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "spi_bus_add_device failed: %s", esp_err_to_name(err));
        return false;
    }

    /* --- radio --------------------------------------------------------- */
    sx127x_hal_t hal = {};
    hal.ctx = NULL;
    hal.spi_xfer = hal_spi_xfer;
    hal.cs = hal_cs;
    hal.reset = (s_cfg.pin_reset >= 0) ? hal_reset : NULL;
    hal.dio0 = (s_cfg.pin_dio0 >= 0) ? hal_dio0 : NULL;
    hal.delay_ms = hal_delay_ms;

    sx127x_config_t radio_cfg = SX127X_DEFAULT_CONFIG;
    radio_cfg.frequency_hz = s_cfg.frequency_hz;
    radio_cfg.spreading_factor = s_cfg.spreading_factor;
    radio_cfg.bandwidth_hz = s_cfg.bandwidth_hz;
    radio_cfg.coding_rate = s_cfg.coding_rate;
    radio_cfg.tx_power_dbm = s_cfg.tx_power_dbm;
    radio_cfg.sync_word = s_cfg.sync_word;
    radio_cfg.preamble_len = s_cfg.preamble_len;
    radio_cfg.crc_on = true;
    radio_cfg.tx_timeout_ms = 5000;

    sx127x_result_t res = sx127x_init(&s_radio, &hal, &radio_cfg);
    if (res != SX127X_OK)
    {
        ESP_LOGE(TAG, "SX127x init failed (%s). Check wiring: SCK=%d MISO=%d MOSI=%d NSS=%d RST=%d",
                 sx127x_result_str(res), s_cfg.pin_sck, s_cfg.pin_miso, s_cfg.pin_mosi,
                 s_cfg.pin_nss, s_cfg.pin_reset);
        s_ready = false;
        return false;
    }

    if (!s_mutex)
        s_mutex = xSemaphoreCreateMutex();

    s_ready = true;

    ESP_LOGI(TAG, "RA-02 ready: %.3f MHz, SF%u, BW%lu kHz, CR4/%u, %d dBm, sync 0x%02X, ACK %s",
             (double)s_cfg.frequency_hz / 1e6, s_cfg.spreading_factor,
             (unsigned long)(s_cfg.bandwidth_hz / 1000), s_cfg.coding_rate,
             s_cfg.tx_power_dbm, s_cfg.sync_word, s_cfg.ack ? "on" : "off");

    return true;
}

bool LoRaLink_IsReady(void)
{
    return s_ready;
}

/* ------------------------------------------------------------------------- */
/* sending                                                                    */
/* ------------------------------------------------------------------------- */

static void device_id(char *out, size_t out_len)
{
    memset(out, 0, out_len);

    std::string id = wlan_config.hostname;
    if (id.empty())
        id = "watermeter";

    if (id.size() > out_len - 1)
        id.resize(out_len - 1);

    memcpy(out, id.c_str(), id.size());
}

static int decimals_of(const char *value)
{
    const char *dot = strchr(value, '.');
    if (!dot)
        return 0;

    int d = 0;
    for (const char *p = dot + 1; *p; ++p)
        if (isdigit((unsigned char)*p))
            d++;

    return d > 9 ? 9 : d;
}

/* Wait for an ACK that belongs to seq, drop everything else (e.g. stale ACKs) */
static bool wait_for_ack(uint8_t seq, uint32_t timeout_ms)
{
    if (!s_cfg.ack)
        return true;

    uint8_t buf[LORA_MAX_PAYLOAD];
    uint8_t len = 0;
    int64_t start = esp_timer_get_time();
    int64_t timeout_us = (int64_t)timeout_ms * 1000;

    while ((esp_timer_get_time() - start) < timeout_us)
    {
        int res = sx127x_poll_rx(&s_radio, buf, sizeof(buf), &len);

        if (res == 1)
        {
            lora_packet_t pkt;
            if (lora_packet_open(buf, len, &pkt))
            {
                if (pkt.type == LORA_TYPE_ACK && pkt.seq == seq)
                {
                    s_stats.last_rssi = s_radio.last_rssi;
                    s_stats.last_snr = s_radio.last_snr;
                    return true;
                }
                ESP_LOGD(TAG, "ignoring packet type %u seq %u", pkt.type, pkt.seq);
            }
        }
        else if (res == SX127X_ERR_CRC)
        {
            ESP_LOGW(TAG, "received packet with CRC error");
        }

        vTaskDelay(pdMS_TO_TICKS(2));
    }

    return false;
}

bool LoRaLink_SendReading(const char *sensor, const char *value, uint32_t timestamp, bool error)
{
    if (!s_ready || !value || value[0] == '\0')
        return false;

    if (s_mutex && xSemaphoreTake(s_mutex, pdMS_TO_TICKS(10000)) != pdTRUE)
    {
        ESP_LOGW(TAG, "mutex timeout, dropping reading");
        return false;
    }

    lora_packet_t pkt = {};
    device_id(pkt.id, sizeof(pkt.id));

    snprintf(pkt.sensor, sizeof(pkt.sensor), "%s", (sensor && sensor[0]) ? sensor : "default");

    pkt.type = LORA_TYPE_READING;
    pkt.value = (float)atof(value);
    pkt.timestamp = timestamp;
    pkt.decimals = (uint8_t)decimals_of(value);
    pkt.flags = LORA_FLAG_ACK_REQ;
    if (error)
        pkt.flags |= LORA_FLAG_ERROR;

    s_seq++;
    pkt.seq = s_seq;

    bool delivered = false;
    uint8_t attempts = (uint8_t)(s_cfg.retries + 1);

    sx127x_start_rx(&s_radio);

    for (uint8_t attempt = 1; attempt <= attempts && !delivered; ++attempt)
    {
        lora_packet_seal(&pkt);

        if (attempt > 1)
            s_stats.retries++;

        sx127x_result_t res = sx127x_transmit(&s_radio, (const uint8_t *)&pkt, sizeof(pkt));
        if (res != SX127X_OK)
        {
            ESP_LOGW(TAG, "transmit failed (attempt %u/%u): %s", attempt, attempts,
                     sx127x_result_str(res));
            continue;
        }

        s_stats.sent++;

        if (!s_cfg.ack)
        {
            delivered = true;
            break;
        }

        /* stay in RX while waiting for the ACK */
        sx127x_start_rx(&s_radio);

        if (wait_for_ack(pkt.seq, s_cfg.ack_timeout_ms))
        {
            delivered = true;
            s_stats.acked++;
            break;
        }

        ESP_LOGW(TAG, "no ACK for seq %u (attempt %u/%u)", pkt.seq, attempt, attempts);
    }

    if (!delivered)
        s_stats.failed++;

    if (delivered && s_cfg.ack)
        ESP_LOGI(TAG, "sent %s=%s (seq %u, rssi %d dBm, snr %d dB)",
                 pkt.sensor, value, pkt.seq, s_stats.last_rssi, s_stats.last_snr);
    else if (delivered)
        ESP_LOGI(TAG, "sent %s=%s (seq %u)", pkt.sensor, value, pkt.seq);

    if (s_mutex)
        xSemaphoreGive(s_mutex);

    return delivered;
}

void LoRaLink_GetStats(lora_link_stats_t *out)
{
    if (!out)
        return;
    *out = s_stats;
}

void LoRaLink_Sleep(void)
{
    if (s_ready)
        sx127x_sleep(&s_radio);
}

#endif // ENABLE_LORA
