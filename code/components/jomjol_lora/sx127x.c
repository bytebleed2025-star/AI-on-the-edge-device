/**
 * @file sx127x.c
 * @brief Minimal SX1276/77/78/79 LoRa driver, see sx127x.h
 */
#include "sx127x.h"

#include <math.h>
#include <string.h>

/* ------------------------------------------------------------------------- */
/* low level helpers                                                          */
/* ------------------------------------------------------------------------- */

#define SX127X_SPI_READ  0x00
#define SX127X_SPI_WRITE 0x80

static void sx127x_delay(sx127x_t *dev, uint32_t ms)
{
    if (dev->hal.delay_ms)
        dev->hal.delay_ms(dev->hal.ctx, ms);
}

uint8_t sx127x_read_reg(sx127x_t *dev, uint8_t addr)
{
    uint8_t tx[2] = {(uint8_t)(addr & 0x7F), 0x00};
    uint8_t rx[2] = {0x00, 0x00};

    if (!dev->hal.spi_xfer)
        return 0;

    if (dev->hal.cs)
        dev->hal.cs(dev->hal.ctx, 0);
    dev->hal.spi_xfer(dev->hal.ctx, tx, rx, sizeof(tx));
    if (dev->hal.cs)
        dev->hal.cs(dev->hal.ctx, 1);

    return rx[1];
}

void sx127x_write_reg(sx127x_t *dev, uint8_t addr, uint8_t value)
{
    uint8_t tx[2] = {(uint8_t)(addr | SX127X_SPI_WRITE), value};

    if (!dev->hal.spi_xfer)
        return;

    if (dev->hal.cs)
        dev->hal.cs(dev->hal.ctx, 0);
    dev->hal.spi_xfer(dev->hal.ctx, tx, NULL, sizeof(tx));
    if (dev->hal.cs)
        dev->hal.cs(dev->hal.ctx, 1);
}

void sx127x_read_burst(sx127x_t *dev, uint8_t addr, uint8_t *buf, size_t len)
{
    uint8_t header = (uint8_t)(addr & 0x7F);

    if (!dev->hal.spi_xfer || !buf || len == 0)
        return;

    if (dev->hal.cs)
        dev->hal.cs(dev->hal.ctx, 0);
    dev->hal.spi_xfer(dev->hal.ctx, &header, NULL, 1);
    dev->hal.spi_xfer(dev->hal.ctx, NULL, buf, len);
    if (dev->hal.cs)
        dev->hal.cs(dev->hal.ctx, 1);
}

void sx127x_write_burst(sx127x_t *dev, uint8_t addr, const uint8_t *buf, size_t len)
{
    uint8_t header = (uint8_t)(addr | SX127X_SPI_WRITE);

    if (!dev->hal.spi_xfer || !buf || len == 0)
        return;

    if (dev->hal.cs)
        dev->hal.cs(dev->hal.ctx, 0);
    dev->hal.spi_xfer(dev->hal.ctx, &header, NULL, 1);
    dev->hal.spi_xfer(dev->hal.ctx, buf, NULL, len);
    if (dev->hal.cs)
        dev->hal.cs(dev->hal.ctx, 1);
}

/* ------------------------------------------------------------------------- */
/* helpers                                                                    */
/* ------------------------------------------------------------------------- */

/* Note: the HAL spi_xfer() must accept tx == NULL or rx == NULL. For a
   write-only transfer the implementation sends zero bytes. */

uint8_t sx127x_bandwidth_code(uint32_t bandwidth_hz)
{
    switch (bandwidth_hz)
    {
        case 7800:   return 0;
        case 10400:  return 1;
        case 15600:  return 2;
        case 20800:  return 3;
        case 31250:  return 4;
        case 41700:  return 5;
        case 62500:  return 6;
        case 125000: return 7;
        case 250000: return 8;
        case 500000: return 9;
        default:     return 0xFF;
    }
}

static void sx127x_set_opmode(sx127x_t *dev, uint8_t mode)
{
    sx127x_write_reg(dev, SX127X_REG_OPMODE, (uint8_t)(SX127X_LONG_RANGE_MODE | mode));
}

static void sx127x_set_modem_config(sx127x_t *dev)
{
    /* ModemConfig1: BW[7:4] | CR[3:1] | implicitHeader[0] */
    uint8_t cfg1 = (uint8_t)((dev->bandwidth_code << 4) |
                             ((dev->coding_rate & 0x07) << 1));

    /* ModemConfig2: SF[7:4] | TxContinuous[3] | RxPayloadCrcOn[2] */
    uint8_t cfg2 = (uint8_t)((dev->spreading_factor << 4) | 0x04);

    sx127x_write_reg(dev, SX127X_REG_MODEM_CONFIG_1, cfg1);
    sx127x_write_reg(dev, SX127X_REG_MODEM_CONFIG_2, cfg2);

    /* AGC on, low data rate optimize for long symbols (SF11/SF12 at BW125) */
    bool ldr = (dev->spreading_factor >= 11 && dev->bandwidth_code <= 7);
    sx127x_write_reg(dev, SX127X_REG_MODEM_CONFIG_3, (uint8_t)(0x04 | (ldr ? 0x08 : 0x00)));

    /* symbol timeout: 4 symbols -> rx single timeouts quickly */
    sx127x_write_reg(dev, SX127X_REG_SYMB_TIMEOUT_LSB, 0x00);
}

uint32_t sx127x_time_on_air_ms(const sx127x_t *dev, uint8_t payload_len)
{
    static const uint32_t bw_hz[10] = {7800, 10400, 15600, 20800, 31250,
                                      41700, 62500, 125000, 250000, 500000};

    if (dev->bandwidth_code > 9 || dev->spreading_factor < 6 || dev->spreading_factor > 12)
        return 0;

    double bw = (double)bw_hz[dev->bandwidth_code];
    double ts = (double)(1u << dev->spreading_factor) / bw;   /* symbol time in s */
    int sf = dev->spreading_factor;

    /* explicit header, CRC on */
    double preamble = (8.0 + 4.25) * ts;
    double numerator = 8.0 * (double)payload_len - 4.0 * (double)sf + 28.0 + 16.0;
    double denom = 4.0 * (double)(sf - 2);
    int n_payload = (int)ceil(numerator / denom);
    if (n_payload < 0)
        n_payload = 0;
    n_payload *= (dev->coding_rate - 4);   /* CR 5..8 -> 4/5 .. 4/8 */
    double payload_time = (double)(8 + n_payload) * ts;

    return (uint32_t)((preamble + payload_time) * 1000.0) + 1;
}

const char *sx127x_result_str(sx127x_result_t res)
{
    switch (res)
    {
        case SX127X_OK:            return "ok";
        case SX127X_ERR_SPI:       return "SPI error";
        case SX127X_ERR_NOT_SX127X:return "not an SX127x (bad version)";
        case SX127X_ERR_PARAM:     return "invalid parameter";
        case SX127X_ERR_TIMEOUT:   return "timeout";
        case SX127X_ERR_CRC:       return "CRC error";
        default:                   return "unknown";
    }
}

/* ------------------------------------------------------------------------- */
/* public API                                                                 */
/* ------------------------------------------------------------------------- */

sx127x_result_t sx127x_set_frequency(sx127x_t *dev, uint32_t frequency_hz)
{
    if (frequency_hz < 137000000UL || frequency_hz > 1020000000UL)
        return SX127X_ERR_PARAM;

    /* Frf = f * 2^19 / 32 MHz */
    uint64_t frf = ((uint64_t)frequency_hz << 19) / 32000000ULL;

    sx127x_write_reg(dev, SX127X_REG_FRF_MSB, (uint8_t)(frf >> 16));
    sx127x_write_reg(dev, SX127X_REG_FRF_MID, (uint8_t)(frf >> 8));
    sx127x_write_reg(dev, SX127X_REG_FRF_LSB, (uint8_t)(frf >> 0));

    dev->frequency_hz = frequency_hz;
    return SX127X_OK;
}

sx127x_result_t sx127x_set_tx_power(sx127x_t *dev, int8_t dbm)
{
    if (dbm < 2)
        dbm = 2;
    if (dbm > 20)
        dbm = 20;

    /* RA-02 uses the PA_BOOST pin. +20 dBm needs the high power DAC. */
    if (dbm > 17)
    {
        sx127x_write_reg(dev, SX127X_REG_PA_DAC, 0x87);   /* +20 dBm */
        if (dbm > 20)
            dbm = 20;
        dbm = (int8_t)(dbm - 3);
    }
    else
    {
        sx127x_write_reg(dev, SX127X_REG_PA_DAC, 0x84);   /* +17 dBm */
    }

    sx127x_write_reg(dev, SX127X_REG_PACONFIG, (uint8_t)(0x80 | (dbm - 2)));  /* PA_BOOST | power */
    sx127x_write_reg(dev, SX127X_REG_OCP, 0x2B);                              /* OCP on, ~210 mA */

    dev->tx_power_dbm = dbm;
    return SX127X_OK;
}

sx127x_result_t sx127x_apply_config(sx127x_t *dev)
{
    if (!dev || !dev->initialised)
        return SX127X_ERR_PARAM;

    sx127x_set_opmode(dev, SX127X_MODE_STDBY);

    sx127x_write_reg(dev, SX127X_REG_FIFO_TX_BASE_ADDR, 0x00);
    sx127x_write_reg(dev, SX127X_REG_FIFO_RX_BASE_ADDR, 0x00);
    sx127x_set_frequency(dev, dev->frequency_hz);
    sx127x_set_modem_config(dev);

    sx127x_write_reg(dev, SX127X_REG_PREAMBLE_MSB, 0x00);
    sx127x_write_reg(dev, SX127X_REG_PREAMBLE_LSB, 0x08);
    sx127x_write_reg(dev, SX127X_REG_SYNC_WORD, 0x12);
    sx127x_write_reg(dev, SX127X_REG_INVERT_IQ, 0x27);
    sx127x_write_reg(dev, SX127X_REG_DETECT_OPTIMIZE,
                     (uint8_t)(dev->spreading_factor == 6 ? 0x05 : 0x03));
    sx127x_write_reg(dev, SX127X_REG_DETECTION_THRESHOLD,
                     (uint8_t)(dev->spreading_factor == 6 ? 0x0C : 0x0A));
    sx127x_write_reg(dev, SX127X_REG_LNA, 0x23);   /* LNA boost HF + max gain */
    sx127x_write_reg(dev, SX127X_REG_PARAMP, 0x09);

    /* DIO0 -> RxDone (mapping 00); TX is polled anyway */
    sx127x_write_reg(dev, SX127X_REG_DIO_MAPPING_1, 0x00);

    sx127x_set_tx_power(dev, dev->tx_power_dbm);

    sx127x_write_reg(dev, SX127X_REG_IRQ_FLAGS, SX127X_IRQ_ALL);   /* clear */
    sx127x_write_reg(dev, SX127X_REG_IRQ_FLAGS_MASK, 0x00);        /* all enabled */

    return SX127X_OK;
}

sx127x_result_t sx127x_init(sx127x_t *dev, const sx127x_hal_t *hal, const sx127x_config_t *cfg)
{
    if (!dev || !hal || !hal->spi_xfer || !cfg)
        return SX127X_ERR_PARAM;

    memset(dev, 0, sizeof(*dev));
    dev->hal = *hal;

    uint8_t bw = sx127x_bandwidth_code(cfg->bandwidth_hz);
    if (bw == 0xFF)
        return SX127X_ERR_PARAM;
    if (cfg->spreading_factor < 6 || cfg->spreading_factor > 12)
        return SX127X_ERR_PARAM;
    if (cfg->coding_rate < 5 || cfg->coding_rate > 8)
        return SX127X_ERR_PARAM;

    /* hardware reset: 10 ms low, then wait for the oscillator */
    if (dev->hal.reset)
    {
        dev->hal.reset(dev->hal.ctx, 1);
        sx127x_delay(dev, 2);
        dev->hal.reset(dev->hal.ctx, 0);
        sx127x_delay(dev, 10);
        dev->hal.reset(dev->hal.ctx, 1);
        sx127x_delay(dev, 10);
    }

    uint8_t version = sx127x_read_reg(dev, SX127X_REG_VERSION);
    if (version != 0x12)
        return SX127X_ERR_NOT_SX127X;

    /* switch to LoRa mode first, then to standby */
    sx127x_set_opmode(dev, SX127X_MODE_SLEEP);
    sx127x_delay(dev, 2);
    sx127x_set_opmode(dev, SX127X_MODE_STDBY);
    sx127x_delay(dev, 2);

    dev->frequency_hz = cfg->frequency_hz;
    dev->spreading_factor = cfg->spreading_factor;
    dev->bandwidth_code = bw;
    dev->coding_rate = cfg->coding_rate;
    dev->tx_power_dbm = cfg->tx_power_dbm;
    dev->tx_timeout_ms = cfg->tx_timeout_ms ? cfg->tx_timeout_ms : 5000;
    dev->initialised = true;

    sx127x_result_t res = sx127x_apply_config(dev);
    if (res != SX127X_OK)
        return res;

    /* optional settings that are not part of apply_config() */
    if (cfg->sync_word != 0x12)
        sx127x_write_reg(dev, SX127X_REG_SYNC_WORD, cfg->sync_word);

    uint16_t preamble = cfg->preamble_len ? cfg->preamble_len : 8;
    sx127x_write_reg(dev, SX127X_REG_PREAMBLE_MSB, (uint8_t)(preamble >> 8));
    sx127x_write_reg(dev, SX127X_REG_PREAMBLE_LSB, (uint8_t)(preamble & 0xFF));

    if (!cfg->crc_on)
        sx127x_write_reg(dev, SX127X_REG_MODEM_CONFIG_2, (uint8_t)(dev->spreading_factor << 4));

    return SX127X_OK;
}

sx127x_result_t sx127x_sleep(sx127x_t *dev)
{
    if (!dev || !dev->initialised)
        return SX127X_ERR_PARAM;
    sx127x_set_opmode(dev, SX127X_MODE_SLEEP);
    return SX127X_OK;
}

sx127x_result_t sx127x_standby(sx127x_t *dev)
{
    if (!dev || !dev->initialised)
        return SX127X_ERR_PARAM;
    sx127x_set_opmode(dev, SX127X_MODE_STDBY);
    return SX127X_OK;
}

sx127x_result_t sx127x_transmit(sx127x_t *dev, const uint8_t *data, uint8_t len)
{
    if (!dev || !dev->initialised || !data || len == 0)
        return SX127X_ERR_PARAM;

    sx127x_set_opmode(dev, SX127X_MODE_STDBY);
    sx127x_write_reg(dev, SX127X_REG_IRQ_FLAGS, SX127X_IRQ_ALL);

    sx127x_write_reg(dev, SX127X_REG_FIFO_TX_BASE_ADDR, 0x00);
    sx127x_write_reg(dev, SX127X_REG_FIFO_ADDR_PTR, 0x00);
    sx127x_write_burst(dev, SX127X_REG_FIFO, data, len);
    sx127x_write_reg(dev, SX127X_REG_PAYLOAD_LENGTH, len);

    sx127x_set_opmode(dev, SX127X_MODE_TX);

    /* Wait for TxDone. The airtime depends on SF/BW, use the configured
       timeout plus the computed airtime so slow settings still work. */
    uint32_t toa = sx127x_time_on_air_ms(dev, len);
    uint32_t deadline = dev->tx_timeout_ms + toa;
    uint32_t waited = 0;

    while (waited < deadline)
    {
        if (sx127x_read_reg(dev, SX127X_REG_IRQ_FLAGS) & SX127X_IRQ_TX_DONE)
        {
            sx127x_write_reg(dev, SX127X_REG_IRQ_FLAGS, SX127X_IRQ_ALL);
            sx127x_set_opmode(dev, SX127X_MODE_STDBY);
            return SX127X_OK;
        }
        if (dev->hal.dio0 && dev->hal.dio0(dev->hal.ctx))
        {
            /* DIO0 is mapped to RxDone by default; ignore, keep polling */
        }
        sx127x_delay(dev, 1);
        waited += 1;
    }

    sx127x_write_reg(dev, SX127X_REG_IRQ_FLAGS, SX127X_IRQ_ALL);
    sx127x_set_opmode(dev, SX127X_MODE_STDBY);
    return SX127X_ERR_TIMEOUT;
}

sx127x_result_t sx127x_start_rx(sx127x_t *dev)
{
    if (!dev || !dev->initialised)
        return SX127X_ERR_PARAM;

    sx127x_set_opmode(dev, SX127X_MODE_STDBY);
    sx127x_write_reg(dev, SX127X_REG_IRQ_FLAGS, SX127X_IRQ_ALL);
    sx127x_write_reg(dev, SX127X_REG_FIFO_RX_BASE_ADDR, 0x00);
    sx127x_write_reg(dev, SX127X_REG_FIFO_ADDR_PTR, 0x00);
    sx127x_set_opmode(dev, SX127X_MODE_RX_CONT);

    return SX127X_OK;
}

int sx127x_poll_rx(sx127x_t *dev, uint8_t *buf, uint8_t max_len, uint8_t *out_len)
{
    if (!dev || !dev->initialised || !buf)
        return SX127X_ERR_PARAM;

    uint8_t flags = sx127x_read_reg(dev, SX127X_REG_IRQ_FLAGS);

    if (!(flags & SX127X_IRQ_RX_DONE))
    {
        /* CRC error alone also sets RxDone, so nothing to do here */
        return 0;
    }

    sx127x_write_reg(dev, SX127X_REG_IRQ_FLAGS, SX127X_IRQ_ALL);

    if (flags & SX127X_IRQ_CRC_ERROR)
    {
        sx127x_start_rx(dev);
        return SX127X_ERR_CRC;
    }

    uint8_t len = sx127x_read_reg(dev, SX127X_REG_RX_NB_BYTES);
    uint8_t addr = sx127x_read_reg(dev, SX127X_REG_FIFO_RX_CURRENT_ADDR);

    int8_t rssi = (int8_t)sx127x_read_reg(dev, SX127X_REG_PKT_RSSI_VALUE);
    int8_t snr_raw = (int8_t)sx127x_read_reg(dev, SX127X_REG_PKT_SNR_VALUE);

    dev->last_rssi = (int8_t)(rssi - 157);            /* HF port (433 MHz) */
    dev->last_snr = (int8_t)(snr_raw / 4);

    if (len > max_len)
        len = max_len;

    if (len > 0)
    {
        sx127x_write_reg(dev, SX127X_REG_FIFO_ADDR_PTR, addr);
        sx127x_read_burst(dev, SX127X_REG_FIFO, buf, len);
    }

    if (out_len)
        *out_len = len;

    /* go back to continuous RX for the next packet */
    sx127x_start_rx(dev);
    return 1;
}

int sx127x_wait_rx(sx127x_t *dev, uint8_t *buf, uint8_t max_len, uint8_t *out_len,
                   uint32_t timeout_ms)
{
    uint32_t waited = 0;

    while (waited <= timeout_ms)
    {
        int res = sx127x_poll_rx(dev, buf, max_len, out_len);
        if (res != 0)
            return res;

        sx127x_delay(dev, 1);
        waited += 1;
    }

    return 0;
}
