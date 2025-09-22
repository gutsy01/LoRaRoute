#include "lora_sx1276.h"
#include "board.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"   // [PATCH] vTaskDelay
#include "freertos/task.h"       // [PATCH] vTaskDelay
#include <cstring>
#include <cmath>

static const char* TAG = "SX1276";

// [PATCH] Allow selecting SPI host from board.h, default to HSPI (SPI2_HOST)
#ifndef LORA_SPI_HOST
#define LORA_SPI_HOST SPI2_HOST
#endif

// ====== Register & konstanta penting ======
static constexpr uint8_t REG_FIFO              = 0x00;
static constexpr uint8_t REG_OP_MODE           = 0x01;
static constexpr uint8_t REG_FRF_MSB           = 0x06;
static constexpr uint8_t REG_FRF_MID           = 0x07;
static constexpr uint8_t REG_FRF_LSB           = 0x08;

static constexpr uint8_t REG_PA_CONFIG         = 0x09;
static constexpr uint8_t REG_LNA               = 0x0C;
static constexpr uint8_t REG_FIFO_ADDR_PTR     = 0x0D;
static constexpr uint8_t REG_FIFO_TX_BASE_ADDR = 0x0E;
static constexpr uint8_t REG_FIFO_RX_BASE_ADDR = 0x0F;
static constexpr uint8_t REG_FIFO_RX_CURRENT   = 0x10;
static constexpr uint8_t REG_IRQ_FLAGS         = 0x12;
static constexpr uint8_t REG_RX_NB_BYTES       = 0x13;
static constexpr uint8_t REG_MODEM_CONFIG1     = 0x1D;
static constexpr uint8_t REG_MODEM_CONFIG2     = 0x1E;
static constexpr uint8_t REG_PREAMBLE_MSB      = 0x20;
static constexpr uint8_t REG_PREAMBLE_LSB      = 0x21;
static constexpr uint8_t REG_PAYLOAD_LENGTH    = 0x22;
static constexpr uint8_t REG_MODEM_CONFIG3     = 0x26;
static constexpr uint8_t REG_PKT_RSSI_VALUE    = 0x1A;
static constexpr uint8_t REG_DIO_MAPPING1      = 0x40;
static constexpr uint8_t REG_VERSION           = 0x42;

static constexpr uint8_t MODE_LONG_RANGE_MODE  = 0x80; // bit7=1 LoRa
static constexpr uint8_t MODE_SLEEP            = 0x00;
static constexpr uint8_t MODE_STDBY            = 0x01;
static constexpr uint8_t MODE_TX               = 0x03;
static constexpr uint8_t MODE_RX_CONTINUOUS    = 0x05;

static constexpr uint8_t IRQ_TX_DONE_MASK      = 0x08;
static constexpr uint8_t IRQ_RX_DONE_MASK      = 0x40;
static constexpr uint8_t IRQ_PAYLOAD_CRC_ERR   = 0x20;

static constexpr double  F_XOSC = 32e6;
static constexpr double  FSTEP  = F_XOSC / (1 << 19); // 61.03515625 Hz

// ====== SPI handle ======
static spi_device_handle_t s_spi;
static uint8_t s_rx_buf[256];
static int     s_rx_len = 0;
static int     s_rx_idx = 0;
static int     s_last_rssi = -127;

static inline void delay_ms(uint32_t ms) {
  vTaskDelay(pdMS_TO_TICKS(ms));
}

static void write_reg(uint8_t addr, uint8_t value) {
  uint8_t tx[2] = { (uint8_t)(addr | 0x80), value };
  spi_transaction_t t{};
  t.length = 16;
  t.tx_buffer = tx;
  spi_device_transmit(s_spi, &t);
}

static uint8_t read_reg(uint8_t addr) {
  uint8_t tx[2] = { (uint8_t)(addr & 0x7F), 0x00 };
  uint8_t rx[2] = { 0, 0 };
  spi_transaction_t t{};
  t.length = 16;
  t.tx_buffer = tx;
  t.rx_buffer = rx;
  spi_device_transmit(s_spi, &t);
  return rx[1];
}

// GANTI seluruh burst_write & burst_read dengan ini:

static void burst_write(uint8_t addr, const uint8_t* data, size_t len) {
  if (len > 256) len = 256;
  uint8_t buf[1 + 256];
  buf[0] = (uint8_t)(addr | 0x80);     // bit7=1 -> write
  if (len && data) memcpy(&buf[1], data, len);

  spi_transaction_t t{};
  t.length   = (1 + len) * 8;          // alamat + payload
  t.tx_buffer = buf;
  spi_device_transmit(s_spi, &t);
}

static void burst_read(uint8_t addr, uint8_t* data, size_t len) {
  if (len > 256) len = 256;
  uint8_t tx[1 + 256] = {0};
  uint8_t rx[1 + 256] = {0};

  tx[0] = (uint8_t)(addr & 0x7F);      // bit7=0 -> read

  spi_transaction_t t{};
  t.length    = (1 + len) * 8;         // alamat + dummy untuk clocking data
  t.tx_buffer = tx;
  t.rx_buffer = rx;
  spi_device_transmit(s_spi, &t);

  if (len && data) memcpy(data, &rx[1], len);  // lewati byte alamat
}


static void reset_chip() {
  gpio_set_direction((gpio_num_t)LORA_RST, GPIO_MODE_OUTPUT);
  gpio_set_level((gpio_num_t)LORA_RST, 0);
  delay_ms(2);
  gpio_set_level((gpio_num_t)LORA_RST, 1);
  delay_ms(10);
}

static void set_opmode(uint8_t mode) {
  write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | mode);
}

static void set_frequency(uint32_t hz) {
  uint32_t frf = (uint32_t)std::llround((double)hz / FSTEP);
  write_reg(REG_FRF_MSB, (uint8_t)(frf >> 16));
  write_reg(REG_FRF_MID, (uint8_t)(frf >> 8));
  write_reg(REG_FRF_LSB, (uint8_t)(frf));
}

static void set_tx_power(int dbm) {
  // PA_BOOST path (pin PA_BOOST pada banyak modul)
  int p = dbm;
  if (p < 2)  p = 2;
  if (p > 17) p = 17;
  write_reg(REG_PA_CONFIG, (uint8_t)(0x80 | (p - 2))); // PA_BOOST + power
}

static void set_modem(uint8_t sf, uint32_t bw_hz) {
  // BW map (approx): 7=125k, 8=250k, 9=500k (bit 7..4)
  uint8_t bw;
  if (bw_hz >= 500000) bw = 9;
  else if (bw_hz >= 250000) bw = 8;
  else bw = 7; // 125k default

  // CodingRate 4/5 (001), Header explicit, CRC on (later)
  uint8_t mc1 = (bw << 4) | (1 << 1) | (0x00);
  write_reg(REG_MODEM_CONFIG1, mc1);

  if (sf < 6) sf = 6;
  if (sf > 12) sf = 12;

  // >>> CRC OFF: hapus (1<<2)
  uint8_t mc2 = ((sf << 4) | 0x03);
  write_reg(REG_MODEM_CONFIG2, mc2);

  // LowDataRateOptimize untuk SF11/12 @BW125
  uint8_t mc3 = 0;
  bool ldo = (bw_hz == 125000) && (sf >= 11);
  if (ldo) mc3 |= (1 << 3); // LowDataRateOptimize
  mc3 |= (1 << 2); // AgcAutoOn
  write_reg(REG_MODEM_CONFIG3, mc3);
}

bool sx1276_begin() {
  // ===== SPI init pakai pin dari board.h =====
  spi_bus_config_t bus{};
  bus.mosi_io_num = LORA_MOSI;   // [PATCH] contoh JUMA: 27
  bus.miso_io_num = LORA_MISO;   // [PATCH] contoh JUMA: 19
  bus.sclk_io_num = LORA_SCK;    // [PATCH] contoh JUMA: 5
  bus.quadwp_io_num = -1;
  bus.quadhd_io_num = -1;
  bus.max_transfer_sz = 4096;
  ESP_ERROR_CHECK(spi_bus_initialize(LORA_SPI_HOST, &bus, SPI_DMA_CH_AUTO));  // [PATCH]

  spi_device_interface_config_t dev{};
  dev.mode = 0;
  dev.clock_speed_hz = 8 * 1000 * 1000; // 8 MHz
  dev.spics_io_num = LORA_SS;           // [PATCH] contoh JUMA: 18 (CS otomatis)
  dev.queue_size = 4;
  ESP_ERROR_CHECK(spi_bus_add_device(LORA_SPI_HOST, &dev, &s_spi));            // [PATCH]

  // DIO0 (RxDone) sebagai input
  gpio_set_direction((gpio_num_t)LORA_DIO0, GPIO_MODE_INPUT);

  reset_chip();

  // cek versi (0x12 untuk SX1276/77/78/79)
  uint8_t ver = read_reg(REG_VERSION);
  if (ver == 0x00 || ver == 0xFF) {
    ESP_LOGE(TAG, "SX1276 not found (REG_VERSION=0x%02X)", ver);
    return false;
  }
  ESP_LOGI(TAG, "SX1276 REG_VERSION=0x%02X", ver);

  set_opmode(MODE_SLEEP);
  set_opmode(MODE_STDBY);

  // base address FIFO
  write_reg(REG_FIFO_TX_BASE_ADDR, 0x80);
  write_reg(REG_FIFO_RX_BASE_ADDR, 0x00);

  set_frequency((uint32_t)LORA_FREQ_HZ);
  set_tx_power(LORA_TX_POWER_DBM);
  set_modem(LORA_SF, (uint32_t)LORA_BW);

  // preamble default
  write_reg(REG_PREAMBLE_MSB, 0x00);
  write_reg(REG_PREAMBLE_LSB, 0x08);

  // map DIO0: RxDone(00) / TxDone(01) di bit 7..6
  write_reg(REG_DIO_MAPPING1, 0x00); // default RxDone pada RX

  // Masuk RX continuous
  set_opmode(MODE_RX_CONTINUOUS);
  return true;
}

// ========== TX API ==========
static uint8_t s_txbuf[256];
static size_t  s_txlen = 0;

void sx1276_begin_packet() {
  s_txlen = 0;
}

void sx1276_write(const char* data, size_t len) {
  if (!data || len == 0) return;
  size_t space = sizeof(s_txbuf) - s_txlen;
  if (len > space) len = space;
  memcpy(&s_txbuf[s_txlen], data, len);
  s_txlen += len;
}

void sx1276_end_packet() {
  // standby dulu
  set_opmode(MODE_STDBY);
  // Tambah: clear semua IRQ biar status bersih
  write_reg(REG_IRQ_FLAGS, 0xFF);
  // set FIFO addr TX
  write_reg(REG_FIFO_ADDR_PTR, read_reg(REG_FIFO_TX_BASE_ADDR));
  // tulis payload ke FIFO
  burst_write(REG_FIFO, s_txbuf, s_txlen);
  write_reg(REG_PAYLOAD_LENGTH, (uint8_t)s_txlen);

  // set DIO0=TxDone (01 on bits 7..6)
  write_reg(REG_DIO_MAPPING1, 0x40);

  // trigger TX
  set_opmode(MODE_TX);

  // tunggu TxDone
  uint64_t t0 = esp_timer_get_time();
  while (true) {
    uint8_t flags = read_reg(REG_IRQ_FLAGS);
    if (flags & IRQ_TX_DONE_MASK) {
      write_reg(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK); // clear
      break;
    }
    if ((esp_timer_get_time() - t0) > 3 * 1000 * 1000ULL) { // timeout 3s
      ESP_LOGW(TAG, "Tx timeout");
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }

  // kembali RX continuous (map DIO0 ke RxDone)
  write_reg(REG_DIO_MAPPING1, 0x00);
  set_opmode(MODE_RX_CONTINUOUS);
}

// ========== RX API ==========
int sx1276_parse_packet() {
  // cek RxDone
  uint8_t flags = read_reg(REG_IRQ_FLAGS);
  if (!(flags & IRQ_RX_DONE_MASK)) {
    return 0; // tidak ada paket baru
  }

  // clear RxDone
  write_reg(REG_IRQ_FLAGS, IRQ_RX_DONE_MASK);

  // cek CRC error
  if (flags & IRQ_PAYLOAD_CRC_ERR) {
    // clear CRC error
    write_reg(REG_IRQ_FLAGS, IRQ_PAYLOAD_CRC_ERR);
    return 0;
  }

  // baca alamat FIFO current
  uint8_t cur = read_reg(REG_FIFO_RX_CURRENT);
  write_reg(REG_FIFO_ADDR_PTR, cur);

  // panjang paket
  uint8_t len = read_reg(REG_RX_NB_BYTES);
  if (len > sizeof(s_rx_buf)) len = sizeof(s_rx_buf);

  burst_read(REG_FIFO, s_rx_buf, len);
  s_rx_len = len;
  s_rx_idx = 0;

  // RSSI (HF band: -157 + pktRSSI)
  int pkt = (int)read_reg(REG_PKT_RSSI_VALUE);
  s_last_rssi = pkt - 157;

  return s_rx_len;
}

int sx1276_read_byte() {
  if (s_rx_idx >= s_rx_len) return -1;
  return (int)s_rx_buf[s_rx_idx++];
}

int sx1276_packet_rssi() {
  return s_last_rssi;
}
