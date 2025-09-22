#pragma once
#include <cstdint>
#include <cstddef>

// Konfigurasi diambil dari board.h:
//  - LORA_SS   (CS/NSS)
//  - LORA_RST  (reset pin)
//  - LORA_DIO0 (RxDone/TXDone interrupt pin)
//  - LORA_FREQ_HZ, LORA_SF, LORA_BW, LORA_TX_POWER_DBM

// Inisialisasi radio -> true jika sukses
bool sx1276_begin();

// TX buffer API sederhana (meniru Arduino LoRa)
void sx1276_begin_packet();
void sx1276_write(const char* data, size_t len);
void sx1276_end_packet(); // blocking sampai TxDone

// RX polling: kembalikan payload size jika paket baru tersedia, 0 kalau tidak ada
int  sx1276_parse_packet();

// Baca byte dari buffer RX (dipanggil berulang sampai habis)
int  sx1276_read_byte();

// RSSI paket terakhir (dBm, integer)
int  sx1276_packet_rssi();
