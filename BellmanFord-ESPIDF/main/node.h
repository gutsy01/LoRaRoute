#pragma once
#include <cstdint>
#include <string>

// ====== MAC address declarations (read-only) ======
extern const uint8_t NODE_0[6];
extern const uint8_t NODE_1[6];
extern const uint8_t NODE_2[6];
extern const uint8_t NODE_3[6];
extern const uint8_t NODE_4[6];
extern const uint8_t NODE_5[6];
extern const uint8_t NODE_6[6];
extern const uint8_t NODE_7[6];
extern const uint8_t NODE_8[6];
extern const uint8_t NODE_9[6];

// ====== Node identity ======
extern int NODE_ID;            // set per perangkat (0..9)
extern int DESTINATION_NODE;   // tujuan (opsional)

// ====== Lifecycle / config ======
void initNodes();
void setDestinationNode(int nodeId);

// ====== Mapping helpers (payload berbasis node_id) ======
int         macToNodeId(const std::string& macUpper); // "AA:BB:..." -> node_id, -1 jika tak dikenal
std::string nodeIdToMac(int nodeId);                  // node_id -> "AA:BB:..."

// ====== Hello (diimplementasi di LoRaRouting.cpp) ======
void sendHelloMessages();

// Catatan:
// - Tidak ada Arduino.h, tidak ada tipe String.
// - getMacAddress() TIDAK dideklarasikan di sini karena di port ESP‑IDF
//   kita sediakan implementasinya di LoRaRouting.cpp (membaca MAC via esp_read_mac()).
