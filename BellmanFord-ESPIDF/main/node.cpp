#include "node.h"
#include "LoRaRouting.h"   // untuk sendHelloMessages() & macToString()/getMacAddress() bila dibutuhkan
#include <string>
#include <cstdio>
#include <cctype>
#include "esp_log.h"

static const char* TAG = "node";

// ===============================
//  MAC address definitions
// ===============================
const uint8_t NODE_0[6] = {0x0C, 0xB8, 0x15, 0xC3, 0x5F, 0x18};
const uint8_t NODE_1[6] = {0xD0, 0xEF, 0x76, 0x57, 0x03, 0x40};
const uint8_t NODE_2[6] = {0x0C, 0xB8, 0x15, 0xC4, 0x1A, 0x90};
const uint8_t NODE_3[6] = {0x0C, 0xB8, 0x15, 0xC5, 0x05, 0xC8};
const uint8_t NODE_4[6] = {0x0C, 0xB8, 0x15, 0xC3, 0x61, 0x00};
const uint8_t NODE_5[6] = {0x0C, 0xB8, 0x15, 0xC3, 0xB9, 0xA8};
const uint8_t NODE_6[6] = {0x30, 0xC6, 0xF7, 0x1E, 0x9A, 0xA0};
const uint8_t NODE_7[6] = {0x44, 0x17, 0x93, 0x87, 0x97, 0x70};
const uint8_t NODE_8[6] = {0x78, 0x21, 0x84, 0x88, 0x76, 0xEC};
const uint8_t NODE_9[6] = {0x78, 0x21, 0x84, 0xDD, 0x1A, 0x64};

// ===============================
//  Node identity
// ===============================
int NODE_ID = 0;            // Ubah per-node (bisa nanti via menuconfig)
int DESTINATION_NODE = -1;  // Default: tidak ada tujuan

// ===============================
//  Local helpers
// ===============================

// format 6 byte MAC -> "AA:BB:CC:DD:EE:FF"
static std::string macBytesToString(const uint8_t* mac) {
    char buf[18];
    std::snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return std::string(buf);
}

// "AA:BB:CC:DD:EE:FF" (UPPER) -> 6 byte; return false jika format salah
static bool parseMac(const std::string& macUpper, uint8_t out[6]) {
    if (macUpper.size() != 17) return false;
    auto hex = [](char c) -> int {
        if (c >= '0' && c <= '9') return c - '0';
        char u = (char)std::toupper((unsigned char)c);
        if (u >= 'A' && u <= 'F') return 10 + (u - 'A');
        return -1;
    };
    for (int i = 0; i < 6; ++i) {
        char hi = macUpper[i*3 + 0];
        char lo = macUpper[i*3 + 1];
        if (i < 5 && macUpper[i*3 + 2] != ':') return false;
        int h = hex(hi), l = hex(lo);
        if (h < 0 || l < 0) return false;
        out[i] = (uint8_t)((h << 4) | l);
    }
    return true;
}

static bool macEqual(const uint8_t* a, const uint8_t* b) {
    for (int i = 0; i < 6; ++i) if (a[i] != b[i]) return false;
    return true;
}

// ===============================
//  Mapping node_id <-> MAC (string)
// ===============================
std::string nodeIdToMac(int nodeId) {
    switch (nodeId) {
        case 0: return macBytesToString(NODE_0);
        case 1: return macBytesToString(NODE_1);
        case 2: return macBytesToString(NODE_2);
        case 3: return macBytesToString(NODE_3);
        case 4: return macBytesToString(NODE_4);
        case 5: return macBytesToString(NODE_5);
        case 6: return macBytesToString(NODE_6);
        case 7: return macBytesToString(NODE_7);
        case 8: return macBytesToString(NODE_8);
        case 9: return macBytesToString(NODE_9);
        default: return std::string();
    }
}

int macToNodeId(const std::string& macStr) {
    // uppercase input
    std::string m = macStr;
    for (auto &c : m) c = (char)std::toupper((unsigned char)c);

    uint8_t x[6];
    if (!parseMac(m, x)) return -1;

    if (macEqual(x, NODE_0)) return 0;
    if (macEqual(x, NODE_1)) return 1;
    if (macEqual(x, NODE_2)) return 2;
    if (macEqual(x, NODE_3)) return 3;
    if (macEqual(x, NODE_4)) return 4;
    if (macEqual(x, NODE_5)) return 5;
    if (macEqual(x, NODE_6)) return 6;
    if (macEqual(x, NODE_7)) return 7;
    if (macEqual(x, NODE_8)) return 8;
    if (macEqual(x, NODE_9)) return 9;

    return -1;
}

// ===============================
//  Exposed API
// ===============================
void initNodes() {
    ESP_LOGI(TAG, "Initializing node...");
    ESP_LOGI(TAG, "Node ID: %d", NODE_ID);

    // Kirim Hello awal (diimplementasi di LoRaRouting.cpp)
    sendHelloMessages();
}

void setDestinationNode(int nodeId) {
    DESTINATION_NODE = nodeId;
    ESP_LOGI(TAG, "Destination node set to: %d", DESTINATION_NODE);
}
