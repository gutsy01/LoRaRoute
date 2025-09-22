// LoRaRouting.cpp — ESP-IDF (native, tanpa Arduino)
// Radio: SX1276 via driver lora_sx1276.{h,cpp}

#include "LoRaRouting.h"
#include "board.h"
#include "node.h"
#include "lora_sx1276.h"

#include <algorithm>
#include <cstring>
#include <cstdio>
#include <cstdlib>

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_mac.h"
#include "esp_random.h"

static const char *TAG = "LoRaRouting";

RoutingEntry routingTable[10]; // simpan hingga 10 entry

// ===== waktu (ms) =====
static inline uint32_t now_ms() {
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

// ===== helper upper-case =====
static inline std::string upper(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c){ return (unsigned char)std::toupper(c); });
    return s;
}

// ====== RADIO (wrapper ke driver SX1276) ======
static bool radio_begin()                          { return sx1276_begin(); }
static void radio_begin_packet()                   { sx1276_begin_packet(); }
static void radio_write(const char *data, size_t len){ sx1276_write(data, len); }
static void radio_end_packet()                     { sx1276_end_packet(); }
static int  radio_parse_packet()                   { return sx1276_parse_packet(); }
static int  radio_read_byte()                      { return sx1276_read_byte(); }
static int  radio_packet_rssi()                    { return sx1276_packet_rssi(); }

// Wrapper publik agar main.cpp bisa polling RX
int LoRa_ParsePacket() { return radio_parse_packet(); }

// ====== Implementasi getMacAddress() versi ESP-IDF ======
static uint8_t g_mac[6] = {0};
const uint8_t* getMacAddress() {
    if (g_mac[0]==0 && g_mac[1]==0 && g_mac[2]==0) {
        esp_read_mac(g_mac, ESP_MAC_WIFI_STA);
    }
    return g_mac;
}

// -------------------- Radio Init --------------------
void initLoRa() {
    if (!radio_begin()) {
        ESP_LOGE(TAG, "Starting LoRa failed!");
        abort();
    }
    ESP_LOGI(TAG, "LoRa Initialized (native SX1276). F=%.0f Hz SF=%d BW=%.0f Hz P=%d dBm",
             (double)LORA_FREQ_HZ, (int)LORA_SF, (double)LORA_BW, (int)LORA_TX_POWER_DBM);
}

// -------------------- Utility --------------------
std::string macToString(const uint8_t *macAddr) {
    char buf[18];
    snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
             macAddr[0], macAddr[1], macAddr[2], macAddr[3], macAddr[4], macAddr[5]);
    return std::string(buf);
}

// legacy helper (fallback mac-based calls)
int getDestinationFromMac(const std::string& mac_in) {
    std::string mac = upper(mac_in);
    if (mac == macToString(NODE_0)) return 0;
    if (mac == macToString(NODE_1)) return 1;
    if (mac == macToString(NODE_2)) return 2;
    if (mac == macToString(NODE_3)) return 3;
    if (mac == macToString(NODE_4)) return 4;
    if (mac == macToString(NODE_5)) return 5;
    if (mac == macToString(NODE_6)) return 6;
    if (mac == macToString(NODE_7)) return 7;
    return -1;
}

// -------------------- Hello --------------------
void sendHelloMessages() {
    char nodeName[16];
    snprintf(nodeName, sizeof(nodeName), "NODE_%d", NODE_ID);

    std::string message = std::string("Hello from ") + nodeName;
    std::string macString = macToString(getMacAddress());
    message += " MAC: " + macString;

    radio_begin_packet();
    radio_write(message.c_str(), message.size());
    radio_end_packet();

    ESP_LOGI(TAG, "Sending a Hello message: %s", message.c_str());
}

// -------------------- RX Handler --------------------
static bool isAsciiClean(const std::string& s) {
    for (unsigned char c : s) {
        if (c < 32 || c > 126) return false;
    }
    return true;
}

void onDataRecv(int packetSize) {
    if (packetSize <= 0) return;

    std::string received;
    received.reserve((size_t)packetSize);
    while (true) {
        int b = radio_read_byte();
        if (b < 0) break;
        received.push_back((char)b);
        if ((int)received.size() >= packetSize) break;
    }

    // sanitasi dasar
    if (received.size() > 230) { ESP_LOGW(TAG, "Drop: oversize"); return; }
    if (!isAsciiClean(received))  { ESP_LOGW(TAG, "Drop: non-ASCII"); return; }

    ESP_LOGI(TAG, "Message received: %s", received.c_str());

    // ---- ROUTINGID (baru) ----
    if (received.rfind("ROUTINGID|", 0) == 0) { // startsWith
        int rssi_to_sender = radio_packet_rssi();
        parseAndUpdateRoutingTableId(received, rssi_to_sender);
        printRoutingTableId();
        return;
    }

    // ---- Legacy ROUTING (MAC) -> abaikan ----
    if (received.rfind("ROUTING|", 0) == 0) {
        ESP_LOGI(TAG, "Legacy ROUTING message received (ignored in ID mode).");
        return;
    }

    // ---- HELLO (ambil MAC & RSSI tetangga) ----
    auto pos = received.find("MAC:");
    if (pos != std::string::npos) {
        std::string macString = received.substr(pos + 5);
        macString = upper(macString);
        ESP_LOGI(TAG, "From MAC: %s", macString.c_str());

        int rssi = radio_packet_rssi();
        ESP_LOGI(TAG, "Received RSSI value: %d", rssi);

        // update RSSI table
        {
            uint32_t currentTime = now_ms();
            int nid = macToNodeId(macString);

            // update kalau sudah ada
            for (int i = 0; i < 10; i++) {
                if (routingTable[i].macAddress == macString) {
                    routingTable[i].rssi = rssi;
                    routingTable[i].cost = -rssi;                // cost link ke tetangga
                    routingTable[i].lastUpdated = currentTime;
                    routingTable[i].nextHop = macString;         // tetangga langsung
                    routingTable[i].nextHopId = nid;             // id tetangga langsung
                    routingTable[i].destination = (nid >= 0 ? nid : routingTable[i].destination);
                    goto done_update;
                }
            }

            // atau buat entri baru
            for (int i = 0; i < 10; i++) {
                if (routingTable[i].macAddress.empty()) {
                    routingTable[i].macAddress  = macString;
                    routingTable[i].rssi        = rssi;
                    routingTable[i].cost        = -rssi;
                    routingTable[i].nextHop     = macString;
                    routingTable[i].nextHopId   = nid;
                    routingTable[i].destination = (nid >= 0 ? nid : getDestinationFromMac(macString));
                    routingTable[i].lastUpdated = currentTime;
                    break;
                }
            }
        }
        done_update:;
        // sinkronkan nextHopId untuk neighbor
        int nid = macToNodeId(macString);
        if (nid >= 0) {
            for (int i = 0; i < 10; i++) {
                if (routingTable[i].macAddress == macString || routingTable[i].destination == nid) {
                    routingTable[i].nextHopId = nid;
                    break;
                }
            }
        }
    }
}

// -------------------- Bellman-Ford --------------------
void runBellmanFord() {
    ESP_LOGI(TAG, "Running Bellman-Ford to update routing table...");
    std::string myMac = macToString(getMacAddress());

    // init cost & nexthop
    for (int i = 0; i < 10; i++) {
        if (routingTable[i].macAddress == myMac) {
            routingTable[i].cost = 0;
            routingTable[i].nextHop = myMac;
            routingTable[i].nextHopId = NODE_ID;
        } else if (!routingTable[i].macAddress.empty()) {
            routingTable[i].cost    = -routingTable[i].rssi;               // tetangga langsung
            routingTable[i].nextHop = routingTable[i].macAddress;
            routingTable[i].nextHopId = macToNodeId(routingTable[i].nextHop);
        }
    }

    // relax N-1
    for (int k = 0; k < 9; k++) {
        for (int i = 0; i < 10; i++) {
            if (routingTable[i].macAddress.empty() || routingTable[i].macAddress == myMac) continue;
            for (int j = 0; j < 10; j++) {
                if (routingTable[j].macAddress.empty() || i == j) continue;
                // split horizon: jangan lewat tetangga yang nextHop ke kita
                if (routingTable[j].nextHop == myMac) continue;

                int costToNeighbor = -routingTable[j].rssi;
                int totalCost = costToNeighbor + routingTable[j].cost;
                if (totalCost < routingTable[i].cost) {
                    routingTable[i].cost    = totalCost;
                    routingTable[i].nextHop = routingTable[j].macAddress;
                    routingTable[i].nextHopId = macToNodeId(routingTable[j].macAddress);
                }
            }
        }
    }
    ESP_LOGI(TAG, "Routing table updated.");
    printRoutingTableId();
}

// -------------------- Timeout / Aging --------------------
void checkRoutingTableTimeout() {
    uint32_t currentTime = now_ms();
    const uint32_t timeout = 60000;

    for (int i = 0; i < 10; i++) {
        if (!routingTable[i].macAddress.empty() &&
            currentTime - routingTable[i].lastUpdated > timeout) {
            ESP_LOGW(TAG, "Entry timeout: %s", routingTable[i].macAddress.c_str());
            routingTable[i] = RoutingEntry{}; // reset ke default
        }
    }
}

// -------------------- Forwarding (by node_id) --------------------
void forwardData(int targetNode) {
    for (int i = 0; i < 10; i++) {
        if (routingTable[i].destination == targetNode) {
            uint32_t now = now_ms();
            if (now - routingTable[i].lastUpdated > 10000) {
                ESP_LOGW(TAG, "Route stale. Abort forwarding.");
                return;
            }
            char destinationNode[16];
            snprintf(destinationNode, sizeof(destinationNode), "NODE_%d", targetNode);

            ESP_LOGI(TAG, "Forwarding to next hop (ID %d) MAC %s",
                     routingTable[i].nextHopId, routingTable[i].nextHop.c_str());

            std::string payload = std::string("Data to ") + destinationNode;
            radio_begin_packet();
            radio_write(payload.c_str(), payload.size());
            radio_end_packet();

            ESP_LOGI(TAG, "Data successfully forwarded to node: %s", destinationNode);
            return;
        }
    }
    ESP_LOGW(TAG, "Destination node not found in routing table!");
}

// -------------------- ROUTINGID Serializer --------------------
std::string serializeRoutingTableWithSenderId(int targetNextHopId) {
    // Format: ROUTINGID|<sender_id>|<dest_id,rssi,cost,next_hop_id>|...|
    char head[32];
    snprintf(head, sizeof(head), "ROUTINGID|%d|", NODE_ID);
    std::string msg = head;

    for (int i = 0; i < 10; i++) {
        if (routingTable[i].destination < 0) continue;

        int nhId = routingTable[i].nextHopId >= 0
                 ? routingTable[i].nextHopId
                 : macToNodeId(routingTable[i].nextHop);

        // split horizon by ID
        if (targetNextHopId >= 0 && nhId == targetNextHopId) continue;

        char buf[64];
        snprintf(buf, sizeof(buf), "%d,%d,%d,%d|",
                 routingTable[i].destination,
                 routingTable[i].rssi,
                 routingTable[i].cost,
                 nhId);
        msg += buf;
    }
    return msg;
}

void sendRoutingTableId() {
    std::string payload = serializeRoutingTableWithSenderId(-1);
    radio_begin_packet();
    radio_write(payload.c_str(), payload.size());
    radio_end_packet();
    ESP_LOGI(TAG, "RoutingID broadcast sent.");
}

void sendRoutingTableToId(int neighborId) {
    std::string payload = serializeRoutingTableWithSenderId(neighborId);
    radio_begin_packet();
    radio_write(payload.c_str(), payload.size());
    radio_end_packet();
    ESP_LOGI(TAG, "RoutingID sent to NODE_%d.", neighborId);
}

// -------------------- ROUTINGID Parser --------------------
static bool stoi_safe(const std::string& s, int &out) {
    if (s.empty()) return false;
    char *end = nullptr;
    long v = strtol(s.c_str(), &end, 10);
    if (end == s.c_str() || *end != '\0') return false;
    out = (int)v;
    return true;
}

void parseAndUpdateRoutingTableId(const std::string& message, int rssiToSender) {
    auto p1 = message.find('|'); if (p1 == std::string::npos) return;
    auto p2 = message.find('|', p1 + 1); if (p2 == std::string::npos) return;

    int senderId = -1;
    if (!stoi_safe(message.substr(p1 + 1, p2 - (p1 + 1)), senderId) || senderId < 0) return;

    std::string senderMac = nodeIdToMac(senderId);

    // gunakan RSSI paket ini sebagai biaya ke neighbor
    int costToNeighbor = -rssiToSender;

    // segarkan/insert entri untuk tetangga pengirim
    bool haveSender = false;
    for (int i = 0; i < 10; i++) {
        if (routingTable[i].destination == senderId || routingTable[i].macAddress == senderMac) {
            routingTable[i].destination = senderId;
            routingTable[i].macAddress  = senderMac;
            routingTable[i].rssi        = rssiToSender;
            routingTable[i].cost        = -rssiToSender;
            routingTable[i].nextHopId   = senderId;
            routingTable[i].nextHop     = senderMac;
            routingTable[i].lastUpdated = now_ms();
            haveSender = true;
            break;
        }
    }
    if (!haveSender) {
        for (int i = 0; i < 10; i++) {
            if (routingTable[i].destination < 0 && routingTable[i].macAddress.empty()) {
                routingTable[i].destination = senderId;
                routingTable[i].macAddress  = senderMac;
                routingTable[i].rssi        = rssiToSender;
                routingTable[i].cost        = -rssiToSender;
                routingTable[i].nextHopId   = senderId;
                routingTable[i].nextHop     = senderMac;
                routingTable[i].lastUpdated = now_ms();
                break;
            }
        }
    }

    size_t start = p2 + 1;
    while (start < message.size()) {
        size_t pipe = message.find('|', start);
        if (pipe == std::string::npos) break;
        std::string e = message.substr(start, pipe - start);
        start = pipe + 1;
        if (e.empty()) continue;

        size_t c1 = e.find(','); size_t c2 = e.find(',', c1 + 1); size_t c3 = e.find(',', c2 + 1);
        if (c1==std::string::npos || c2==std::string::npos || c3==std::string::npos) continue;

        int destId=-1, rssi=0, neighborCost=0, nextHopId=-1;
        if (!stoi_safe(e.substr(0, c1), destId)) continue;
        if (!stoi_safe(e.substr(c1+1, c2-(c1+1)), rssi)) continue;
        if (!stoi_safe(e.substr(c2+1, c3-(c2+1)), neighborCost)) continue;
        if (!stoi_safe(e.substr(c3+1), nextHopId)) continue;

        // Skip filler seperti "0,0,0,0" kecuali self-entry si pengirim
        if ((destId == 0 && rssi == 0 && neighborCost == 0 && nextHopId == 0) ||
            (neighborCost <= 0 && destId != senderId)) {
            continue;
        }
        // [PATCH] Abaikan entri untuk diri sendiri;
        // kita tidak perlu menyimpan/overwrite self-route dari tetangga
        if (destId == NODE_ID) {
            continue;
        }
        int totalCost = costToNeighbor + neighborCost;

        bool updated = false;
        for (int i = 0; i < 10; i++) {
            if (routingTable[i].destination == destId) {
                if (totalCost < routingTable[i].cost) {
                    routingTable[i].destination = destId;
                    routingTable[i].rssi        = rssi;
                    routingTable[i].cost        = totalCost;
                    routingTable[i].nextHopId   = senderId;      // next hop = pengirim
                    routingTable[i].nextHop     = senderMac;
                    routingTable[i].macAddress  = nodeIdToMac(destId);
                    routingTable[i].lastUpdated = now_ms();
                }
                updated = true;
                break;
            }
        }
        if (!updated) {
            for (int i = 0; i < 10; i++) {
                if (routingTable[i].destination < 0 && routingTable[i].macAddress.empty()) {
                    routingTable[i].destination = destId;
                    routingTable[i].rssi        = rssi;
                    routingTable[i].cost        = totalCost;
                    routingTable[i].nextHopId   = senderId;
                    routingTable[i].nextHop     = senderMac;
                    routingTable[i].macAddress  = nodeIdToMac(destId);
                    routingTable[i].lastUpdated = now_ms();
                    break;
                }
            }
        }
    }
    ESP_LOGI(TAG, "Routing table (ID) updated from neighbor!");
}

// -------------------- Print (by Node ID) --------------------
void printRoutingTableId() {
    ESP_LOGI(TAG, "Routing Table (by Node ID):");
    ESP_LOGI(TAG, "------------------------------------------------");
    ESP_LOGI(TAG, "DestID  RSSI  NextHopID  Cost  LastUpdated(ms)");
    ESP_LOGI(TAG, "------------------------------------------------");
    uint32_t now = now_ms();
    for (int i = 0; i < 10; i++) {
        // [PATCH] jangan tampilkan self-route
        if (routingTable[i].destination >= 0 &&
            routingTable[i].destination != NODE_ID) {
            ESP_LOGI(TAG, "%6d %5d %10d %6d %14u",
                     routingTable[i].destination,
                     routingTable[i].rssi,
                     routingTable[i].nextHopId,
                     routingTable[i].cost,
                     (unsigned)(now - routingTable[i].lastUpdated));
        }
    }
    ESP_LOGI(TAG, "------------------------------------------------");
}

