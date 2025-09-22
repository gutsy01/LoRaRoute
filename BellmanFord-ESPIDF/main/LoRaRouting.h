#pragma once
#include <string>
#include <cstdint>

// Maks 10 entri seperti versi Arduino
struct RoutingEntry {
    std::string macAddress;   // MAC destinasi "AA:BB:CC:DD:EE:FF"
    int destination = -1;     // node_id tujuan
    int rssi = 0;
    int cost = 10000;
    std::string nextHop;      // MAC next hop
    int nextHopId = -1;       // node_id next hop
    uint32_t lastUpdated = 0; // ms
};

// ===== API utama yang dipanggil dari main.cpp =====
void initLoRa();
void sendHelloMessages();
void onDataRecv(int packetSize);

void runBellmanFord();
void forwardData(int targetNode);
void checkRoutingTableTimeout();

void sendRoutingTableId();                     // broadcast sekali
void sendRoutingTableToId(int neighborId);     // targeted (split horizon by id)
void parseAndUpdateRoutingTableId(const std::string& msg, int rssiToSender);
void printRoutingTableId();
int  LoRa_ParsePacket();  // wrapper untuk polling RX dari main.cpp

// ===== Util =====
std::string macToString(const uint8_t *macAddr);
int         getDestinationFromMac(const std::string& mac);
std::string serializeRoutingTableWithSenderId(int targetNextHopId = -1);

// Tabel routing global
extern RoutingEntry routingTable[10];

// NODE_ID & mapping MAC<->ID disediakan oleh node.{h,cpp}
extern int          NODE_ID;
std::string         nodeIdToMac(int id);
int                 macToNodeId(const std::string&);

// MAC perangkat (Wi‑Fi STA) → dipakai untuk identitas node
const uint8_t* getMacAddress();
