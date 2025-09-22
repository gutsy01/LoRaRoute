// main.cpp — port dari main.ino ke ESP-IDF (tanpa Arduino core)
// Pola: setup()/loop() → app_main() + FreeRTOS task
// program dynamic-only F-G5ii (versi ROUTINGID)
//berhasil

#include <stdio.h>
#include <cstdlib>               // srand, rand
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "esp_random.h"

#include "board.h"
#include "LoRaRouting.h"   // deklarasi initLoRa, onDataRecv, runBellmanFord, dll.
#include "node.h"          // deklarasi initNodes, NODE_ID, dll.

// ====== Helper: millis()/random() versi ESP-IDF ======
static inline uint32_t millis() {
  return (uint32_t)(esp_timer_get_time() / 1000ULL);
}
static inline void randomSeed(uint32_t seed) {
  srand(seed);
}
static inline uint32_t urand(uint32_t max_exclusive) {
  return (uint32_t)(esp_random() % (max_exclusive ? max_exclusive : 1));
}

// [PATCH] — Hapus stub parser RX lama (lora_parse_packet):
// (Dihapus seluruh fungsi yang sebelumnya mengembalikan 0 terus-menerus)

// ====== Port dari setup() ======
static void setup_port() {
  ESP_LOGI("APP", "Booting Dynamic Routing (ESP-IDF)");

  initLoRa();
  initNodes();              // cetak Node ID + hello awal
  runBellmanFord();

  randomSeed(((uint32_t)esp_random() << 10) ^ millis());
  ESP_LOGI("APP", "Node ID: %d", NODE_ID);

  sendHelloMessages();
}

// ====== Port dari loop() → task FreeRTOS ======
static void loop_task(void *arg) {
  uint32_t tHello = 0, tRoute = 0, tBF = 0, tAging = 0;

  while (true) {
    uint32_t now = millis();

    // [PATCH] — Gunakan parser RX nyata dari LoRaRouting.cpp (wrapper ke driver SX1276)
    int packetSize = LoRa_ParsePacket();
    if (packetSize > 0) {
      onDataRecv(packetSize);
    }

    if (now - tHello > (uint32_t)(10000u + urand(300))) {
      sendHelloMessages();
      tHello = now;
    }

    if (now - tRoute > (uint32_t)(9000u + urand(3000))) {
      sendRoutingTableId();
      tRoute = now;
    }

    if (now - tBF > 15000u) {
      runBellmanFord();
      tBF = now;
    }

    if (now - tAging > 2000u) {
      checkRoutingTableTimeout();
      tAging = now;
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ====== Titik masuk ESP-IDF ======
extern "C" void app_main(void) {
  setup_port();
  xTaskCreate(loop_task, "loop", 4096, nullptr, 5, nullptr);
}
