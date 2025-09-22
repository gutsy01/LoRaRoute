#pragma once
// board.h (ESP-IDF friendly)

#define JUMA_BOARD   // <-- pilih salah satu
//#define JUL_BOARD 

// ————— Mapping pin LoRa —————
#if defined(LILYGO_BOARD)
  #define LORA_SCK   5
  #define LORA_MISO  19
  #define LORA_MOSI  27
  #define LORA_SS    18
  #define LORA_RST   23
  #define LORA_DIO0  26

#elif defined(JUMA_BOARD)
  #define LORA_SCK   5
  #define LORA_MISO  19
  #define LORA_MOSI  27
  #define LORA_SS    18
  #define LORA_RST   14
  #define LORA_DIO0  26

#elif defined(JUL_BOARD)
  #define LORA_SCK   18
  #define LORA_MISO  19
  #define LORA_MOSI  23
  #define LORA_SS    5
  #define LORA_RST   14
  #define LORA_DIO0  2

#else
  #error "Pilih salah satu board: #define LILYGO_BOARD atau JUL_BOARD atau JUMA_BOARD"
#endif

// alias lama (jika masih ada kode refer ke SCK/MISO/MOSI/SS/RST/DIO0)
#ifndef SCK
  #define SCK  LORA_SCK
#endif
#ifndef MISO
  #define MISO LORA_MISO
#endif
#ifndef MOSI
  #define MOSI LORA_MOSI
#endif
#ifndef SS
  #define SS   LORA_SS
#endif
#ifndef RST
  #define RST  LORA_RST
#endif
#ifndef DIO0
  #define DIO0 LORA_DIO0
#endif

// LED status onboard (DevKit V1 umumnya GPIO2)
#ifndef STATUS_LED
  #define STATUS_LED 2
#endif

// ====== Konfigurasi radio SX1276 (PHY) ======
#ifndef LORA_FREQ_HZ
#define LORA_FREQ_HZ      923E6   // Indonesia/AS923
#endif

#ifndef LORA_SF
#define LORA_SF           7
#endif

#ifndef LORA_BW
#define LORA_BW           125E3
#endif

#ifndef LORA_TX_POWER_DBM
#define LORA_TX_POWER_DBM 14
#endif

