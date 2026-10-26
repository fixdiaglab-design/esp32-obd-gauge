// PID-сканер: визначає швидкість CAN-шини авто та виводить у Serial Monitor,
// які OBD-II параметри (PID) авто підтримує, а потім показує їх живі значення.
// Екран не потрібен. Лише плата + CAN-трансивер SN65HVD230 + штекер OBD.
// Скетч надсилає тільки стандартні запити "режим 01" (читання), нічого не записує.
//
// Serial Monitor: швидкість 115200.
// ESP32-C3: Tools -> USB CDC On Boot: Enabled.

#include "driver/twai.h"

// ---- Піни CAN ----
#define CAN_TX 3          // ESP32-C3 SuperMini
#define CAN_RX 4
// ESP32 DevKit (WROOM-32): CAN_TX 22, CAN_RX 21

bool supported[256];

// ---------- CAN ----------
bool startBus(int which) {
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX, (gpio_num_t)CAN_RX, TWAI_MODE_NORMAL);
  twai_timing_config_t t500 = TWAI_TIMING_CONFIG_500KBITS();
  twai_timing_config_t t250 = TWAI_TIMING_CONFIG_250KBITS();
  twai_timing_config_t t = (which == 0) ? t500 : t250;
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&g, &t, &f) != ESP_OK) return false;
  if (twai_start() != ESP_OK) { twai_driver_uninstall(); return false; }
  return true;
}

void stopBus() {
  twai_stop();
  twai_driver_uninstall();
}

bool queryPid(uint8_t pid, uint8_t *out) {
  twai_clear_receive_queue();
  twai_message_t m = {};
  m.identifier = 0x7DF;
  m.data_length_code = 8;
  m.data[0] = 2; m.data[1] = 1; m.data[2] = pid;
  if (twai_transmit(&m, pdMS_TO_TICKS(50)) != ESP_OK) return false;
  unsigned long t = millis();
  twai_message_t r;
  while (millis() - t < 150) {
    if (twai_receive(&r, pdMS_TO_TICKS(10)) == ESP_OK &&
        r.identifier >= 0x7E8 && r.identifier <= 0x7EF &&
        r.data[1] == 0x41 && r.data[2] == pid) {
      for (int i = 0; i < 4; i++) out[i] = r.data[3 + i];
      return true;
    }
  }
  return false;
}

// ---------- Назви відомих PID ----------
const char *pidName(uint8_t p) {
  switch (p) {
    case 0x04: return "Navantazhennia dvyhuna (%)";
    case 0x05: return "Temperatura OZh (C)";
    case 0x06: return "Korotkochasna korektsiia palyva B1";
    case 0x07: return "Dovhochasna korektsiia palyva B1";
    case 0x0B: return "Tysk u vpusknomu kolektori (kPa) - nadduv";
    case 0x0C: return "Oberty (rpm)";
    case 0x0D: return "Shvydkist (km/h)";
    case 0x0E: return "Kut zapalyuvannia";
    case 0x0F: return "Temperatura vpusknoho povitria (C)";
    case 0x10: return "Masova vytrata povitria MAF (g/s)";
    case 0x11: return "Polozhennia drosela (%)";
    case 0x1F: return "Chas pislia zapusku dvyhuna (s)";
    case 0x2F: return "Riven palyva (%)";
    case 0x33: return "Atmosfernyi tysk (kPa)";
    case 0x42: return "Napruha bortmerezhi (V)";
    case 0x46: return "Temperatura navkolyshnoho seredovyshcha (C)";
    case 0x5C: return "Temperatura olyvy (C)";
    case 0x5E: return "Vytrata palyva (L/h)";
    default:   return "";
  }
}

// ---------- Перелік підтримуваних PID ----------
void scanSupported() {
  memset(supported, 0, sizeof(supported));
  uint8_t base = 0x00;
  while (base <= 0xC0) {
    uint8_t d[4];
    if (!queryPid(base, d)) break;
    uint32_t mask = ((uint32_t)d[0] << 24) | ((uint32_t)d[1] << 16) |
                    ((uint32_t)d[2] << 8) | d[3];
    for (int i = 0; i < 32; i++)
      if (mask & (1UL << (31 - i))) supported[base + 1 + i] = true;
    if (!(mask & 1)) break;          // немає наступного діапазону
    base += 0x20;
  }
}

void setup() {
  Serial.begin(115200);
  unsigned long w = millis();
  while (!Serial && millis() - w < 4000) delay(10);
  Serial.println();
  Serial.println("=== PID-skaner OBD-II ===");
  Serial.println("Umova: zapalyuvannia vmknene (dvyhun mozhe ne pratsiuvaty).");

  int found = -1;
  for (int b = 0; b < 2 && found < 0; b++) {
    Serial.printf("Probuiu %s ...\n", b == 0 ? "500 kbit/s" : "250 kbit/s");
    if (!startBus(b)) { Serial.println("  Ne vdalosia zapustyty CAN (perevir piny)."); continue; }
    uint8_t d[4];
    for (int tries = 0; tries < 5; tries++) {
      if (queryPid(0x00, d)) { found = b; break; }
      delay(200);
    }
    if (found < 0) stopBus();
  }

  if (found < 0) {
    Serial.println("Avto ne vidpovidaie.");
    Serial.println("Perevir: 1) zapalyuvannia, 2) CANH-pin6 / CANL-pin14, 3) GND-pin4 abo 5,");
    Serial.println("4) CTX/CRX ne pereplutani, 5) na module nemaie rezystora 120 Om.");
    while (true) delay(1000);
  }

  Serial.printf("OK! Shvydkist shyny: %s\n", found == 0 ? "500 kbit/s" : "250 kbit/s");
  scanSupported();

  Serial.println("\nAvto pidtrymuie taki PID (rezhym 01):");
  int count = 0;
  for (int p = 1; p <= 0xE0; p++) {
    if (supported[p]) {
      Serial.printf("  0x%02X  %s\n", p, pidName(p));
      count++;
    }
  }
  Serial.printf("Razom: %d\n", count);

  Serial.println("\nCho nam potribno dlia pryladu:");
  struct { uint8_t pid; const char *n; } need[] = {
    {0x0C, "Oberty"}, {0x0D, "Shvydkist"}, {0x2F, "Riven palyva"},
    {0x0B, "Tysk u kolektori (nadduv)"}, {0x33, "Atmosfernyi tysk (dlia nadduvu)"},
    {0x05, "Temperatura OZh"}, {0x42, "Napruha bortmerezhi"},
  };
  for (auto &n : need)
    Serial.printf("  0x%02X %-34s %s\n", n.pid, n.n, supported[n.pid] ? "TAK" : "NI");

  Serial.println("\nZhyvi znachennia (kozhnu sekundu):");
}

void loop() {
  uint8_t d[4];
  if (supported[0x0C] && queryPid(0x0C, d)) Serial.printf("RPM=%d  ", (d[0] * 256 + d[1]) / 4);
  if (supported[0x0D] && queryPid(0x0D, d)) Serial.printf("Speed=%d  ", d[0]);
  if (supported[0x2F] && queryPid(0x2F, d)) Serial.printf("Fuel=%d%%  ", d[0] * 100 / 255);
  if (supported[0x0B] && queryPid(0x0B, d)) Serial.printf("MAP=%dkPa  ", d[0]);
  if (supported[0x05] && queryPid(0x05, d)) Serial.printf("Coolant=%dC  ", d[0] - 40);
  if (supported[0x42] && queryPid(0x42, d)) Serial.printf("Volt=%.1fV  ", (d[0] * 256 + d[1]) / 1000.0);
  Serial.println();
  delay(1000);
}
