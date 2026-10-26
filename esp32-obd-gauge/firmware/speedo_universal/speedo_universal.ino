/*
  УНІВЕРСАЛЬНИЙ ПРИЛАД OBD-II
  ESP32-C3 SuperMini + GC9A01 240x240 + SN65HVD230 + LVGL 8.3 (SquareLine)

  Що робить:
   1. Після увімкнення шукає авто: пробує CAN 500 кбіт/с, потім 250 кбіт/с.
   2. Питає в авто, які стандартні PID (режим 01) воно підтримує.
   3. Будує список екранів-параметрів ЛИШЕ з того, що авто вміє.
   4. Усі параметри показує на ОДНОМУ екрані SquareLine (дуга + число + назва + одиниці).
   5. Коротке натискання BOOT (GPIO9) = наступний параметр.
      Утримання 0.7-2 с = км/год <-> миль/год, °C <-> °F, бар <-> psi.
      Утримання 2-4 с = яскравість (4 рівні, по колу).
      Утримання 4 с і більше = Wi-Fi вмикається/вимикається (керування зі смартфона).
   6. Якщо авто «заснуло» або кабель вийняли, прилад сам повертається до пошуку.
   Одиниці виміру, останній параметр і яскравість зберігаються після вимкнення.

  Другий вигляд «Панель» (ENABLE_DASH): швидкість у центрі, обороти дугою, температура ОЖ
  і boost. Перемикається зі смартфона або кнопкою (короткими натисканнями після
  останнього параметра).

  Керування зі смартфона: плата створює Wi-Fi мережу WIFI_SSID, сторінка керування
  відкривається в браузері за адресою 192.168.4.1 (параметр, яскравість, одиниці).
  Wi-Fi вмикається сам при запуску (WIFI_AUTOSTART) і вмикається/вимикається утриманням
  кнопки 4 с. Вимикається сам після WIFI_IDLE_MS без дій на сторінці (0 = не вимикати).

  Опитується лише той параметр, який зараз на екрані (+ атмосферний тиск для BOOST),
  тому оновлення швидке і шина не перевантажується.

  Додати новий параметр: один рядок у таблиці PARAMS + (за потреби) формула в decodePid().

  DEMO = 1: тест без авто (усі параметри, вигадані значення).
*/

#include <Arduino.h>
#include <math.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Arduino_GFX_Library.h>
#include <lvgl.h>
#include "ui.h"
#include "driver/twai.h"
#include "esp_sleep.h"

#define DEMO 0
#define DEBUG_SERIAL 1      // 1 = друкувати значення в Serial Monitor раз на секунду

// Стартова анімація: дуга пробігає від 0 до максимуму й назад (як стрілка в авто)
#define BOOT_ANIM    1       // 0 = вимкнути
#define BOOT_UP_MS   800     // скільки йде дуга вгору
#define BOOT_DOWN_MS 800     // скільки йде назад

#define ENABLE_WIFI  1       // 0 = Wi-Fi повністю вимкнено в коді (як у версії без Wi-Fi)

// Другий вигляд екрана «Панель». Потрібен екран ScreenDash у SquareLine-проєкті
// (імена віджетів нижче). Якщо у вашому UI його немає, поставте 0.
#define ENABLE_DASH  1

// Wi-Fi (керування зі смартфона). ОБОВ'ЯЗКОВО змініть пароль (мінімум 8 символів)!
#define WIFI_SSID     "Gauge"
#define WIFI_PASS     "change-me-1234"
#define WIFI_AUTOSTART 1                // 1 = Wi-Fi вмикається одразу при запуску (кнопка теж працює)
#define WIFI_IDLE_MS  600000UL          // вимкнути Wi-Fi після 10 хв без дій (0 = ніколи не вимикати)
#define WIFI_TX_POWER WIFI_POWER_11dBm  // нижча потужність = стабільніше на SuperMini
#define WIFI_TEMP_OFF 85.0f             // °C чипа: вимкнути Wi-Fi (0 = захист вимкнено)
#define WIFI_TEMP_ON  75.0f             // °C чипа: дозволити Wi-Fi знову

// Сон, якщо авто не знайдено: через SLEEP_AFTER_MS без жодної відповіді прилад засинає
// (deep sleep) і прокидається, коли на CAN-шині зʼявляється активність. 0 = не засинати.
#define SLEEP_AFTER_MS 900000UL         // 15 хвилин

// Максимум шкали обертів (дизель ~5000, бензин ~7000)
#define RPM_MAX 6000

// ---------------- ІМЕНА ВІДЖЕТІВ ----------------
// Мають збігатися з іменами в експортованому ui_Screen1.c.
#define W_ARC    ui_ArciUni
#define W_VALUE  ui_LabelUniValue
#define W_UNIT   ui_LabelUniUnit
#define W_NAME   ui_LabelUniName
#define W_DOTS   ui_PanelDots
#define W_SCREEN ui_Screen1

#if ENABLE_DASH
#define D_SCREEN ui_ScreenDash          // екран «Панель»
#define D_ARC    ui_ArciDashRpm         // дуга обертів
#define D_SPEED  ui_LabelDashSpeed      // велике число - швидкість
#define D_SPUNIT ui_LabelDashSpeedUnitkmh  // км/год або mph
#define D_RPM    ui_LabelDashRpm        // обороти текстом
#define D_TEMP   ui_LabelDashTemp       // температура ОЖ
#define D_BOOST  ui_LabelDashBoost      // boost
#endif

// ---------------- ПІНИ ----------------
#define TFT_SCK   6
#define TFT_MOSI  7
#define TFT_CS   10
#define TFT_DC    1
#define TFT_RST   0

#define CAN_TX    3
#define CAN_RX    4

#define BTN_PIN   9

// ---------------- КОЛЬОРИ ДУГИ ----------------
#define COL_NORMAL  0x11BCDE
#define COL_WARN    0xFF9800
#define COL_ALERT   0xF44336

// ---------------- ТАБЛИЦЯ ПАРАМЕТРІВ ----------------
// Базові одиниці: км/год, °C, кПа. Для BOOST - надлишковий тиск у кПа (MAP мінус атмосферний).
// minV/maxV - діапазон дуги в базових одиницях.
// warnAt - частка шкали (0..1), з якої дуга стає помаранчевою (+0.1 - червоною). 0 = без кольорів.
// Порядок у таблиці = порядок перемикання.

enum Kind : uint8_t { K_NONE, K_SPEED, K_TEMP, K_BOOST };

// Тип описано тут, ВИЩЕ за першу функцію: Arduino IDE сам вставляє прототипи функцій
// перед першою функцією, і тип у параметрах має бути вже відомий.
enum UiMode : uint8_t { UI_UNI = 0, UI_DASH = 1 };

struct Param {
  uint8_t pid;
  const char *name;
  const char *unitM;      // метричні одиниці
  const char *unitI;      // імперські одиниці
  Kind kind;
  float minV;
  float maxV;
  uint8_t dec;            // знаків після коми
  uint16_t refreshMs;     // як часто опитувати
  float warnAt;
  bool smooth;            // згладжувати (повільні датчики)
};

static const Param PARAMS[] = {
  // pid   назва        M       I      тип      min    max       dec ms    warn  smooth
  { 0x0C, "RPM",       "rpm",  "rpm", K_NONE,    0,   RPM_MAX,  0, 40,   0.80f, false },
  { 0x0D, "SPEED",     "km/h", "mph", K_SPEED,   0,   240,      0, 60,   0.00f, false },
  { 0x05, "COOLANT",   "C",    "F",   K_TEMP,   40,   130,      0, 1000, 0.72f, false },
  { 0x0B, "BOOST",     "bar",  "psi", K_BOOST, -100,  200,      1, 60,   0.90f, false },
  { 0x04, "LOAD",      "%",    "%",   K_NONE,    0,   100,      0, 100,  0.00f, false },
  { 0x11, "THROTTLE",  "%",    "%",   K_NONE,    0,   100,      0, 80,   0.00f, false },
  { 0x2F, "FUEL",      "%",    "%",   K_NONE,    0,   100,      0, 2000, 0.00f, true  },
  { 0x42, "VOLTAGE",   "V",    "V",   K_NONE,    8,   16,       1, 1000, 0.00f, false },
  { 0x0F, "INTAKE",    "C",    "F",   K_TEMP,  -20,   80,       0, 1000, 0.00f, false },
  { 0x46, "AMBIENT",   "C",    "F",   K_TEMP,  -30,   50,       0, 5000, 0.00f, false },
  { 0x5C, "OIL",       "C",    "F",   K_TEMP,   40,   150,      0, 1000, 0.73f, false },
  { 0x10, "MAF",       "g/s",  "g/s", K_NONE,    0,   250,      1, 200,  0.00f, false },
  { 0x5E, "FUEL RATE", "L/h",  "L/h", K_NONE,    0,   50,       1, 1000, 0.00f, false },
  { 0x3C, "CAT B1S1",  "C",    "F",   K_TEMP,    0,   900,      0, 1000, 0.00f, false },
  { 0x3E, "CAT B1S2",  "C",    "F",   K_TEMP,    0,   900,      0, 1000, 0.00f, false },
};

static const int NPARAMS = sizeof(PARAMS) / sizeof(PARAMS[0]);
#define MAX_ACTIVE 32

// ---------------- ДИСПЛЕЙ ----------------

Arduino_DataBus *bus = new Arduino_ESP32SPI(
    TFT_DC, TFT_CS, TFT_SCK, TFT_MOSI, GFX_NOT_DEFINED);

Arduino_GFX *gfx = new Arduino_GC9A01(bus, TFT_RST, 0, true);

// Яскравість: у модуля немає виводу підсвітки, тому приглушуємо самі пікселі
// (множимо RGB565 на коефіцієнт у flushCb). Коефіцієнт 256 = 100%.
static uint8_t brPct = 100;          // яскравість у відсотках, 5..100
static uint16_t brScale = 256;       // те саме у масштабі 0..256

static lv_disp_draw_buf_t drawBuf;
static lv_color_t buf1[240 * 30];
static lv_disp_drv_t dispDrv;

void flushCb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_p) {
  uint32_t w = area->x2 - area->x1 + 1;
  uint32_t h = area->y2 - area->y1 + 1;

#if (LV_COLOR_16_SWAP == 0)
  uint32_t sc = brScale;
  if (sc < 256) {
    uint16_t *px = (uint16_t *)&color_p->full;
    uint32_t n = w * h;
    for (uint32_t i = 0; i < n; i++) {
      uint16_t c = px[i];
      uint16_t r = ((c >> 11) * sc) >> 8;
      uint16_t g = (((c >> 5) & 0x3F) * sc) >> 8;
      uint16_t b = ((c & 0x1F) * sc) >> 8;
      px[i] = (r << 11) | (g << 5) | b;
    }
  }
#endif

#if (LV_COLOR_16_SWAP != 0)
  gfx->draw16bitBeRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#else
  gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#endif

  lv_disp_flush_ready(drv);
}

// ---------------- СТАН ----------------

enum State { ST_SCAN, ST_RUN };
static State state = ST_SCAN;

static UiMode uiMode = UI_UNI;       // який вигляд зараз на екрані
static UiMode savedMode = UI_UNI;    // який вигляд обрано (зберігається в пам'яті)
static const bool DASH_ON = (ENABLE_DASH != 0);

static Preferences prefs;
static bool imperial = false;
static uint8_t savedPid = 0x0C;

static bool supported[256];
static float pidVal[256];
static bool pidHas[256];
static uint32_t pidTime[256];
static uint32_t lastReq[256];

static int active[MAX_ACTIVE];   // індекси в PARAMS, які авто підтримує
static int nActive = 0;
static int cur = 0;              // поточна позиція в active[]

static int busIdx = 0;           // 0 = 500 кбіт/с, 1 = 250 кбіт/с
static uint32_t nextScanAt = 0;
static uint32_t lastRxOk = 0;

static lv_obj_t *dots[MAX_ACTIVE];

// ---------------- CAN: старт / стоп ----------------

static bool canStart(int idx) {
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
      (gpio_num_t)CAN_TX, (gpio_num_t)CAN_RX, TWAI_MODE_NORMAL);
  twai_timing_config_t t500 = TWAI_TIMING_CONFIG_500KBITS();
  twai_timing_config_t t250 = TWAI_TIMING_CONFIG_250KBITS();
  twai_timing_config_t t = (idx == 0) ? t500 : t250;
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g, &t, &f) != ESP_OK) {
    return false;
  }
  if (twai_start() != ESP_OK) {
    twai_driver_uninstall();
    return false;
  }
  return true;
}

static void canStop() {
  twai_stop();
  twai_driver_uninstall();
}

// ---------------- CAN: блокувальний запит (лише під час пошуку) ----------------

static bool queryBlocking(uint8_t pid, uint8_t *out) {
  twai_clear_receive_queue();

  twai_message_t m = {};
  m.identifier = 0x7DF;
  m.data_length_code = 8;
  m.data[0] = 2;
  m.data[1] = 1;
  m.data[2] = pid;

  if (twai_transmit(&m, pdMS_TO_TICKS(50)) != ESP_OK) {
    return false;
  }

  uint32_t t = millis();
  twai_message_t r;

  while ((uint32_t)(millis() - t) < 150) {
    if (twai_receive(&r, pdMS_TO_TICKS(10)) == ESP_OK &&
        !r.extd && !r.rtr &&
        r.identifier >= 0x7E8 && r.identifier <= 0x7EF &&
        r.data_length_code >= 7 &&
        r.data[1] == 0x41 && r.data[2] == pid) {
      for (int i = 0; i < 4; i++) {
        out[i] = r.data[3 + i];
      }
      return true;
    }
  }
  return false;
}

// Бітові маски підтримуваних PID: 0x00, 0x20, 0x40, ...
static void scanSupported() {
  memset(supported, 0, sizeof(supported));

  uint8_t base = 0x00;
  while (base <= 0xC0) {
    uint8_t d[4];
    if (!queryBlocking(base, d)) {
      break;
    }
    uint32_t mask = ((uint32_t)d[0] << 24) | ((uint32_t)d[1] << 16) |
                    ((uint32_t)d[2] << 8) | d[3];
    for (int i = 0; i < 32; i++) {
      if (mask & (1UL << (31 - i))) {
        supported[base + 1 + i] = true;
      }
    }
    if (!(mask & 1)) {
      break;                       // наступного діапазону немає
    }
    base += 0x20;
  }
}

// ---------------- CAN: неблокувальне опитування (робочий режим) ----------------

static bool waitingPid = false;
static uint8_t requestedPid = 0;
static uint32_t requestStarted = 0;
static uint32_t requestStartedUs = 0;
static uint32_t rxCount = 0;           // діагностика: скільки відповідей за секунду
static uint32_t rttSumUs = 0;          // і скільки мкс у середньому йде запит-відповідь
static volatile bool uiDirty = false;  // з'явилося нове значення - малювати одразу
static constexpr uint32_t CAN_TIMEOUT_MS = 80;

// Відновлення після bus-off і застряглої черги передачі
static void canRecoverIfNeeded() {
  static uint32_t lastCheck = 0;
  uint32_t now = millis();

  if ((uint32_t)(now - lastCheck) < 500) {
    return;
  }
  lastCheck = now;

  twai_status_info_t st;
  if (twai_get_status_info(&st) != ESP_OK) {
    return;
  }

  if (st.state == TWAI_STATE_BUS_OFF) {
    twai_initiate_recovery();
    waitingPid = false;
  } else if (st.state == TWAI_STATE_STOPPED) {
    twai_start();
    waitingPid = false;
  }

  if (st.msgs_to_tx > 3) {
    twai_clear_transmit_queue();
    waitingPid = false;
  }
}

static bool sendPid(uint8_t pid) {
  twai_message_t m = {};
  m.identifier = 0x7DF;
  m.data_length_code = 8;
  m.data[0] = 0x02;
  m.data[1] = 0x01;
  m.data[2] = pid;

  if (twai_transmit(&m, 0) != ESP_OK) {
    return false;
  }

  requestedPid = pid;
  requestStarted = millis();
  requestStartedUs = micros();
  waitingPid = true;
  return true;
}

// Формули стандартних PID (SAE J1979). d[] = байти A, B, ... ; n = скільки їх є.
static bool decodePid(uint8_t pid, const uint8_t *d, uint8_t n, float &out) {
  if (n < 1) {
    return false;
  }
  float A = d[0];
  float B = (n >= 2) ? d[1] : 0;

  switch (pid) {
    case 0x04: out = A * 100.0f / 255.0f;            return true;   // навантаження %
    case 0x05: out = A - 40.0f;                      return true;   // ОЖ °C
    case 0x0B: out = A;                              return true;   // MAP кПа (абс.)
    case 0x0C: if (n < 2) return false;
               out = (A * 256.0f + B) / 4.0f;        return true;   // обороти
    case 0x0D: out = A;                              return true;   // швидкість км/год
    case 0x0F: out = A - 40.0f;                      return true;   // впускне повітря °C
    case 0x10: if (n < 2) return false;
               out = (A * 256.0f + B) / 100.0f;      return true;   // MAF г/с
    case 0x11: out = A * 100.0f / 255.0f;            return true;   // дросель %
    case 0x2F: out = A * 100.0f / 255.0f;            return true;   // паливо %
    case 0x33: out = A;                              return true;   // атмосферний тиск кПа
    case 0x3C:
    case 0x3E: if (n < 2) return false;
               out = (A * 256.0f + B) / 10.0f - 40.0f; return true; // каталізатор °C
    case 0x42: if (n < 2) return false;
               out = (A * 256.0f + B) / 1000.0f;     return true;   // напруга В
    case 0x46: out = A - 40.0f;                      return true;   // довкілля °C
    case 0x5C: out = A - 40.0f;                      return true;   // олива °C
    case 0x5E: if (n < 2) return false;
               out = (A * 256.0f + B) / 20.0f;       return true;   // витрата л/год
    default:   return false;
  }
}

static bool isSmooth(uint8_t pid) {
  for (int i = 0; i < NPARAMS; i++) {
    if (PARAMS[i].pid == pid) {
      return PARAMS[i].smooth;
    }
  }
  return false;
}

static void processCanResponse(const twai_message_t &r) {
  if (r.extd || r.rtr) return;
  if (r.identifier < 0x7E8 || r.identifier > 0x7EF) return;
  if (r.data_length_code < 4) return;
  if (r.data[1] != 0x41 || r.data[2] != requestedPid) return;

  uint8_t n = (r.data[0] >= 2) ? (r.data[0] - 2) : 0;
  if (n > r.data_length_code - 3) {
    n = r.data_length_code - 3;
  }

  float v;
  if (!decodePid(requestedPid, &r.data[3], n, v)) {
    return;
  }

  if (isSmooth(requestedPid) && pidHas[requestedPid]) {
    v = pidVal[requestedPid] * 0.8f + v * 0.2f;
  }

  pidVal[requestedPid] = v;
  pidHas[requestedPid] = true;
  pidTime[requestedPid] = millis();
  lastRxOk = millis();
  waitingPid = false;

  rxCount++;
  rttSumUs += (uint32_t)(micros() - requestStartedUs);
  uiDirty = true;
}

#if ENABLE_DASH
// Режим «Панель»: по черзі опитуємо обороти, швидкість, boost, температуру
// (за раз один запит; береться той параметр, який найдовше чекає)
static void pollDash(uint32_t now) {
  static const uint8_t pids[] = { 0x0C, 0x0D, 0x0B, 0x05, 0x33 };
  static const uint16_t ints[] = { 50, 100, 100, 1000, 10000 };

  int best = -1;
  int32_t bestOver = 0;

  for (int i = 0; i < 5; i++) {
    uint8_t pid = pids[i];
    if (!supported[pid]) {
      continue;
    }
    if (pid == 0x33 && !supported[0x0B]) {
      continue;
    }
    int32_t over = (int32_t)(now - lastReq[pid]) - (int32_t)ints[i];
    if (over >= 0 && (best < 0 || over > bestOver)) {
      best = i;
      bestOver = over;
    }
  }

  if (best >= 0 && sendPid(pids[best])) {
    lastReq[pids[best]] = now;
  }
}
#endif

static void pollCan() {
  if (state != ST_RUN) {
    return;
  }

  canRecoverIfNeeded();

  twai_message_t r;
  while (twai_receive(&r, 0) == ESP_OK) {
    if (waitingPid) {
      processCanResponse(r);
    }
  }

  uint32_t now = millis();

  if (waitingPid && (uint32_t)(now - requestStarted) >= CAN_TIMEOUT_MS) {
    waitingPid = false;
  }
  if (waitingPid) {
    return;
  }

#if ENABLE_DASH
  if (uiMode == UI_DASH) {
    pollDash(now);
    return;
  }
#endif

  const Param &p = PARAMS[active[cur]];

  if ((uint32_t)(now - lastReq[p.pid]) >= p.refreshMs) {
    if (sendPid(p.pid)) {
      lastReq[p.pid] = now;
    }
  } else if (p.kind == K_BOOST && supported[0x33] &&
             (uint32_t)(now - lastReq[0x33]) >= 10000) {
    if (sendPid(0x33)) {
      lastReq[0x33] = now;
    }
  }
}

// ---------------- ОДИНИЦІ ----------------

static float toDisplay(Kind k, float v) {
  switch (k) {
    case K_SPEED: return imperial ? v * 0.621371f : v;
    case K_TEMP:  return imperial ? v * 1.8f + 32.0f : v;
    case K_BOOST: return imperial ? v * 0.145038f : v / 100.0f;
    default:      return v;
  }
}

static const char *unitOf(const Param &p) {
  return imperial ? p.unitI : p.unitM;
}

// Значення параметра в базових одиницях (для BOOST віднімаємо атмосферний тиск)
static float baseValue(const Param &p) {
  float v = pidVal[p.pid];
  if (p.kind == K_BOOST) {
    float baro = pidHas[0x33] ? pidVal[0x33] : 101.3f;
    v -= baro;
  }
  return v;
}

static const Param *findParam(uint8_t pid) {
  for (int i = 0; i < NPARAMS; i++) {
    if (PARAMS[i].pid == pid) {
      return &PARAMS[i];
    }
  }
  return &PARAMS[0];
}

static bool freshPid(uint8_t pid) {
  return pidHas[pid] && (uint32_t)(millis() - pidTime[pid]) < 3000;
}

static uint32_t zoneColor(float frac, float warnAt, uint32_t normal) {
  if (warnAt <= 0.0f) {
    return normal;
  }
  if (frac >= warnAt + 0.1f) {
    return COL_ALERT;
  }
  if (frac >= warnAt) {
    return COL_WARN;
  }
  return normal;
}

// ---------------- UI ----------------

static char lastText[16] = "";
static int lastArc = -1;
static uint32_t lastColor = 0;

static void showStatus(const char *unitText) {
  uiMode = UI_UNI;
  if (lv_scr_act() != W_SCREEN) {
    lv_scr_load(W_SCREEN);
  }
  lv_label_set_text(W_NAME, "OBD-II");
  lv_label_set_text(W_VALUE, "--");
  lv_label_set_text(W_UNIT, unitText);
  lv_arc_set_value(W_ARC, 0);
  lastText[0] = 0;
  lastArc = -1;
}

static void buildDots() {
  lv_obj_clean(W_DOTS);

  int sz  = (nActive <= 8) ? 6 : (nActive <= 12 ? 5 : 4);
  int gap = (nActive <= 8) ? 7 : (nActive <= 12 ? 5 : 3);

  lv_obj_set_width(W_DOTS, 120);
  lv_obj_set_flex_flow(W_DOTS, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(W_DOTS, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  lv_obj_set_style_pad_all(W_DOTS, 0, 0);
  lv_obj_set_style_pad_column(W_DOTS, gap, 0);

  for (int i = 0; i < nActive && i < MAX_ACTIVE; i++) {
    lv_obj_t *d = lv_obj_create(W_DOTS);
    lv_obj_remove_style_all(d);
    lv_obj_set_size(d, sz, sz);
    lv_obj_set_style_radius(d, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_opa(d, LV_OPA_COVER, 0);
    lv_obj_clear_flag(d, LV_OBJ_FLAG_CLICKABLE);
    dots[i] = d;
  }
}

static void highlightDots() {
  for (int i = 0; i < nActive && i < MAX_ACTIVE; i++) {
    bool on = (i == cur);
    lv_obj_set_style_bg_color(dots[i], lv_color_hex(on ? 0xFFFFFF : COL_NORMAL), 0);
    lv_obj_set_style_bg_opa(dots[i], on ? 255 : 110, 0);
  }
}

// Підписи, які змінюються лише при перемиканні параметра або одиниць
static void applyParamStatic() {
  const Param &p = PARAMS[active[cur]];
  lv_label_set_text(W_NAME, p.name);
  lv_label_set_text(W_UNIT, unitOf(p));
  lastText[0] = 0;
  lastArc = -1;
  lastColor = 0;
  highlightDots();
}

static void buildActive() {
  nActive = 0;

  for (int i = 0; i < NPARAMS && nActive < MAX_ACTIVE; i++) {
#if DEMO
    active[nActive++] = i;
#else
    if (supported[PARAMS[i].pid]) {
      active[nActive++] = i;
    }
#endif
  }

  // Запасний варіант, якщо авто не повідомило жодного відомого PID
  if (nActive == 0) {
    active[nActive++] = 0;   // RPM
    active[nActive++] = 1;   // SPEED
  }

  cur = 0;
  for (int i = 0; i < nActive; i++) {
    if (PARAMS[active[i]].pid == savedPid) {
      cur = i;
      break;
    }
  }

  buildDots();
  applyParamStatic();
}

#if ENABLE_DASH

static char dTxtSpeed[16], dTxtRpm[16], dTxtTemp[16], dTxtBoost[16];
static int dArc = -1;
static uint32_t dColRpm = 0, dColTemp = 0, dColBoost = 0;
static bool dImp = false;

static void dashInvalidate() {
  dTxtSpeed[0] = dTxtRpm[0] = dTxtTemp[0] = dTxtBoost[0] = 0;
  dArc = -1;
  dColRpm = dColTemp = dColBoost = 0;
  dImp = !imperial;                    // примусово оновити підпис одиниць
}

static void setTxt(lv_obj_t *o, char *cache, const char *t) {
  if (strcmp(cache, t) != 0) {
    strncpy(cache, t, 15);
    cache[15] = 0;
    lv_label_set_text(o, t);
  }
}

static void refreshDash() {
  const Param *pSpd = findParam(0x0D);
  const Param *pRpm = findParam(0x0C);
  const Param *pTmp = findParam(0x05);
  const Param *pBst = findParam(0x0B);

  float spd = 0, rpm = 0, tmp = 0, bst = 0;
  bool okS, okR, okT, okB;

#if DEMO
  float ph = millis() / 3000.0f;
  spd = 120.0f * (0.5f + 0.5f * sinf(ph));
  rpm = RPM_MAX * (0.5f + 0.5f * sinf(ph * 1.3f));
  tmp = 90.0f;
  bst = -20.0f + 120.0f * (0.5f + 0.5f * sinf(ph * 1.1f));
  okS = okR = okT = okB = true;
#else
  okS = freshPid(0x0D);
  okR = freshPid(0x0C);
  okT = freshPid(0x05);
  okB = freshPid(0x0B);
  if (okS) spd = pidVal[0x0D];
  if (okR) rpm = pidVal[0x0C];
  if (okT) tmp = pidVal[0x05];
  if (okB) bst = baseValue(*pBst);
#endif

  char t[16];

  // швидкість (центр)
  if (okS) snprintf(t, sizeof(t), "%.0f", toDisplay(K_SPEED, spd));
  else strcpy(t, "--");
  setTxt(D_SPEED, dTxtSpeed, t);

  if (dImp != imperial) {
    dImp = imperial;
    lv_label_set_text(D_SPUNIT, imperial ? pSpd->unitI : pSpd->unitM);
  }

  // обороти: текст + дуга
  if (okR) snprintf(t, sizeof(t), "%.0f rpm", rpm);
  else strcpy(t, "-- rpm");
  setTxt(D_RPM, dTxtRpm, t);

  float fr = okR ? rpm / (float)RPM_MAX : 0;
  fr = constrain(fr, 0.0f, 1.0f);
  int arc = (int)(fr * 1000.0f);
  if (arc != dArc) {
    dArc = arc;
    lv_arc_set_value(D_ARC, arc);
  }
  uint32_t colR = zoneColor(fr, pRpm->warnAt, COL_NORMAL);
  if (colR != dColRpm) {
    dColRpm = colR;
    lv_obj_set_style_arc_color(D_ARC, lv_color_hex(colR), LV_PART_INDICATOR | LV_STATE_DEFAULT);
  }

  // температура ОЖ
  if (okT) snprintf(t, sizeof(t), "%.0f %s", toDisplay(K_TEMP, tmp), imperial ? pTmp->unitI : pTmp->unitM);
  else strcpy(t, "--");
  setTxt(D_TEMP, dTxtTemp, t);

  float ft = okT ? (tmp - pTmp->minV) / (pTmp->maxV - pTmp->minV) : 0;
  uint32_t colT = zoneColor(ft, pTmp->warnAt, 0xFAFDFE);
  if (colT != dColTemp) {
    dColTemp = colT;
    lv_obj_set_style_text_color(D_TEMP, lv_color_hex(colT), LV_PART_MAIN | LV_STATE_DEFAULT);
  }

  // boost
  if (okB) {
    float b = toDisplay(K_BOOST, bst);
    if (fabsf(b) < 0.05f) {
      b = 0;                           // прибрати «-0.0»
    }
    snprintf(t, sizeof(t), "%.1f %s", b, imperial ? pBst->unitI : pBst->unitM);
  } else {
    strcpy(t, "--");
  }
  setTxt(D_BOOST, dTxtBoost, t);

  float fb = okB ? (bst - pBst->minV) / (pBst->maxV - pBst->minV) : 0;
  uint32_t colB = zoneColor(fb, pBst->warnAt, 0xFAFDFE);
  if (colB != dColBoost) {
    dColBoost = colB;
    lv_obj_set_style_text_color(D_BOOST, lv_color_hex(colB), LV_PART_MAIN | LV_STATE_DEFAULT);
  }
}

static void setUiMode(UiMode m, bool save) {
  if (m == UI_DASH && state != ST_RUN) {
    return;
  }

  if (m != uiMode) {
    uiMode = m;
    if (m == UI_DASH) {
      lv_scr_load(D_SCREEN);
      dashInvalidate();
      lastReq[0x0C] = lastReq[0x0D] = lastReq[0x0B] = lastReq[0x05] = 0;
    } else {
      lv_scr_load(W_SCREEN);
      applyParamStatic();
      lastReq[PARAMS[active[cur]].pid] = 0;
    }
  }

  if (save) {
    savedMode = m;
    prefs.putUChar("mode", (uint8_t)m);
  }
}

#else

static inline void dashInvalidate() {}
static inline void refreshDash() {}
static inline void setUiMode(UiMode, bool) {}

#endif

// Тимчасове повідомлення на екрані (Wi-Fi увімкнено/вимкнено)
static uint32_t overlayUntil = 0;

static void showOverlay(const char *name, const char *value, const char *unit) {
  if (lv_scr_act() != W_SCREEN) {
    lv_scr_load(W_SCREEN);             // повідомлення показуємо на звичайному екрані
  }
  lv_label_set_text(W_NAME, name);
  lv_label_set_text(W_VALUE, value);
  lv_label_set_text(W_UNIT, unit);
  lv_arc_set_value(W_ARC, 0);
  lastText[0] = 0;
  lastArc = -1;
  overlayUntil = millis() + 4000;
  if (overlayUntil == 0) {
    overlayUntil = 1;
  }
}

static bool overlayActive() {
  if (overlayUntil == 0) {
    return false;
  }
  if ((int32_t)(millis() - overlayUntil) < 0) {
    return true;
  }
  overlayUntil = 0;
  if (state == ST_RUN) {
    if (DASH_ON && uiMode == UI_DASH) {
#if ENABLE_DASH
      lv_scr_load(D_SCREEN);
#endif
      dashInvalidate();
    } else {
      applyParamStatic();
    }
  }
  return false;
}

static void refreshUi() {
  if (overlayActive()) {
    return;
  }

  if (state != ST_RUN) {
    return;
  }

  if (DASH_ON && uiMode == UI_DASH) {
    refreshDash();
    return;
  }

  const Param &p = PARAMS[active[cur]];
  uint32_t now = millis();

  float base;
  bool fresh;

#if DEMO
  float ph = now / 4000.0f;
  base = p.minV + (0.5f + 0.5f * sinf(ph)) * (p.maxV - p.minV);
  fresh = true;
#else
  fresh = pidHas[p.pid] && (uint32_t)(now - pidTime[p.pid]) < 3000;
  base = fresh ? baseValue(p) : 0;
#endif

  char text[16];

  if (!fresh) {
    strcpy(text, "--");
  } else {
    float disp = toDisplay(p.kind, base);
    float eps = 0.5f;
    for (int i = 0; i < p.dec; i++) {
      eps /= 10.0f;
    }
    if (fabsf(disp) < eps) {
      disp = 0;                    // прибрати «-0»
    }
    snprintf(text, sizeof(text), "%.*f", p.dec, disp);
  }

  if (strcmp(text, lastText) != 0) {
    strcpy(lastText, text);
    lv_label_set_text(W_VALUE, text);
  }

  float frac = fresh ? (base - p.minV) / (p.maxV - p.minV) : 0;
  frac = constrain(frac, 0.0f, 1.0f);

  int arc = (int)(frac * 1000.0f);
  if (arc != lastArc) {
    lastArc = arc;
    lv_arc_set_value(W_ARC, arc);
  }

  uint32_t col = COL_NORMAL;
  if (p.warnAt > 0.0f) {
    if (frac >= p.warnAt + 0.1f) {
      col = COL_ALERT;
    } else if (frac >= p.warnAt) {
      col = COL_WARN;
    }
  }
  if (col != lastColor) {
    lastColor = col;
    lv_obj_set_style_arc_color(W_ARC, lv_color_hex(col), LV_PART_INDICATOR | LV_STATE_DEFAULT);
  }
}

// ---------------- КНОПКА ----------------

static void selectParam(int idx) {
  if (idx < 0 || idx >= nActive) {
    return;
  }
  cur = idx;
  uint8_t pid = PARAMS[active[cur]].pid;
  lastReq[pid] = 0;                // опитати одразу
  prefs.putUChar("pid", pid);
  applyParamStatic();
}

static void nextParam() {
  // Коло сторінок: параметри 1..N, потім «Панель», потім знову параметр 1
  if (uiMode == UI_DASH) {
    setUiMode(UI_UNI, true);
    selectParam(0);
    return;
  }
  if (DASH_ON && state == ST_RUN && cur == nActive - 1) {
    setUiMode(UI_DASH, true);
    return;
  }
  if (nActive < 2) {
    return;
  }
  selectParam((cur + 1) % nActive);
}

static void toggleUnits() {
  imperial = !imperial;
  prefs.putBool("imp", imperial);
  applyParamStatic();
}

// ---------------- ЯСКРАВІСТЬ ----------------

static bool brSavePending = false;
static uint32_t brSaveAt = 0;

static void setBrightness(int pct) {
  pct = constrain(pct, 5, 100);
  if (pct == brPct) {
    return;
  }
  brPct = (uint8_t)pct;
  brScale = (uint16_t)(pct * 256 / 100);
  lv_obj_invalidate(lv_scr_act());     // перемалювати весь екран з новою яскравістю
  brSavePending = true;                // записати у пам'ять, коли рух повзунка стихне
  brSaveAt = millis() + 2000;
}

static void cycleBrightness() {
  static const uint8_t LEVELS[] = { 100, 60, 30, 14 };
  uint8_t next = LEVELS[0];
  for (unsigned i = 0; i < sizeof(LEVELS); i++) {
    if (LEVELS[i] < brPct) {
      next = LEVELS[i];
      break;
    }
  }
  setBrightness(next);
}

// ---------------- WI-FI + ВЕБ-СТОРІНКА ----------------
// Веб-сервер працює в ОКРЕМОМУ завданні FreeRTOS, тому повільні запити телефону
// не затримують CAN і екран. Сторінка нічого не змінює сама: вона лише залишає
// команди (cmdSelect/cmdBright/cmdUnits), а головний цикл виконує їх у processWeb()
// (чіпати LVGL з іншого завдання не можна).

#if ENABLE_WIFI

static WebServer server(80);
static volatile bool wifiOn = false;
static volatile uint32_t wifiLastAct = 0;
static volatile int wifiReq = 0;       // 1 = увімкнути, 2 = вимкнути
static volatile int wifiEvent = 0;     // 1 = щойно ввімкнено, 2 = щойно вимкнено
static volatile int cmdSelect = -1;
static volatile int cmdBright = -1;
static volatile int cmdUnits = -1;
static volatile int cmdMode = -1;
static bool wifiHot = false;           // чип перегрітий: Wi-Fi заблоковано

static const char PAGE[] PROGMEM = R"HTML(<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Gauge</title>
<style>
body{margin:0;padding:16px;background:#000;color:#eee;font:16px system-ui,sans-serif}
h1{font-size:18px;color:#11bcde;margin:0 0 8px}
h2{font-size:14px;color:#888;margin:20px 0 8px;font-weight:600}
#val{font-size:54px;font-weight:700;text-align:center;margin:10px 0 0}
#unit{font-size:15px;color:#11bcde;text-align:center;margin-bottom:6px;min-height:20px}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:8px}
button{padding:14px 8px;border:1px solid #11bcde;background:#001418;color:#eee;border-radius:10px;font-size:15px}
button.on{background:#11bcde;color:#000;font-weight:700}
input[type=range]{width:100%;margin:6px 0}
</style></head><body>
<h1>OBD-II прилад</h1>
<div id="val">--</div><div id="unit"></div>
<div id="mdbox" style="display:none"><h2>ЕКРАН</h2>
<div class="grid"><button id="m0">Параметри</button><button id="m1">Панель</button></div></div>
<h2>ПАРАМЕТР</h2><div class="grid" id="list"></div>
<h2>ЯСКРАВІСТЬ: <span id="bv"></span>%</h2>
<input id="br" type="range" min="5" max="100" step="5">
<h2>ОДИНИЦІ</h2>
<div class="grid"><button id="u0">km/h, C, bar</button><button id="u1">mph, F, psi</button></div>
<script>
var busy=0,names="",t;
function q(u){return fetch(u).then(function(r){return r.json()}).catch(function(){return null})}
function $(i){return document.getElementById(i)}
function draw(s){
 if(!s)return;
 $('val').textContent=s.v;
 $('unit').textContent=s.st=='run'?s.u:'Пошук авто...';
 var n=s.list.join('|');
 if(n!=names){names=n;$('list').innerHTML='';
  s.list.forEach(function(x,i){var b=document.createElement('button');b.id='p'+i;b.textContent=x;
   b.onclick=function(){q('/api/set?p='+i).then(draw)};$('list').appendChild(b)})}
 s.list.forEach(function(x,i){$('p'+i).className=(i==s.cur)?'on':''});
 if(!busy){$('br').value=s.br;$('bv').textContent=s.br}
 $('mdbox').style.display=s.dash?'block':'none';
 $('m0').className=s.mode?'':'on';$('m1').className=s.mode?'on':'';
 $('u0').className=s.imp?'':'on';$('u1').className=s.imp?'on':'';
}
$('br').oninput=function(){busy=1;$('bv').textContent=this.value;var v=this.value;clearTimeout(t);
 t=setTimeout(function(){q('/api/br?v='+v).then(function(){busy=0})},80)};
$('m0').onclick=function(){q('/api/mode?v=0').then(draw)};
$('m1').onclick=function(){q('/api/mode?v=1').then(draw)};
$('u0').onclick=function(){q('/api/units?v=0').then(draw)};
$('u1').onclick=function(){q('/api/units?v=1').then(draw)};
function poll(){q('/state').then(draw)}
setInterval(poll,1000);poll();
</script></body></html>)HTML";

// ov* - значення, яке щойно запросили зі сторінки (щоб відповідь одразу показала зміну)
static void sendState(int ovCur = -1, int ovBr = -1, int ovImp = -1, int ovMode = -1) {
  String j;
  j.reserve(400);
  char buf[16];

  int n = nActive;
  int c = (ovCur >= 0) ? ovCur : cur;
  int br = (ovBr >= 0) ? ovBr : (int)brPct;
  int imp = (ovImp >= 0) ? ovImp : (imperial ? 1 : 0);
  int md = DASH_ON ? ((ovMode >= 0) ? ovMode : (int)uiMode) : 0;
  bool run = (state == ST_RUN) && n > 0;

  const char *vtxt = (run && lastText[0]) ? lastText : "--";
  const char *utxt = "";
  if (run && c < n) {
    utxt = imp ? PARAMS[active[c]].unitI : PARAMS[active[c]].unitM;
  }
#if ENABLE_DASH
  if (run && md == 1) {                // на «Панелі» показуємо швидкість
    vtxt = dTxtSpeed[0] ? dTxtSpeed : "--";
    utxt = imp ? "mph" : "km/h";
  }
#endif

  j += "{\"st\":\"";
  j += run ? "run" : "scan";
  j += "\",\"v\":\"";
  j += vtxt;
  j += "\",\"u\":\"";
  j += utxt;
  j += "\",\"cur\":";
  snprintf(buf, sizeof(buf), "%d", (md == 1) ? -1 : c);
  j += buf;
  j += ",\"mode\":";
  j += (md == 1) ? "1" : "0";
  j += ",\"dash\":";
  j += DASH_ON ? "1" : "0";
  j += ",\"imp\":";
  j += imp ? "1" : "0";
  j += ",\"br\":";
  snprintf(buf, sizeof(buf), "%d", br);
  j += buf;
  j += ",\"list\":[";
  for (int i = 0; run && i < n; i++) {
    if (i) {
      j += ",";
    }
    j += "\"";
    j += PARAMS[active[i]].name;
    j += "\"";
  }
  j += "]}";

  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", j);
}

static void wifiRoutes() {
  server.on("/", []() {
    wifiLastAct = millis();
    server.send(200, "text/html; charset=utf-8", PAGE);
  });
  server.on("/state", []() {          // опитування сторінкою НЕ продовжує час роботи Wi-Fi
    sendState();
  });
  server.on("/api/set", []() {
    wifiLastAct = millis();
    int v = server.hasArg("p") ? atoi(server.arg("p").c_str()) : -1;
    if (state == ST_RUN && v >= 0 && v < nActive) {
      cmdSelect = v;
    } else {
      v = -1;
    }
    sendState(v, -1, -1, v >= 0 ? 0 : -1);
  });
  server.on("/api/br", []() {
    wifiLastAct = millis();
    int v = server.hasArg("v") ? atoi(server.arg("v").c_str()) : -1;
    if (v >= 0) {
      v = constrain(v, 5, 100);
      cmdBright = v;
    }
    sendState(-1, v);
  });
  server.on("/api/units", []() {
    wifiLastAct = millis();
    int v = server.hasArg("v") ? (atoi(server.arg("v").c_str()) != 0 ? 1 : 0) : -1;
    if (v >= 0) {
      cmdUnits = v;
    }
    sendState(-1, -1, v);
  });
  server.on("/api/mode", []() {
    wifiLastAct = millis();
    int v = server.hasArg("v") ? (atoi(server.arg("v").c_str()) != 0 ? 1 : 0) : -1;
    if (DASH_ON && state == ST_RUN && v >= 0) {
      cmdMode = v;
    } else {
      v = -1;
    }
    sendState(-1, -1, -1, v);
  });
}

// Завдання веб-сервера: сама мережа вмикається і вимикається тут же
static void webTask(void *) {
  for (;;) {
    int req = wifiReq;

    if (req == 1) {
      wifiReq = 0;
      WiFi.mode(WIFI_AP);
      WiFi.softAP(WIFI_SSID, WIFI_PASS, 1, 0, 2);   // канал 1, не прихована, до 2 клієнтів
      WiFi.setTxPower(WIFI_TX_POWER);
      server.begin();
      wifiLastAct = millis();
      wifiOn = true;
      wifiEvent = 1;
    } else if (req == 2) {
      wifiReq = 0;
      wifiOn = false;          // спершу позначити вимкненим, щоб головний цикл не просив вимкнути вдруге
      server.stop();
      WiFi.softAPdisconnect(true);
      WiFi.mode(WIFI_OFF);
      wifiEvent = 2;
    }

    if (wifiOn) {
      server.handleClient();
    }
    vTaskDelay(pdMS_TO_TICKS(wifiOn ? 3 : 50));
  }
}

static void webBegin() {
  wifiRoutes();
  xTaskCreate(webTask, "web", 8192, NULL, 1, NULL);
}

static void toggleWifi() {
  if (!wifiOn && wifiHot) {
    showOverlay("WI-FI", "HOT", "cooling");   // чип ще гарячий, Wi-Fi не вмикаємо
    return;
  }
  wifiReq = wifiOn ? 2 : 1;
}

// Викликається з головного циклу: виконує команди зі сторінки і стежить за простоєм
static void processWeb() {
  int v;

  if ((v = cmdMode) >= 0) {
    cmdMode = -1;
    setUiMode(v ? UI_DASH : UI_UNI, true);
  }
  if ((v = cmdSelect) >= 0) {
    cmdSelect = -1;
    if (state == ST_RUN) {
      setUiMode(UI_UNI, true);         // вибір параметра з телефона повертає звичайний вигляд
      selectParam(v);
    }
  }
  if ((v = cmdBright) >= 0) {
    cmdBright = -1;
    setBrightness(v);
  }
  if ((v = cmdUnits) >= 0) {
    cmdUnits = -1;
    if ((v != 0) != imperial) {
      toggleUnits();
    }
  }

  v = wifiEvent;
  if (v) {
    wifiEvent = 0;
    if (v == 1) {
      Serial.println("Wi-Fi ON: 192.168.4.1");
      showOverlay("WI-FI", "ON", "192.168.4.1");
    } else {
      Serial.println("Wi-Fi OFF");
      showOverlay("WI-FI", "OFF", "");
    }
  }

  if (WIFI_IDLE_MS > 0 && wifiOn && wifiReq == 0 && (uint32_t)(millis() - wifiLastAct) > WIFI_IDLE_MS) {
    wifiReq = 2;
  }

  // Захист від перегріву: температура кристала раз на 2 секунди (гістерезис ON < OFF)
  static uint32_t lastTempChk = 0;
  if (WIFI_TEMP_OFF > 0 && (uint32_t)(millis() - lastTempChk) >= 2000) {
    lastTempChk = millis();
    float tc = temperatureRead();
    if (!wifiHot && tc >= WIFI_TEMP_OFF) {
      wifiHot = true;
      if (wifiOn && wifiReq == 0) {
        wifiReq = 2;
      }
      Serial.printf("Chip too hot (%.1f C): Wi-Fi blocked\n", tc);
    } else if (wifiHot && tc <= WIFI_TEMP_ON) {
      wifiHot = false;
      Serial.printf("Chip cooled (%.1f C): Wi-Fi allowed\n", tc);
    }
  }
}

#else

static inline void webBegin() {}
static inline void toggleWifi() {}
static inline void processWeb() {}

#endif

// ---------------- КНОПКА ----------------
// Коротко = наступний параметр; 0.7-2 с = одиниці; 2-4 с = яскравість (після відпускання);
// 4 с і більше = Wi-Fi (спрацьовує, поки кнопка ще тримається, щоб було видно, коли відпускати).

static void handleButton() {
  static bool lastRaw = false;
  static bool stable = false;
  static uint32_t changedAt = 0;
  static uint32_t pressedAt = 0;
  static bool wifiDone = false;

  bool raw = (digitalRead(BTN_PIN) == LOW);
  uint32_t now = millis();

  if (raw != lastRaw) {
    lastRaw = raw;
    changedAt = now;
  }

  if ((uint32_t)(now - changedAt) >= 30 && raw != stable) {
    stable = raw;

    if (stable) {
      pressedAt = now;
      wifiDone = false;
    } else if (!wifiDone) {
      uint32_t held = now - pressedAt;

      if (held >= 2000) {
        cycleBrightness();
      } else if (state == ST_RUN) {
        if (held >= 700) {
          toggleUnits();
        } else {
          nextParam();
        }
      }
    }
  }

  if (stable && !wifiDone && (uint32_t)(now - pressedAt) >= 4000) {
    wifiDone = true;
    toggleWifi();
  }
}

// ---------------- ПОШУК АВТО ----------------

static void scanStep() {
  if (!canStart(busIdx)) {
    busIdx ^= 1;
    return;
  }

  uint8_t d[4];
  bool ok = false;

  for (int i = 0; i < 3 && !ok; i++) {
    ok = queryBlocking(0x00, d);
    if (!ok) {
      delay(100);
    }
  }

  if (!ok) {
    canStop();
    busIdx ^= 1;
    return;
  }

  Serial.printf("OBD found, bus %s\n", busIdx == 0 ? "500k" : "250k");

  scanSupported();

  memset(pidHas, 0, sizeof(pidHas));
  memset(lastReq, 0, sizeof(lastReq));
  waitingPid = false;

#if DEBUG_SERIAL
  Serial.print("Supported PIDs:");
  for (int p = 1; p <= 0xE0; p++) {
    if (supported[p]) {
      Serial.printf(" %02X", p);
    }
  }
  Serial.println();
#endif

  overlayUntil = 0;                    // скасувати тимчасове повідомлення (напр. Wi-Fi), якщо воно ще на екрані
  buildActive();
  lastRxOk = millis();
  state = ST_RUN;

  if (savedMode == UI_DASH) {
    setUiMode(UI_DASH, false);         // повернути обраний раніше вигляд «Панель»
  }
}

// ---------------- СТАРТОВА АНІМАЦІЯ ----------------

#if BOOT_ANIM
static void bootAnimation() {
  uint32_t t0 = millis();
  uint32_t lastT = t0;

  while (true) {
    uint32_t now = millis();
    uint32_t t = now - t0;

    if (t >= (uint32_t)(BOOT_UP_MS + BOOT_DOWN_MS)) {
      break;
    }

    float f = (t < (uint32_t)BOOT_UP_MS)
                  ? (float)t / BOOT_UP_MS
                  : 1.0f - (float)(t - BOOT_UP_MS) / BOOT_DOWN_MS;
    f = f * f * (3.0f - 2.0f * f);          // плавний розгін і гальмування

    lv_arc_set_value(W_ARC, (int)(f * 1000.0f));

#if !LV_TICK_CUSTOM
    lv_tick_inc(now - lastT);
    lastT = now;
#endif

    lv_timer_handler();
    delay(5);
  }

  lv_arc_set_value(W_ARC, 0);
  lv_timer_handler();
}
#endif

// ---------------- СОН (авто не знайдено) ----------------

static uint32_t scanSince = 0;          // відколи ми безперервно шукаємо авто

#if !DEMO
static void goToSleep() {
  Serial.println("No car for a long time: deep sleep (wake on CAN activity)");

  showOverlay("SLEEP", "zZz", "wake on CAN");
  for (int i = 0; i < 40; i++) {        // показати повідомлення близько секунди
    lv_tick_inc(25);
    lv_timer_handler();
    delay(25);
  }

  gfx->fillScreen(0x0000);
  gfx->displayOff();

  // Прокинутися, коли трансивер потягне лінію CRX (GPIO4) вниз: на шині з'явився трафік
  pinMode(CAN_RX, INPUT_PULLUP);
  esp_deep_sleep_enable_gpio_wakeup(1ULL << CAN_RX, ESP_GPIO_WAKEUP_GPIO_LOW);
  Serial.flush();
  esp_deep_sleep_start();
}
#endif

// ---------------- SETUP ----------------

void setup() {
  Serial.begin(115200);

  pinMode(BTN_PIN, INPUT_PULLUP);

  prefs.begin("gauge", false);
  imperial = prefs.getBool("imp", false);
  savedPid = prefs.getUChar("pid", 0x0C);
  savedMode = prefs.getUChar("mode", 0) ? UI_DASH : UI_UNI;
  brPct = constrain(prefs.getUChar("brp", 100), 5, 100);
  brScale = (uint16_t)(brPct * 256 / 100);
  webBegin();
#if ENABLE_WIFI && WIFI_AUTOSTART
  wifiReq = 1;                         // увімкнути Wi-Fi одразу після запуску
#endif

  if (!gfx->begin()) {
    Serial.println("Display initialization failed");
  }
  gfx->fillScreen(0x0000);

  lv_init();
  lv_disp_draw_buf_init(&drawBuf, buf1, NULL, 240 * 30);
  lv_disp_drv_init(&dispDrv);
  dispDrv.hor_res = 240;
  dispDrv.ver_res = 240;
  dispDrv.flush_cb = flushCb;
  dispDrv.draw_buf = &drawBuf;
  lv_disp_drv_register(&dispDrv);
  lv_timer_set_period(lv_disp_get_default()->refr_timer, 15);   // частіше перемальовувати (за замовчуванням 30 мс)

  ui_init();
  lv_scr_load(W_SCREEN);               // стартуємо зі звичайного екрана

  // Дуга показує проміле (0..1000) від діапазону поточного параметра
  lv_arc_set_range(W_ARC, 0, 1000);
  lv_arc_set_value(W_ARC, 0);
  lv_obj_clear_flag(W_ARC, LV_OBJ_FLAG_CLICKABLE);

#if ENABLE_DASH
  lv_arc_set_range(D_ARC, 0, 1000);
  lv_arc_set_value(D_ARC, 0);
  lv_obj_clear_flag(D_ARC, LV_OBJ_FLAG_CLICKABLE);
#endif

#if BOOT_ANIM
  showStatus("STARTING");
  bootAnimation();
#endif

#if DEMO
  Serial.println("DEMO mode");
  buildActive();
  state = ST_RUN;
  if (savedMode == UI_DASH) {
    setUiMode(UI_DASH, false);
  }
#else
  showStatus("STARTING");
#endif

  scanSince = millis();
}

// ---------------- LOOP ----------------

void loop() {
#if !LV_TICK_CUSTOM
  static uint32_t lastTick = 0;
  uint32_t nowMs = millis();
  lv_tick_inc(nowMs - lastTick);
  lastTick = nowMs;
#endif

#if !DEMO
  if (state == ST_SCAN) {
#if ENABLE_WIFI
    if (wifiOn && (uint32_t)(millis() - wifiLastAct) < 120000) {
      scanSince = millis();            // ви працюєте зі сторінкою: не засинати
    }
#endif
    if (SLEEP_AFTER_MS > 0 && (uint32_t)(millis() - scanSince) >= SLEEP_AFTER_MS) {
      goToSleep();
    }
    if ((int32_t)(millis() - nextScanAt) >= 0) {
      if (!overlayActive()) {
        showStatus(busIdx == 0 ? "SEARCH 500k" : "SEARCH 250k");
        lv_timer_handler();        // показати статус до блокувального пошуку
      }
      scanStep();
      nextScanAt = millis() + 500;
    }
  } else {
    pollCan();

    // Авто «заснуло» або кабель вийняли: повернутися до пошуку
    if ((uint32_t)(millis() - lastRxOk) > 8000) {
      Serial.println("No response from car, searching again");
      canStop();
      state = ST_SCAN;
      nextScanAt = 0;
      scanSince = millis();
    }
  }
#endif

  handleButton();

  processWeb();

  if (brSavePending && (int32_t)(millis() - brSaveAt) >= 0) {
    brSavePending = false;
    prefs.putUChar("brp", brPct);
  }

  static uint32_t lastUiUpdate = 0;
  uint32_t now = millis();

  if (uiDirty || (uint32_t)(now - lastUiUpdate) >= 50) {
    uiDirty = false;
    lastUiUpdate = now;
    refreshUi();
  }

#if DEBUG_SERIAL
  static uint32_t lastPrint = 0;
  if (state == ST_RUN && (uint32_t)(millis() - lastPrint) >= 1000) {
    lastPrint = millis();
    const Param &p = PARAMS[active[cur]];
    uint32_t cnt = rxCount;
    uint32_t avgUs = cnt ? rttSumUs / cnt : 0;
    rxCount = 0;
    rttSumUs = 0;
    Serial.printf("%s = %s %s   (%d/%d)   rate=%u/s  reply=%u.%u ms\n", p.name, lastText, unitOf(p),
                  cur + 1, nActive, (unsigned)cnt, (unsigned)(avgUs / 1000), (unsigned)((avgUs % 1000) / 100));
  }
#endif

  lv_timer_handler();
  delay(2);
}
