# Програмна частина

## 1. Arduino IDE

- Arduino IDE 2.x.
- Пакет плат **esp32 by Espressif Systems** (проєкт збирався з ядром 3.x). Якщо при встановленні чи завантаженні виникають помилки `esptool`, видаліть пакет повністю й встановіть заново через Boards Manager.

## 2. Бібліотеки (Library Manager)

| Бібліотека | Версія |
|---|---|
| **lvgl** | **8.3.11** (не 9.x) |
| **GFX Library for Arduino** (Moon On Our Nation) | остання |

## 3. lv_conf.h

Скопіюйте [`firmware/lv_conf.h`](../firmware/lv_conf.h) у папку `Документи/Arduino/libraries/` **поруч** із папкою `lvgl` (а не всередину неї).

Відмінності від шаблону LVGL:

- `#if 1` на початку файлу (вмикає вміст);
- `LV_COLOR_DEPTH 16`, `LV_COLOR_16_SWAP 0`;
- `LV_TICK_CUSTOM 0` (відлік часу для LVGL робить сам скетч);
- увімкнені шрифти Montserrat **8, 12, 14, 18, 30**. Якщо додасте у SquareLine інший розмір, увімкніть відповідний `LV_FONT_MONTSERRAT_xx`.

## 4. Налаштування плати (Tools)

| Параметр | Значення |
|---|---|
| Board | **ESP32C3 Dev Module** |
| USB CDC On Boot | **Enabled** (інакше Serial Monitor порожній) |
| Partition Scheme | **Huge APP (3MB No OTA/1MB SPIFFS)** |
| Flash Size | 4 MB |
| Core Debug Level | None |

Прошивка займає приблизно 1.36 МБ (близько 43 % розділу Huge APP). Зі стандартним розділом (1.3 МБ) вона не вміщається.

> Тримайте проєкт у простому шляху без пробілів (наприклад, `D:\arduino\speedo_universal`).

## 5. Інтерфейс у SquareLine Studio

Інтерфейс спроєктовано в **SquareLine Studio 1.6.2**. Параметри проєкту:

| Параметр | Значення |
|---|---|
| Resolution | 240 × 240 |
| Color depth | **16 bit** (не «16 bit swap») |
| Board / LVGL | Arduino, LVGL **8.3.11** |
| Save screens separately | увімкнено (окремий файл на кожен екран) |
| Імена віджетів | мають починатися з `ui_` (стандартний префікс) |

Скетч звертається до віджетів за іменами. Вони задані макросами `W_*` (звичайний екран) і `D_*` (панель) вгорі `speedo_universal.ino`. Якщо ваші імена інші, змініть макроси.

### Екран `Screen1` (універсальний)

Фон `000000`, у Flags зніміть Scrollable.

| Імʼя в коді | Віджет | Призначення |
|---|---|---|
| `ui_Screen1` | Screen | звичайний екран |
| `ui_ArciUni` | Arc 235×235 | дуга: кути 135→45, ширина індикатора 20, Knob opacity 0, Clickable вимкнено. Діапазон код встановлює сам (0…1000) |
| `ui_LabelUniValue` | Label | велике число, шрифт `Font2` (Russo One 55 px, символи ASCII 0x20–0x7F) |
| `ui_LabelUniName` | Label | назва параметра, Montserrat 12, зверху (Y ≈ −50) |
| `ui_LabelUniUnit` | Label | одиниці, Montserrat 14, знизу (Y ≈ +50) |
| `ui_PanelDots` | Panel ≈ 100×27 | порожня прозора панель унизу (Y ≈ +100), без рамки: крапки-індикатор створює код |

### Екран `ScreenDash` («Панель»)

| Імʼя в коді | Віджет | Призначення |
|---|---|---|
| `ui_ScreenDash` | Screen | екран «Панель» |
| `ui_ArciDashRpm` | Arc | дуга обертів (як на першому екрані) |
| `ui_LabelDashSpeed` | Label | швидкість, шрифт `Font2`, по центру |
| `ui_LabelDashSpeedUnitkmh` | Label | км/год або mph, Montserrat 14 |
| `ui_LabelDashRpm` | Label | обороти текстом, Montserrat 18, зверху |
| `ui_LabelDashTemp` | Label | температура ОЖ, Montserrat 14, ліворуч унизу |
| `ui_LabelDashBoost` | Label | boost, Montserrat 14, праворуч унизу |

Підписи `TEMP` і `BOOST` це звичайні статичні Label, код їх не чіпає.
Не маєте екрана «Панель»? Поставте `ENABLE_DASH 0`.

### Експорт

1. **Export → Export UI Files**.
2. Скопіюйте **всі** експортовані файли просто в папку скетча (без підпапок), **повністю замінивши** попередні. Видаліть залишки старих експортів (наприклад, `ui_Screen2.*`, зайві `ui_img_*.c`).
3. Збирайте. Змішування файлів із різних експортів або різних версій LVGL дає помилки типу `lv_image_dsc_t`.

## 6. Збірка й прошивка

1. Відкрийте `firmware/speedo_universal/speedo_universal.ino`.
2. Змініть `WIFI_PASS`.
3. `DEMO 1` для першого запуску без авто, потім `DEMO 0`.
4. **Відключіть buck-конвертер**, підключіть плату по USB, натисніть Upload.
5. Відкрийте Serial Monitor (115200): там видно знайдену швидкість шини, список підтримуваних PID і раз на секунду значення з частотою відповідей.
