### 1  Direct‑Attached Storage (DAS)

| Storage option    | Usable capacity                 | Typical module / IC      | Bus          | Pros                                      | Cons                                  | Best use‑case                  |
| ----------------- | ------------------------------- | ------------------------ | ------------ | ----------------------------------------- | ------------------------------------- | ------------------------------ |
| **MicroSD card**  | Up to 1 TB                      | SparkFun BOB‑00544 (3 V) | SPI or SDMMC | Huge capacity, hot‑swappable, FAT32/exFAT | Highest idle current; removable media | High‑volume data logging       |
| **SPI NOR Flash** | 8 Mbit – 128 Mbit (1 – 16 MB)   | Adafruit W25Q128JV       | SPI          | Tiny, ≈ 1 µA power‑down, inexpensive      | ≤ 16 MB; block‑erase                  | Firmware assets, circular logs |
| **I²C EEPROM**    | 8 Kbit – 1 Mbit (1 KB – 128 KB) | Microchip 24LC1025       | I²C          | 2‑wire, ultra‑low‑power                   | Slow writes; very small               | Calibration blobs              |

> **Tip – LittleFS vs. SPIFFS**  
> LittleFS is the recommended flash file‑system on ESP32; SPIFFS remains available for legacy sketches.

#### 1.1  MicroSD Card Integration

* **Breakout:** SparkFun MicroSD Transflash Breakout (BOB‑00544) or any 3 V‑ready socket.
* **SPI wiring:**

| SD pin | ESP32 pin (suggested) |
| ------ | --------------------- |
| CS     | GPIO 5                |
| MOSI   | GPIO 23               |
| MISO   | GPIO 19               |
| SCK    | GPIO 18               |

* **Firmware:** `esp_vfs_fat_sdmmc_mount()` (ESP‑IDF) or `SD.begin(csPin)` (Arduino); supports FAT32 / exFAT.

#### 1.2  External SPI NOR Flash

* Breakout Adafruit W25Q128JV (**16 MB / 128 Mbit**). Stand‑by current ≈ 1 µA.
* **Firmware:** LittleFS (**recommended**) or legacy SPIFFS (`esp_vfs_littlefs_register()`).
* **Partitioning example (OTA + data):** 3×1.8 MB firmware slots + 8 MB LittleFS.

#### 1.3  I²C EEPROM (24LC1025)

* **Capacity:** 1 Mbit = 128 KB, exposed as two 64 KB banks (I²C 0x50 & 0x54).
* **Bus:** default I²C (GPIO 21 / 22). Library: `I2C_24LC1025`.
* Ideal for credentials or calibration constants that must survive full flash erase.

#### 1.4  USB‑to‑UART Serial Console

1. USB‑C port → CH9102F (or CP2102N on early units) to ESP32 UART0.
2. Flash or monitor at **115 200 baud** (up to 921 600 supported).
3. Auto‑reset/bootloader wiring means no buttons required during upload.
4. **Drivers:** Windows 10/11 auto‑installs; macOS may need the CH9102 driver (link in References).

#### 1.5  Power & Reliability Tips

* 3.3 V‑only peripherals; enable brown‑out detection.
* A **Li‑Po battery is mandatory for Wi‑Fi TX peaks**; USB alone can brown‑out (\~380 mA spikes).
* Enable wear‑levelling for SD/NOR flash; issue deep‑power‑down `0xB9` to W25Q chips; gate SD power with a PFET during sleep.

#### 1.6  File‑System Choices at a Glance

| File‑system   | Location      | Pros                        | Cons                 |
| ------------- | ------------- | --------------------------- | -------------------- |
| **LittleFS**  | SPI/NOR flash | Robust, power‑loss tolerant | Slightly larger code |
| **SPIFFS**    | SPI/NOR flash | Legacy, tiny footprint      | Susceptible to wear  |
| **FAT/exFAT** | SD card       | PC‑compatible, >4 GB files  | Higher overhead      |

#### 1.7  Typical Flash Partition (16 MB W25Q128JV)

```text
# Name        Type  Sub  Offset   Size
nvs           data  nvs  0x9000   20K
phy_init      data  phy  0xF000   4K
factory       app   ota_0 0x10000 1.8M
ota_1         app   ota_1 0x200000 1.8M
datafs        data  spiffs 0x380000 8M
```

---

### 2  Pin Map & Reserved Signals

| GPIO  | Function on Bat Pro             | Notes                                  |
| ----- | ------------------------------- | -------------------------------------- |
| 0     | Boot‑strapping (BOOT)           | Leave un‑pulled in user circuits       |
| 2     | Boot‑strapping                  | RGB LED (BLUE), Low at boot → failsafe |
| 4     | RGB LED (GREEN)                 | PWM capable                            |
| 5     | SD CS (example)                 | VSPI CS                                |
| 12    | Boot‑strapping (MTDI)           | Keep low at boot                       |
| 13    | RGB LED (RED)                   | —                                      |
| 18    | VSPI CLK / SD SCK               | —                                      |
| 19    | VSPI MISO / LC709203F ALERT INT | Shared input                           |
| 21    | I²C SDA                         | Fuel‑gauge + EEPROM                    |
| 22    | I²C SCL                         | —                                      |
| 23    | VSPI MOSI / SD MOSI             | —                                      |
| 34‑39 | **Input‑only** (GPIO34‑39)      | No output/PWM & no pull‑ups            |

*(Pins not listed are free for general‑purpose use.)*

---

### 3  Typical Wiring Examples

* **MicroSD (VSPI mode):** connect as in table above; add 47 k pull‑ups on CMD/DAT when using SDMMC 4‑bit.
* **Qwiic / STEMMA QT sensor:** plug into JST‑SH port on I²C bus (3.3 V).
* **Li‑Po + USB:** JST‑PH battery + USB‑C for charging; polarity silk‑screened (+ is wire red).

---
### 1  Board Overview – ESP32\_Bat\_Pro

* **MCU:** ESP32‑D0WD‑V3 dual‑core LX6 @ 240 MHz with 4 MB internal flash **plus** 16 MB external W25Q128JV; Wi‑Fi 802.11 b/g/n + Bluetooth 4.2 (Classic BR/EDR + BLE).
* **Power:** 3 V–6 V VIN; 400 mA Li‑Po charger; deep‑sleep ≈ 12 µA (fuel‑gauge active).
* **Peripherals:** 3 × SPI, 2 × I²C, SDMMC, 12‑bit ADC, 2‑ch DAC; USB‑UART (CH9102F/CP2102N).
* **Connectors:** USB‑C, JST‑PH battery, full breadboard header (GPIO34‑39 input‑only).

#### 1.1  Power Consumption Reference

| Mode                              | Typical current    | Conditions             |
| --------------------------------- | ------------------ | ---------------------- |
| Deep‑sleep (RTC timer, gauge on)  | < 12 µA            | USB‑UART + RGB LED off |
| Light‑sleep (CPU halt, Wi‑Fi off) | 0.8 mA             | 150 ms DTIM            |
| Idle (CPU idle)                   | 35 mA              | Wi‑Fi & BT off         |
| Wi‑Fi connected (idle)            | 80 mA              | DTIM3, RSSI −55 dBm    |
| Wi‑Fi TX peak                     | 240–380 mA         | 18 dBm output          |
| BLE advertising 1 Hz              | 14 mA (avg ≈ 3 mA) | 0 dBm                  |

### 2  Storage & Data‑Access Highlights

* **Flash:** 16 MB LittleFS (default) or SPIFFS; supports OTA (dual‑slot).
* **SD card:** up to 1 TB, FAT32/exFAT.
* **USB serial:** CH9102F driver available for macOS (see References).
* **Wireless:** BLE GATT service & HTTP `/data` endpoint over Wi‑Fi AP.

### 3  Typical End‑User Workflow

1. Insert MicroSD → logging starts automatically.
2. Connect USB‑C → serial console appears for logs & flashing.
3. Optionally connect via BLE (SensorShield) or join Wi‑Fi AP `SensorShield_Test` to pull data.

---
*(Sensor‑Shield firmware, scripts & API)*
### 1  Storage Capacity Metrics

* **LittleFS volume:** 1 318 001 bytes (≈ 1.26 MB).
* **Average record:** 251 bytes JSON.
* **Max records:** ≈ 5 251 before rollover.

### 2  Data Retrieval Methods

#### 2.1  USB Serial (Real‑time)

```bash
screen /dev/ttyUSB0 115200  # Linux & macOS
```

Python quick‑logger:

```python
import serial, time
ser = serial.Serial('/dev/ttyUSB0', 115200)
while True:
    if ser.in_waiting:
        print(ser.readline().decode().strip())
    time.sleep(0.1)
```

#### 2.2  Wi‑Fi Access‑Point

* **SSID:** `SensorShield_Test`  ·  **Password:** `test123`
* **Gateway:** `192.168.4.1`
* **Endpoints:** `/` dashboard, `/data`, `/status`, `/clear`

```bash
curl http://192.168.4.1/data > data.json
```

#### 2.3  BLE Data Dump

1. nRF Connect → scan → **SensorShield**.
2. Enable notifications on **LogData**, write `DUMP` to **LogCtrl**.
3. Save packets until `EOF`.

### 3  Fuel‑Gauge (LC709203F) Quick‑Start

```cpp
#include <Wire.h>
#include "LC709203F.h"

LC709203F gauge;

void setup() {
  Wire.begin();                 // SDA 21, SCL 22
  gauge.begin();
  gauge.setPackSize(LC709203F_APA_2000mAh); // match your cell
}

void loop() {
  Serial.printf("Vbat: %.3f V, SOC: %.1f %%\n", gauge.cellVoltage(), gauge.cellPercent());
  delay(5000);
}
```

### 4  BLE Usage Guidance

* Default device name: **SensorShield**.
* Custom service UUID exposes `LogCtrl` (Write‑No‑Rsp) and `LogData` (Notify).
* BLE & Wi‑Fi share the radio; concurrent use reduces throughput.
* Long‑range PHY (Coded 125 kbps) not supported (BT4.2 only).

### 5  Troubleshooting Cheat‑Sheet

| Issue               | Likely cause                 | Quick fix                                  |
| ------------------- | ---------------------------- | ------------------------------------------ |
| USB not enumerating | Missing 5 V or bad cable     | Replace cable / ensure 5 V present         |
| No Wi‑Fi AP         | Firmware crash or deep‑sleep | Reset board; ensure Li‑Po connected        |
| BLE throughput slow | MTU left at 23 bytes         | Increase MTU; keep chunks ≤ 180 B          |
| SD errors           | Long wires or brown‑out      | Shorten traces; enable brown‑out detection |

---

## References

1. Espressif ESP32‑WROOM‑32E Datasheet
2. Winbond W25Q128JV Datasheet
3. Microchip 24LC1025 Datasheet
4. Espressif ESP‑IDF Flash File‑System Guide (LittleFS/SPIFFS)
5. CH9102F macOS Driver – [https://wch.cn/downloads](https://wch.cn/downloads)
---

### General Information
* **Product Name:** ESP32 Bat Pro
* **Manufacturer:** EzSBC
* **Origin:** Designed and made in the USA
* **SKU:** ESP32U-01
* **Description:** An all-in-one ESP32-WROOM-32E board with an onboard Li-Po charger, fuel-gauge, USB-to-serial converter, and deep-sleep optimization. It is designed to be breadboard-friendly.

### Core Components & Memory
* **Processor Module:** ESP32-WROOM-32E, featuring a dual-core LX6 CPU at 240 MHz.
* **Flash Memory:** The board is fitted with a W25Q128JV chip, providing **16 MByte** of SPI flash memory.
* **EEPROM:** A Microchip 24LC1025 I²C EEPROM with 128 KByte capacity. It appears as two 64KB blocks at I²C addresses 0x50 and 0x54.
* **Fuel Gauge:** An LC709203F I²C fuel gauge.
    * It operates at I²C address 0x0B.
    * It provides an interrupt on State of Charge (SOC) or voltage, which is wired to GPIO 19.
    * It does not use Coulomb counting, which prevents long-term drift.
    * It should be configured in software using the provided driver to match the battery cell size (e.g., `LC709203_NOM3p7_Charge4p2` for most Li-Po cells).
* **Low-Frequency Clock:** An on-board 32.768 kHz crystal is included for accurate deep-sleep and RTC timing.
* **USB-to-Serial Chip:** A CH9102F (or CP2102N in some units).
    * It supports auto-download and auto-reset, so no manual button pressing is needed for programming.
    * The default upload speed is 921,600 bps.

### Power & Electrical Specifications
* **Input Voltage:** 3.0 V to 6.0 V on the VIN pin or USB-C port (when no battery is attached).
* **Deep Sleep Current:** Less than 12 µA with the fuel gauge active.
* **Active Current Consumption:**
    * **WiFi Active (CPU idle):** ~80 mA
    * **Bluetooth Active:** ~120 mA
    * **WiFi Transmitting (average):** Up to 240 mA
    * **WiFi Transmitting (peak):** Up to 380 mA
* **Li-Po Charger:**
    * Provides a 400 mA Constant Current/Constant Voltage (CC/CV) charge profile.
    * Charges Li-Po cells to a maximum of 4.2 V.
    * Features auto-recharge when external power is present.
* **LDO:** A low-IQ (low quiescent current) 3.3V LDO is used to extend battery life.

### Connectivity & Interfaces
* **Antenna:** On-board PCB trace antenna.
* **Bluetooth:** Bluetooth v4.2, supporting both Classic (BR/EDR) and Bluetooth Low Energy (BLE).
* **WiFi:** 802.11 b/g/n (HT40). A LiPo battery **is required** for stability during WiFi transmission peaks.
* **UART:** 3 hardware UART controllers (one with hardware flow control).
* **SPI:** 3 hardware SPI buses.
* **I²C:** 2 hardware I²C controllers. The bus is used by the onboard fuel gauge and EEPROM.
* **I²S:** 2 hardware I²S audio buses.
* **ADC:** 12 channels of 12-bit Analog-to-Digital Converters.
* **DAC:** 2 channels of 8-bit Digital-to-Analog Converters.
* **PWM:** PWM output is available on every GPIO pin (except input-only pins).
* **SD/SDIO:** Features a native SD/MMC interface and supports SDIO Master/Slave up to 50 MHz.

### Mechanical, I/O & LEDs
* **Form Factor:** Fits a standard breadboard, leaving one row of holes accessible on each side. All module GPIOs are broken out.
* **GPIO Pins:**
    * **Input-Only:** GPIO pins 34 through 39 are input-only and have no internal pull-ups/downs.
    * **Strapping Pins:** GPIOs 0, 2, 12, and 15 are used for boot mode configuration.
* **Connectors:**
    * **Battery:** JST-PH 2-pin connector.
    * **External Power:** USB-C port or a 2-pin header for Vin.
* **LEDs:** Two RGB LEDs are on the board.
    * One indicates USB activity (red/green) and is controlled by the CH9102F.
    * One is a user-programmable RGB LED tied to three ESP32 GPIO pins.

### Pricing & Availability (as of document date)
* **Base Price:** US $12.95 (for 1-3 units)
* **Volume Pricing:** US $11.95 (4-9 units), US $11.50 (10+ units)
* **Related Products:**
    * 1,000 mAh Li-Po Cell: $5.95
    * 2,000 mAh Li-Po Cell: $7.95





