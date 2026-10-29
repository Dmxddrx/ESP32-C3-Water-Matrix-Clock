# ESP32-C3 Water Matrix Clock

Firmware for an ESP32-C3 Super Mini that drives two chained MAX7219 8×8 matrices as a 16×8 tilt-responsive water display. It reads an MPU6050 over I²C, shows date/time and status on an I²C OLED, synchronizes time over Wi-Fi/NTP, accepts simple Bluetooth Low Energy (BLE) commands, and reads two tactile buttons.

## Project status

**Firmware scaffold implemented; hardware validation still required.** Drivers and the non-blocking application loop are included, but this project has not been compiled against your exact ESP32 Arduino core/display module or tested on your physical hardware yet. The OLED implementation assumes a 128×64 SSD1306 at I²C address `0x3C`; see the configuration section if your display is different.

## Features

- Two daisy-chained MAX7219 modules rendered as one 16×8 framebuffer.
- Water surface animation responds to MPU6050 pitch and roll via a complementary filter.
- Matrix modes: `WATER`, `HEART`, and `LEVEL`.
- SSD1306 OLED pages for time/date, orientation, and network/device status.
- Wi-Fi station connection and NTP time synchronization in Sri Lanka Standard Time (UTC+05:30).
- Time continues from the ESP32 system clock if Wi-Fi temporarily disconnects after synchronization.
- Two debounced tactile buttons.
- BLE GATT command and status characteristics; test using a BLE phone app such as nRF Connect.
- Task watchdog setup in the `.ino`; the main loop uses non-blocking periodic updates.
- Font bitmaps (`5×7`, `6×8`, `7×10`, `11×18`, and `16×26`) adapted from the font source you supplied for the STM32 project.

## Recommended wiring

Your proposed I²C and MOSI connections are usable. I recommend moving matrix CLK and CS off GPIO8/GPIO9 before using this pinout: GPIO8 and GPIO9 are ESP32-C3 strapping pins sampled during reset and can make startup less reliable when attached to external circuitry. GPIO4–GPIO7 are JTAG-capable pins; because GPIO6/GPIO7 are used for I²C as well, this pinout assumes you debug over USB/Serial rather than external JTAG.

| Device | Signal | ESP32-C3 Super Mini |
|---|---|---:|
| MPU6050 + SSD1306 OLED | SDA | GPIO6 |
| MPU6050 + SSD1306 OLED | SCL | GPIO7 |
| Both MAX7219 modules | CLK | GPIO5 |
| First MAX7219 module | DIN | GPIO10 (MOSI) |
| Both MAX7219 modules | CS / LOAD | GPIO4 |
| Tactile button 1 | GPIO to GND when pressed | GPIO0 |
| Tactile button 2 | GPIO to GND when pressed | GPIO3 |
| I²C devices | VCC | 3V3, only if supported by each module |
| ESP32-C3 and all modules | GND | Common ground |
| MAX7219 modules | V+ | Regulated 5 V |

The current constants are in `project_config.h`. If you keep your original wiring (`CLK=GPIO8`, `DIN=GPIO10`, `CS=GPIO9`), change the two matrix pin constants there, but be aware that this uses both GPIO8 and GPIO9 strapping pins. GPIO9 should remain HIGH while the chip samples boot mode.

### MAX7219 logic and power

- Use a 5 V-powered `74AHCT125` or `74HCT125` buffer for DIN, CLK, and CS/LOAD. The MAX7219's guaranteed logic-high threshold at 5 V is above the ESP32-C3's 3.3 V logic-high level, so direct drive is not guaranteed.
- Tie the buffer's active-low `/OE` input LOW and share ground between the ESP32-C3, buffer, and matrix supply.
- The MAX7219 modules are powered from 5 V; do not feed 5 V signals back into ESP32-C3 GPIOs.
- The Super Mini `5V` pad is normally the USB 5 V rail, not a separate high-current regulator output. Two matrix modules can draw significant current. A separate regulated 5 V supply for the matrices is recommended; connect its GND to the ESP32-C3 GND. Do not exceed the USB supply/board current capability.
- The MAX7219 matrices are daisy-chained: ESP32 DIN/MOSI → first module DIN, first module DOUT → second module DIN; CLK and CS are shared. The driver maps the logical left half to the far/second module to match your supplied STM32 driver. If the physical order is reversed, change the mapping in `MAX7219_SetPixel()`.

### I²C devices

The MPU6050 and OLED share SDA/SCL. Typical addresses are `0x68`/`0x69` for the MPU6050 and `0x3C`/`0x3D` for many SSD1306 displays. The MPU driver checks both MPU addresses. The OLED address is set in `project_config.h`. Ensure any I²C pull-ups go to 3.3 V, not 5 V.

## Software requirements

1. Install the **ESP32 by Espressif Systems** board package in Arduino IDE.
2. Select an ESP32-C3 target, for example **ESP32C3 Dev Module**, compatible with your installed core.
3. Install these libraries using Arduino IDE Library Manager:
   - **Adafruit SSD1306**
   - **Adafruit GFX Library**
4. The BLE API used here is supplied by the ESP32 Arduino board package; a separate NimBLE-Arduino library is not required for this version of the sketch.
5. Open `ESP32C3_Water_Matrix_Clock.ino` from this folder and compile/upload. Use Serial Monitor at **115200 baud**.

The project is split into `.cpp`/`.h` modules, but Arduino IDE expects the main `.ino` filename to match the sketch folder name.

## Wi-Fi credentials

1. Copy `arduino_secrets.example.h` to `arduino_secrets.h` in this folder.
2. Set your SSID and password:

   ```cpp
   #define WIFI_SSID     "your_wifi_name"
   #define WIFI_PASSWORD "your_wifi_password"
   ```

3. Do not commit `arduino_secrets.h`; it is listed in `.gitignore`.

If credentials are not configured, the device still runs the matrices, MPU6050, OLED status pages, buttons, and BLE; the clock page will show that it is waiting for NTP.

## Controls

- **Button on GPIO0:** cycle OLED pages — clock/date, orientation, device/network status.
- **Button on GPIO3:** cycle matrix mode — WATER, HEART, LEVEL.
- **BLE device name:** `WaterMatrixClock`.
- **BLE command characteristic:** write a UTF-8 command string:

  | Command | Result |
  |---|---|
  | `MODE=WATER` | Tilt-responsive water with small moving bubbles |
  | `MODE=HEART` | Two heart icons with a subtle brightness pulse |
  | `MODE=LEVEL` | Tilt-responsive liquid level without wave ripples |
  | `PAGE=0` | OLED clock/date page |
  | `PAGE=1` | OLED orientation page |
  | `PAGE=2` | OLED network/status page |
  | `BRIGHT=0` through `BRIGHT=15` | Set MAX7219 brightness |
  | `NEXTMODE` | Cycle matrix modes |

- **BLE status characteristic:** read or subscribe to notifications for pitch, roll, Wi-Fi, time-sync status, matrix mode, and brightness.

## Module layout

```text
ESP32C3_Water_Matrix_Clock.ino  # setup, main loop, task watchdog
project_config.h               # pins and timing constants
max7219.cpp / max7219.h        # SPI chain, framebuffer, text/icons
mpu6050.cpp / mpu6050.h        # direct I2C driver and pitch/roll filter
fonts.cpp / fonts.h            # portable bitmap font tables
icons.cpp / icons.h            # 8x8 heart and smiley icons
wifi_manager.cpp / wifi_manager.h
bluetooth_manager.cpp / bluetooth_manager.h
display_manager.cpp / display_manager.h  # SSD1306 clock/status UI
general.cpp / general.h        # buttons, animation, integration loop
arduino_secrets.example.h      # safe Wi-Fi credential template
```

## First hardware test sequence

1. Confirm the pin changes in the wiring table and power the MAX7219 boards from a stable 5 V supply through the level shifter.
2. Upload, open Serial Monitor, and check for `MPU6050: ready`, `OLED: SSD1306 initialized`, and `BLE: advertising` messages.
3. If the MPU6050 is not detected, confirm SDA=GPIO6, SCL=GPIO7, common ground, 3.3 V supply, and its `0x68`/`0x69` address.
4. If the OLED is not detected, confirm it is really an SSD1306 128×64 module and adjust `OLED_I2C_ADDRESS` or the display driver if required.
5. Verify the matrix orientation, chain order, and brightness. If pixels are mirrored or upside down, adjust the mapping in `MAX7219_SetPixel()`.
6. Configure Wi-Fi and confirm the clock synchronizes; then test the two buttons and BLE commands.

## Notes and limitations

- MPU6050 pitch/roll are estimates, not a calibrated absolute orientation reference. The complementary filter assumes the device is not undergoing sustained strong linear acceleration. Axis direction may need adjustment to match the physical module orientation.
- The MPU6050 has no magnetometer, so this project does not calculate absolute compass heading/yaw.
- `MAX7219_ScrollText()`, `MAX7219_ScrollIcon()`, and `MAX7219_ScrollTextWithIcon()` are provided as blocking convenience functions for simple demonstrations. Do not call them from the regular application loop because they pause sensor/network/button updates.
- This firmware should be compiled and tested with your precise board package version and physical modules before being treated as a finished product.
