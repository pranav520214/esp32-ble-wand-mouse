# ESP32 BLE Wand Mouse

A gesture-controlled BLE air mouse / wand mouse built with an ESP32, an MPU6500 IMU, and an IR touch/click sensor.

The sketch advertises as **ESP32 Air Mouse** over Bluetooth Low Energy and converts wand movement into mouse cursor movement, scrolling, left-click, drag, drop gesture, and calibration actions.

## Features

- BLE mouse support using `BleMouse.h`
- MPU6500 gyroscope + accelerometer motion tracking
- Kalman filtering and smoothing for cursor stability
- IR sensor click input on GPIO 27
- Short press click, hold-to-scroll, hold-to-drag behavior
- Drop gesture support while dragging
- Saved IMU calibration using ESP32 `Preferences`
- Optional 4-corner screen calibration
- Serial Monitor commands for debugging, calibration, screen size, and testing

## Hardware

| Part | Purpose |
|---|---|
| ESP32 development board | Main controller + BLE HID device |
| MPU6500 IMU | Gyroscope and accelerometer motion tracking |
| IR touch/proximity sensor | Click / scroll / drag input |
| USB cable | Programming and serial monitor |

## Wiring used by this sketch

| Signal | ESP32 pin |
|---|---:|
| MPU6500 SDA | GPIO 21 |
| MPU6500 SCL | GPIO 22 |
| IR click sensor output | GPIO 27 |
| I2C address | `0x68` or `0x69` |

The code uses `INPUT_PULLUP` for the IR sensor and assumes the IR signal is **active LOW**.

## Software requirements

Install these before uploading:

1. Arduino IDE or Arduino CLI
2. ESP32 board support package
3. A BLE mouse library that provides `BleMouse.h`

## Uploading

1. Open `WandMouse_Stable/WandMouse_Stable.ino` in Arduino IDE.
2. Select your ESP32 board.
3. Select the correct USB port.
4. Install the missing libraries if Arduino reports them.
5. Upload the sketch.
6. Open Serial Monitor at **115200 baud**.
7. Pair your computer/phone with the BLE device named **ESP32 Air Mouse**.

## First run

On first boot, keep the wand still while IMU calibration runs. After pairing, movement should control the cursor.

Useful Serial Monitor commands:

| Command | Action |
|---|---|
| `help` | Show command list |
| `debug` | Toggle debug output |
| `imu_cal` | Recalibrate gyro + accelerometer baseline |
| `imu_reset` | Clear saved IMU calibration |
| `cal` | Start 4-corner screen calibration |
| `resetcal` | Clear screen calibration |
| `cal_status` | Print calibration status |
| `click` | Send a test left-click |
| `center` | Reset aim / virtual cursor to screen center |
| `size 1920 1080` | Set screen size |
| `scroll on` / `scroll off` | Force MPU scroll mode |

## Videos and demos

Put small demo videos in the `videos/` folder, or host larger videos externally and link them from this README.

Recommended README format:

```markdown
## Demo

[Watch the demo video](videos/demo.mp4)
```

For large videos, use Git LFS or upload the video to a video platform and link it here.

## Project structure

```text
esp32-ble-wand-mouse/
├── WandMouse_Stable/
│   └── WandMouse_Stable.ino
├── docs/
│   ├── setup.md
│   ├── commands.md
│   └── publishing-to-github.md
├── videos/
│   └── README.md
├── images/
│   └── .gitkeep
├── .gitignore
├── .gitattributes
├── LICENSE
└── README.md
```

## Troubleshooting

### MPU6500 not detected

- Check SDA/SCL wiring.
- Check power and ground.
- Confirm the MPU6500 address is `0x68` or `0x69`.
- Keep I2C wires short.

### Cursor drifts while idle

- Run `imu_cal` while the wand is perfectly still.
- Increase `DEADZONE` slightly in the sketch.
- Increase `STILL_GYRO_THRESHOLD` only if the wand locks too aggressively.

### Cursor direction is reversed

Adjust these constants in the sketch:

```cpp
const float CURSOR_X_SIGN = -1.0f;
const float CURSOR_Y_SIGN =  1.0f;
```

### Scrolling direction is reversed

Adjust this constant:

```cpp
const float SCROLL_SIGN = 1.0f;
```

## License

MIT License. See [`LICENSE`](LICENSE).
