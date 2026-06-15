# Thrust Calc

Rocket motor ground software and Esp firmware for static thrust testing, LoRa telemetry, GPS tracking, live plotting, rocket attitude visualization, and serial ignition control.

The current desktop application is the main control panel under `python/rocket_panel/`:

```bash
python python/rocket_panel/main.py
```

The panel supports two data modes:

- `staticTest`: reads the thrust stand/load cell stream and plots thrust.
- `Telemetry`: reads the LoRa receiver stream, plots flight telemetry, draws the GPS route on an OpenStreetMap background, and updates the rocket attitude simulation.

---

## Project Structure

```text
thrust_calc/
|-- arduino/
|   |-- test_stand/
|   |   |-- calibration/
|   |   |   `-- calibration.ino
|   |   `-- thrust_logger/
|   |       `-- thrust_logger.ino
|   `-- telemetry/
|       |-- Receiver/
|       |   `-- Receiver.ino
|       `-- Transmitter/
|           `-- Transmitter.ino
|
|-- python/
|   `-- rocket_panel/
|       |-- main.py
|       |-- app.py
|       |-- config/
|       |-- modes/
|       |-- parsers/
|       |-- plotting/
|       `-- serial_io/
|
|-- data/
|   |-- raw/
|   |-- processed/
|   |-- plots/
|   `-- telemetry/
|
|-- docs/
|-- requirements.txt
|-- LICENSE
`-- README.md
```

`main.py` starts the panel. Most of the current panel behavior is implemented in `app.py`.

---

## Requirements

### Python

- Python 3.10+
- Tk/Tkinter support
- Internet connection for OpenStreetMap tiles in telemetry mode
- Packages from `requirements.txt`

Install dependencies:

```bash
pip install -r requirements.txt
```

Current Python-side core libraries:

- `pyserial` for COM port communication
- `matplotlib` for live graphs, GPS map rendering, and 3D rocket visualization
- `tkinter` from the Python installation for the desktop UI

### Arduino

- Arduino IDE
- ESP32 board support for the ESP32-S3 transmitter
- HX711_ADC Arduino library
- LoRa Arduino library
- Adafruit BMP3XX Arduino library
- TinyGPSPlus Arduino library

---

## Control Panel

Run:

```bash
python python/rocket_panel/main.py
```

The application opens a maximized Tkinter window. Press `F11` for full-screen mode and `Escape` to exit full-screen mode.

### Top Bar

- `Data Port`: incoming serial data port. Use the thrust stand port in `staticTest`, or the LoRa receiver port in `Telemetry`.
- `Data Baud`: baud rate for the selected data port.
- `Ignition Port`: outgoing serial port for the ignition/fuse controller.
- `Ignition Baud`: baud rate for the ignition controller.
- `Refresh`: reloads available COM ports.

The data port and ignition port should normally be different devices. If the same COM port is already open in Arduino Serial Monitor, another Python process, or another tool, the panel cannot open it.

### Main Layout

- Left side: live graphs.
- Right side in `Telemetry`: GPS map on top, rocket simulation below.
- Bottom live data strip: current numeric values.
- Bottom control panel: mode selection, `Start`, `Stop`, `Save`, and `fuse`.

### Modes

#### staticTest

Reads thrust stand data:

```text
time_ms,mass_g
```

The panel:

- waits through a startup warmup period,
- builds a stable baseline from the load cell,
- subtracts the baseline from incoming mass readings,
- converts corrected mass to thrust in Newtons,
- plots live thrust,
- calculates max thrust, burn time, and total impulse.

#### Telemetry

Reads LoRa receiver output. The parser accepts both receiver log lines and direct packet lines:

```text
Raw packet: D,AX,AY,AZ,GX,GY,GZ,BT,P,ALT,LAT,LON,GSAT,GALT
D,AX,AY,AZ,GX,GY,GZ,BT,P,ALT,LAT,LON,GSAT,GALT
AX,AY,AZ,GX,GY,GZ,BT,P,ALT,LAT,LON,GSAT,GALT
```

The panel:

- plots raw accelerometer and gyroscope values,
- plots BMP388 temperature, pressure, and pressure-based altitude,
- plots GPS altitude in the same altitude graph,
- shows latitude, longitude, GPS satellite count, and GPS connection state,
- draws the GPS route on an OpenStreetMap background,
- updates the rocket attitude simulation from accelerometer/gyroscope data.

GPS values of `999`, `-999`, or missing values are treated as not connected. When GPS becomes valid, the status changes to `satellite connected`.

### GPS Map Behavior

Telemetry mode uses OpenStreetMap tiles.

- Default map zoom is `13`.
- Mouse wheel changes zoom one integer OSM level at a time.
- The map center follows the latest valid GPS position.
- To prevent jitter, the map center only moves when the latest GPS position is at least about `50 m` away from the current map center.
- The header shows the current map center and zoom level.

### Fuse / Ignition

The `fuse` button sends serial commands over the selected ignition port:

```text
fuse ON  -> FIRE
fuse OFF -> SAFE
```

The ignition controller must be programmed to receive these commands and handle the physical ignition circuit safely.

---

## Static Test Firmware

### 1. Load Cell Calibration

Upload:

```text
arduino/test_stand/calibration/calibration.ino
```

Basic calibration flow:

1. Open Serial Monitor at `57600` baud.
2. Send `t` to tare the unloaded load cell.
3. Place a known weight on the load cell.
4. Enter the known weight value, for example `500.0`.
5. Press `y` to save the calibration value to EEPROM.

### 2. Thrust Logger

Upload:

```text
arduino/test_stand/thrust_logger/thrust_logger.ino
```

The logger reads the saved calibration value from EEPROM and outputs:

```text
time_ms,mass_g
```

Current serial baud:

```text
57600
```

Panel settings:

```text
Mode      : staticTest
Data Baud : 57600
```

Saved outputs:

```text
data/raw/<timestamp>_raw.csv
data/processed/<timestamp>_processed.csv
data/plots/<timestamp>_thrust.png
```

---

## Telemetry Firmware

### 1. Transmitter

Upload to the ESP32-S3 telemetry node:

```text
arduino/telemetry/Transmitter/Transmitter.ino
```

Hardware used:

- ESP32-S3
- MPU6500 accelerometer/gyroscope over I2C
- BMP388 pressure sensor over I2C
- GPS module over UART with TinyGPSPlus
- SX1278 LoRa module at `433E6`
- Push button on `BUTTON_PIN` to start/stop telemetry streaming

Important transmitter pins:

```text
LoRa SCK   : 12
LoRa MISO  : 13
LoRa MOSI  : 11
LoRa SS    : 10
LoRa RST   : 9
LoRa DIO0  : 14

I2C SDA    : 4
I2C SCL    : 5

GPS RX     : 18
GPS TX     : 17
GPS baud   : 9600

Button     : 3
```

The transmitter sends:

```text
S
T
D,AX,AY,AZ,GX,GY,GZ,BT,P,ALT,LAT,LON,GSAT,GALT
```

Packet fields:

```text
D     packet marker
AX    raw accelerometer X
AY    raw accelerometer Y
AZ    raw accelerometer Z
GX    raw gyroscope X
GY    raw gyroscope Y
GZ    raw gyroscope Z
BT    BMP388 temperature in Celsius
P     BMP388 pressure in hPa
ALT   BMP388 pressure-based altitude in meters
LAT   GPS latitude
LON   GPS longitude
GSAT  GPS satellite count
GALT  GPS altitude in meters
```

When GPS has no valid fix, the transmitter sends sentinel values such as `-999.000000`, `-1`, or `-999.00`. The Python panel treats those GPS values as disconnected.

Example packet:

```text
D,-690,68,7766,-151,133,-10,27.59,891.73,1064.71,39.822400,32.659300,12,1078.80
```

Telemetry packets are sent every `100 ms` while streaming is enabled. The transmitter skips the first few BMP388 readings after startup so the initial unstable pressure/altitude reading is not sent.

### 2. Receiver

Upload to the LoRa ground station:

```text
arduino/telemetry/Receiver/Receiver.ino
```

The receiver:

- listens for LoRa packets at `433E6`,
- prints packet metadata such as packet size, RSSI, and SNR,
- prints `Raw packet: ...`,
- forwards data packets to the panel as CSV without the leading `D,`.

Current receiver serial baud:

```text
115200
```

Panel settings for current receiver firmware:

```text
Mode      : Telemetry
Data Baud : 115200
```

Saved outputs:

```text
data/telemetry/<timestamp>_telemetry.csv
data/plots/<timestamp>_telemetry.png
```

Telemetry CSV columns:

```text
time_s,ax,ay,az,gx,gy,gz,temperature_c,pressure_hpa,altitude_m,latitude,longitude,gps_satellites,gps_altitude_m,gps_connected
```

---

## Workflow Summary

### Static Test

1. Calibrate the load cell if needed.
2. Upload `arduino/test_stand/thrust_logger/thrust_logger.ino`.
3. Run `python python/rocket_panel/main.py`.
4. Select `staticTest`.
5. Select the thrust stand data port.
6. Set `Data Baud` to `57600`.
7. Press `Start`.
8. Press `Stop` when finished.
9. Press `Save` to write CSV and plot outputs.

### Telemetry

1. Upload `arduino/telemetry/Transmitter/Transmitter.ino` to the ESP32-S3 telemetry node.
2. Upload `arduino/telemetry/Receiver/Receiver.ino` to the LoRa receiver node.
3. Run `python python/rocket_panel/main.py`.
4. Select `Telemetry`.
5. Select the receiver data port.
6. Set `Data Baud` to `115200` for the current receiver firmware.
7. Press `Start`.
8. Use the transmitter button to start or stop telemetry streaming.
9. Press `Save` to write telemetry CSV and plot outputs.

### Ignition

1. Connect the ignition controller as a separate serial device.
2. Select it as `Ignition Port`.
3. Set `Ignition Baud` to match the ignition controller firmware.
4. Keep `fuse OFF` until the physical safety setup is ready.
5. Use `fuse ON` only when the ignition circuit is intentionally armed and ready.

---

## Notes

- Close Arduino Serial Monitor and Serial Plotter before opening the same COM port in the panel.
- If telemetry does not appear, check that `Data Baud` is `115200` for the current `Receiver.ino`.
- Keep the load cell unloaded during static test startup so the baseline can settle correctly.
- Static thrust accuracy depends on calibration quality and mechanical stability.
- Telemetry quality depends on LoRa antenna placement, distance, packet loss, and receiver baud selection.
- BMP388 altitude is pressure-based and depends on the sea-level pressure reference in firmware.
- GPS altitude and BMP388 altitude are plotted separately because they come from different measurement sources.
- OpenStreetMap tiles require internet access; if tile download fails, the map can fall back to a plain grid.
