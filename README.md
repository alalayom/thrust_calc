# Thrust Calc

Rocket motor ground software and Arduino firmware for static thrust testing, LoRa telemetry, live plotting, rocket attitude visualization, and ignition control.

The current Python side is centered around a single control panel:

```bash
python python/rocket_panel/main.py
```

The panel opens a full-screen desktop window with:
- a top port selection bar
- a left live graph area
- a right live data and rocket simulation area
- a bottom control panel for `staticTest`, `Telemetry`, `Start`, `Stop`, `Save`, and `fuse`

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
|   |-- telemetry/
|   `-- exports/
|
|-- docs/
|-- requirements.txt
|-- LICENSE
`-- README.md
```

## Requirements

### ESP32

- Arduino IDE
- ESP32 board support for the ESP32-S3 telemetry transmitter
- HX711_ADC Arduino library
- LoRa Arduino library
- Adafruit BMP3XX Arduino library
- TinyGPSPlus Arduino library

### Python

- Python 3.10+
- Python with Tk/Tkinter support
- Dependencies from `requirements.txt`

Install Python dependencies:

```bash
pip install -r requirements.txt
```

---

## Control Panel

Run:

```bash
python python/rocket_panel/main.py
```

### Top Bar

- `Data Port`: incoming serial data port for either static test or telemetry
- `Data Baud`: baud rate for the selected data port
- `Ignition Port`: outgoing serial port for ignition/fuse commands
- `Ignition Baud`: baud rate for the ignition controller
- `Refresh`: refreshes available COM ports

The data port and ignition port should be different devices.

### Main View

- Left side: live graph
- Right side: live values and rocket simulation
- Bottom panel: mode selection and control buttons

### Modes

`staticTest`

- Reads thrust stand serial data in this format:

```text
time_ms,mass_g
```

- Applies startup warmup and baseline filtering to avoid HX711 startup spikes.
- Converts corrected mass to thrust in Newtons.
- Shows live thrust graph and metrics.

`Telemetry`

- Reads LoRa receiver serial output.
- Accepts receiver lines such as:

```text
Raw packet: D,AX,AY,AZ,GX,GY,GZ,BT,P,ALT,LAT,LON,GSAT,GALT
```

- Also accepts direct telemetry packets:

```text
D,AX,AY,AZ,GX,GY,GZ,BT,P,ALT,LAT,LON,GSAT,GALT
```

- Updates live telemetry graphs.
- Runs the built-in Matplotlib 3D rocket attitude simulation.
- Performs gyro startup calibration before showing live attitude changes.

### Fuse

The `fuse` button sends commands over the selected ignition port:

```text
fuse ON  -> FIRE
fuse OFF -> SAFE
```

The ignition Arduino/controller must be programmed to receive those serial commands and handle the actual ignition circuit safely.

---

## Static Test Firmware

### 1. Calibration

Upload:

```text
arduino/test_stand/calibration/calibration.ino
```

Steps:

1. Open Serial Monitor at `57600` baud.
2. Send `t` to tare the unloaded load cell.
3. Place a known weight on the load cell.
4. Enter the known weight value, for example `500.0`.
5. Press `y` to save the calibration value to EEPROM.

### 2. Measurement

Upload:

```text
arduino/test_stand/thrust_logger/thrust_logger.ino
```

The firmware reads the saved calibration value from EEPROM and continuously outputs:

```text
time_ms,mass_g
```

Default panel settings:

```text
Mode      : staticTest
Data Baud : 57600
```

Panel outputs:

```text
data/raw/<timestamp>_raw.csv
data/processed/<timestamp>_processed.csv
data/plots/<timestamp>_thrust.png
```

Calculated metrics:

- Max thrust in N
- Burn time in s
- Total impulse in N.s

---

## Telemetry Firmware

### 1. Transmitter

Upload to the ESP32-S3 sensor/transmitter board:

```text
arduino/telemetry/Transmitter/Transmitter.ino
```

Hardware used:

- MPU6500 for raw accelerometer and gyroscope values
- BMP388 for temperature, pressure, and estimated altitude
- SX1278 LoRa module at `433E6`
- Button on `BUTTON_PIN` to start and stop telemetry streaming
- NE0-M8N GPS for latitude, altitude and altitude values

The transmitter sends:

```text
S
T
D,AX,AY,AZ,GX,GY,GZ,BT,P,ALT,LAT,LON,GSAT,GALT
```

Example packet:

```text
D,-690,68,7766,-151,133,-10,27.59,891.73,1064.71,40.123456,41.123456,10,1080.60
```

Telemetry packets are sent every `100 ms` while streaming is enabled. After reset, the transmitter skips the first few BMP388 readings so the first unstable altitude value is not sent.

### 2. Receiver

Upload to the ground station/receiver Arduino:

```text
arduino/telemetry/Receiver/Receiver.ino
```

The receiver listens for LoRa packets, prints packet metadata, and forwards raw packets to the control panel.

Current receiver serial baud:

```text
9600
```

Default panel settings:

```text
Mode      : Telemetry
Data Baud : 115200
```

Panel outputs:

```text
data/telemetry/<timestamp>_telemetry.csv
data/plots/<timestamp>_telemetry.png
```

Telemetry CSV columns:

```text
time_s,ax,ay,az,gx,gy,gz,temperature_c,pressure_hpa,altitude_m
```

---

## Workflow Summary

### Static Test

1. Calibrate the load cell once.
2. Upload `thrust_logger.ino`.
3. Run `python/rocket_panel/main.py`.
4. Select `staticTest`.
5. Select the thrust stand data port and `57600` baud.
6. Press `Start`.
7. Press `Stop` or `Save` after the test.

### Telemetry

1. Upload `Transmitter.ino` to the telemetry sensor node.
2. Upload `Receiver.ino` to the ground station node.
3. Run `python/rocket_panel/main.py`.
4. Select `Telemetry`.
5. Select the receiver data port and `9600` baud.
6. Press `Start`.
7. Start/stop telemetry with the transmitter button.

### Ignition

1. Connect the ignition controller as a separate serial device.
2. Select it as `Ignition Port`.
3. Keep the ignition controller firmware matched to the panel commands: `FIRE` and `SAFE`.
4. Use `fuse OFF/ON` only when the physical safety setup is ready.

## Notes

- Keep Arduino Serial Monitor and Serial Plotter closed before opening the same COM port in the panel.
- Keep the load cell unloaded during static test startup so baseline filtering can settle correctly.
- Test stand accuracy depends on mechanical stability and calibration quality.
- Telemetry quality depends on LoRa antenna placement, range, and packet loss.
- BMP388 altitude uses a reference sea-level pressure value, so altitude is an estimate.
- Wait for the GPS to acquire a satellite fix.
