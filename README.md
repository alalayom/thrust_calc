# Thrust Calc

Rocket motor test utilities for both static thrust stand measurements and LoRa-based telemetry logging.

The project is split into two main subsystems:
- Test stand: HX711 load cell data acquisition, thrust processing, and plotting
- Telemetry: SX1278 LoRa receiver/transmitter firmware plus Python live logging and plotting

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
|   |-- test_stand/
|   |   |-- main.py
|   |   |-- serial_reader.py
|   |   |-- process_data.py
|   |   |-- plot_data.py
|   |   `-- utils.py
|   `-- telemetry/
|       `-- telemetry.py
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

- Arduino IDE
- Python 3.10+
- HX711_ADC Arduino library
- LoRa Arduino library
- Adafruit BMP280 Arduino library
- Python dependencies from `requirements.txt`

Install Python dependencies:

```bash
pip install -r requirements.txt
```

---

## Test Stand

The test stand subsystem measures load cell data with HX711, converts mass readings to thrust, and produces raw CSV, processed CSV, and thrust plots.

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

### 2. Measurement Mode

Upload:

```text
arduino/test_stand/thrust_logger/thrust_logger.ino
```

This firmware reads the saved calibration value from EEPROM and continuously outputs CSV data:

```text
time_ms,mass_g
```

Example:

```text
1234,512.300
1245,514.100
```

### 3. Python Test Stand Logger

Before running Python:

1. Close Arduino Serial Monitor.
2. Check the COM port in `python/test_stand/main.py`.
3. Make sure no other program is using the serial port.
4. Run:

```bash
python python/test_stand/main.py
```

The script collects serial data, processes it, saves output files, and prints a test summary.

### Test Stand Output Files

```text
data/raw/<timestamp>_raw.csv
data/processed/<timestamp>_processed.csv
data/plots/<timestamp>_thrust.png
```

### Calculated Test Stand Metrics

- Max thrust in N
- Burn time in s
- Total impulse in N.s

---

## Telemetry

The telemetry subsystem uses a LoRa transmitter/receiver pair with a single unified Python script that performs real-time simulation and data logging.

### 1. Telemetry Transmitter

Upload to the sensor/transmitter Arduino or ESP32:

```text
arduino/telemetry/Transmitter/Transmitter.ino
```

Sensors and modules used:

- MPU6050 for raw accelerometer and gyroscope values
- BMP280 for temperature, pressure, and estimated altitude
- SX1278 LoRa module at `433E6`
- Button on `BUTTON_PIN` to start and stop telemetry streaming

The transmitter sends:

- `S` when telemetry starts
- `T` when telemetry stops
- `D,AX,AY,AZ,GX,GY,GZ,BT,P,ALT` for telemetry data packets

```text
D,-3244,64,14448,-55,165,-175,24.00,881.53,1159.32
```

Telemetry packets are sent every `100 ms` while streaming is enabled.

### 2. Telemetry Receiver

Upload to the ground station/receiver Arduino:

```text
arduino/telemetry/Receiver/Receiver.ino
```

The receiver listens for LoRa packets, prints packet metadata to serial, and forwards telemetry values for the Python script.

Serial settings:

```text
115200 baud
```

### 3. Python Telemetry Simulation & Logger

Before running Python:

1. Close Arduino Serial Monitor.
2. Check the COM port in `python/telemetry/telemetry_simulation.py`.
3. Make sure the receiver Arduino is connected to the computer.
4. Run:

```bash
python python/telemetry/telemetry_simulation.py
```

The telemetry script:

- Waits for telemetry packets
- Starts recording on `S` or the first valid data packet
- Performs gyro calibration automatically at startup
- Runs real-time 3D rocket simulation
- Records all telemetry data in memory

### Telemetry Output Files

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

### Test Stand

1. Calibrate the load cell once.
2. Upload `thrust_logger.ino`.
3. Run `python/test_stand/main.py`.
4. Perform the static fire test.
5. Stop the script manually.
6. Review generated CSV files, plot, and metrics.

### Telemetry

1. Upload `Transmitter.ino` to the telemetry sensor node.
2. Upload `Receiver.ino` to the ground station node.
3. Run `python/telemetry/telemetry_simulation.py`.
4. Start telemetry with the transmitter button.
5. Stop telemetry with the same button or wait for timeout.
6. Review generated telemetry CSV and plot.

## Notes

- Keep the load cell unloaded during startup if auto-tare is enabled.
- Test stand accuracy depends on mechanical stability and calibration quality.
- HX711 sampling rate is limited by the module and selected configuration.
- Telemetry quality depends on LoRa antenna placement, range, and packet loss.
- BMP280 altitude uses a reference sea-level pressure value, so altitude is an estimate.
