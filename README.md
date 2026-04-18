# Thrust Calc
HX711-based load cell thrust measurement system for rocket motor testing.

This project consists of two main parts:
- Arduino firmware for data acquisition
- Python scripts for data logging, processing, and visualization

---

## Project Structure
thrust_calc/
├─ arduino/
│ ├─ calibration/
│ │ └─ calibration.ino
│ └─ thrust_logger/
│ └─ thrust_logger.ino
│
├─ python/
│ ├─ main.py
│ ├─ serial_reader.py
│ ├─ process_data.py
│ ├─ plot_data.py
│ └─ utils.py
│
├─ data/
│ ├─ raw/
│ ├─ processed/
│ ├─ plots/
│ └─ exports/
│
├─ docs/
└─ README.md

## Requirements
- Arduino IDE
- Python 3.10+
- HX711_ADC library

## Python dependencies
Install using:

```bash
pip install -r requirements.txt
```

# 1. Calibration
Upload:
```bash
arduino/calibration/calibration.ino
```

Steps:
1. Open Serial Monitor (57600 baud)
2. Send t to tare the load cell (no load)
3. Place a known weight
4. Enter the weight value (e.g. 500.0)
5. Press y to save calibration value to EEPROM

# 2. Measurement Mode
Upload:
```bash
arduino/thrust_logger/thrust_logger.ino
```

This firmware reads calibration value from EEPROM and continuously outputs data in CSV format as time_ms,mass_g
Example:
```bash
1234,512.3
1245,514.1
```

# Python Usage
Before running Python:

1. Close Arduino Serial Monitor
2. Ensure correct COM port is set in main.py
3. Make sure no other program is using the serial port
4. Run Data Collection
```bash
    python python/main.py
```

# Behavior
-> Script starts collecting data immediately
-> Runs continuously
-> Stop manually using:
```bash
Ctrl + C
```

# Output Files
After stopping the script, the following files are generated:

Raw data
```bash
data/raw/<timestamp>_raw.csv
```

Processed data
```bash
data/processed/<timestamp>_processed.csv
```

Plot
```bash
data/plots/<timestamp>_thrust.png
```


# Calculated Metrics
Max thrust (N)
Burn time (s)
Total impulse (N·s)

# Workflow Summary
1. Calibrate sensor (once)
2. Upload thrust_logger firmware
3. Run Python script
4. Perform test
5. Stop script manually
6. Analyze results

# Notes
Ensure load cell is unloaded during startup if auto-tare is enabled
Data accuracy depends on mechanical stability
HX711 sampling rate is limited (~80 Hz max)
Noise may be present; filtering can be added later
