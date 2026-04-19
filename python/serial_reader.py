import time
from pathlib import Path

import pandas as pd
import serial


def collect_serial_data(
    pPort: str,
    pBaudrate: int,
    pRawCsvPath: Path,
    pWarmupDurationS: float = 5.0,
) -> pd.DataFrame:
    try:
        tSerial = serial.Serial(pPort, pBaudrate, timeout=1)
    except serial.SerialException as pException:
        print(f"Could not open serial port {pPort}.")
        print("Make sure Arduino Serial Monitor / Serial Plotter is closed.")
        print("Also check that the selected COM port is correct.")
        raise pException

    time.sleep(2)

    tRows = []
    tStartWallTime = time.time()

    print("Collecting data...")
    print(f"Warming up for the first {pWarmupDurationS:.1f} seconds. Data will not be recorded.")
    print("Press Ctrl+C to stop.")

    try:
        while True:
            tLine = tSerial.readline().decode(errors="ignore").strip()

            if not tLine:
                continue

            if "," not in tLine:
                print(f"[INFO] Skipped: {tLine}")
                continue

            tParts = tLine.split(",")
            if len(tParts) != 2:
                print(f"[INFO] Invalid line: {tLine}")
                continue

            try:
                tTimeMs = float(tParts[0])
                tMassG = float(tParts[1])
            except ValueError:
                print(f"[INFO] Could not parse line: {tLine}")
                continue

            tElapsedWallTimeS = time.time() - tStartWallTime

            if tElapsedWallTimeS < pWarmupDurationS:
                continue

            tRow = {
                "time_ms": tTimeMs,
                "mass_g": tMassG,
            }
            tRows.append(tRow)

            print(f"{tTimeMs:8.0f} ms | {tMassG:10.3f} g")

    except KeyboardInterrupt:
        print("\nData collection stopped by user.")

    finally:
        tSerial.close()

    tDf = pd.DataFrame(tRows)
    tDf.to_csv(pRawCsvPath, index=False)

    return tDf