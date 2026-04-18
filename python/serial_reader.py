import time
from pathlib import Path

import pandas as pd
import serial


def collect_serial_data(
    pPort: str,
    pBaudrate: int,
    pRawCsvPath: Path,
) -> pd.DataFrame:
    tSerial = serial.Serial(pPort, pBaudrate, timeout=1)
    time.sleep(2)

    tRows = []

    print("Collecting data... Press Ctrl+C to stop.")

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