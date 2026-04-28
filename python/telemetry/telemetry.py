import csv
import time
from pathlib import Path
from datetime import datetime

import serial
import matplotlib.pyplot as plt


SERIAL_PORT = "COM5"
BAUD_RATE = 9600
DATA_TIMEOUT_SECONDS = 3.0

BASE_DIR = Path(__file__).resolve().parents[2]

PLOTS_DIR = BASE_DIR / "data" / "plots"
TELEMETRY_DIR = BASE_DIR / "data" / "telemetry"

PLOTS_DIR.mkdir(parents=True, exist_ok=True)
TELEMETRY_DIR.mkdir(parents=True, exist_ok=True)


def make_output_name():
    return datetime.now().strftime("%Y-%m-%d_%H%M") + "_telemetry"


def parse_packet(pLine):
    tIndex = pLine.find("D,")

    if tIndex == -1:
        return None

    tLine = pLine[tIndex:]
    tParts = tLine.split(",")

    if len(tParts) != 10:
        return None

    try:
        return [
            float(tParts[1]),
            float(tParts[2]),
            float(tParts[3]),
            float(tParts[4]),
            float(tParts[5]),
            float(tParts[6]),
            float(tParts[7]),
            float(tParts[8]),
            float(tParts[9]),
        ]
    except ValueError:
        return None


def save_figure(pFigure, pBaseName):
    tPath = PLOTS_DIR / f"{pBaseName}.png"
    pFigure.savefig(tPath, dpi=200, bbox_inches="tight")
    print(f"Saved plot: {tPath}")


def save_csv(
    pBaseName,
    pTime,
    pAx,
    pAy,
    pAz,
    pGx,
    pGy,
    pGz,
    pTemp,
    pPressure,
    pAltitude,
):
    tPath = TELEMETRY_DIR / f"{pBaseName}.csv"

    with open(tPath, "w", newline="", encoding="utf-8") as tFile:
        tWriter = csv.writer(tFile)

        tWriter.writerow([
            "time_s",
            "ax",
            "ay",
            "az",
            "gx",
            "gy",
            "gz",
            "temperature_c",
            "pressure_hpa",
            "altitude_m",
        ])

        for i in range(len(pTime)):
            tWriter.writerow([
                pTime[i],
                pAx[i],
                pAy[i],
                pAz[i],
                pGx[i],
                pGy[i],
                pGz[i],
                pTemp[i],
                pPressure[i],
                pAltitude[i],
            ])

    print(f"Saved CSV: {tPath}")


def save_outputs(
    pFigure,
    pTime,
    pAx,
    pAy,
    pAz,
    pGx,
    pGy,
    pGz,
    pTemp,
    pPressure,
    pAltitude,
):
    if len(pTime) == 0:
        print("No telemetry data received. Nothing saved.")
        return

    tBaseName = make_output_name()

    save_figure(pFigure, tBaseName)

    save_csv(
        tBaseName,
        pTime,
        pAx,
        pAy,
        pAz,
        pGx,
        pGy,
        pGz,
        pTemp,
        pPressure,
        pAltitude,
    )


def main():
    tSerial = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0)

    tRecording = False
    tStartTime = None
    tLastDataTime = None

    tTime = []

    tAx, tAy, tAz = [], [], []
    tGx, tGy, tGz = [], [], []
    tTemp = []
    tPressure = []
    tAltitude = []

    plt.ion()

    tFigure, tAxes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    tFigure.suptitle("Live LoRa Telemetry")

    tLineAx, = tAxes[0].plot([], [], label="AX")
    tLineAy, = tAxes[0].plot([], [], label="AY")
    tLineAz, = tAxes[0].plot([], [], label="AZ")

    tLineGx, = tAxes[1].plot([], [], label="GX")
    tLineGy, = tAxes[1].plot([], [], label="GY")
    tLineGz, = tAxes[1].plot([], [], label="GZ")

    tLineTemp, = tAxes[2].plot([], [], label="Temperature C")

    tLinePressure, = tAxes[3].plot([], [], label="Pressure hPa")
    tLineAltitude, = tAxes[3].plot([], [], label="Altitude m")

    tAxes[0].set_ylabel("Accel Raw")
    tAxes[1].set_ylabel("Gyro Raw")
    tAxes[2].set_ylabel("Temp C")
    tAxes[3].set_ylabel("Pressure / Altitude")
    tAxes[3].set_xlabel("Time (s)")

    for tAxis in tAxes:
        tAxis.grid(True)
        tAxis.legend()

    plt.show(block=False)
    plt.pause(0.5)

    print("Waiting for telemetry...")
    print(f"Plot output directory: {PLOTS_DIR}")
    print(f"CSV output directory: {TELEMETRY_DIR}")

    while plt.fignum_exists(tFigure.number):
        if tRecording and tLastDataTime is not None:
            if time.time() - tLastDataTime > DATA_TIMEOUT_SECONDS:
                print("No new telemetry received. Saving outputs and exiting...")
                save_outputs(
                    tFigure,
                    tTime,
                    tAx,
                    tAy,
                    tAz,
                    tGx,
                    tGy,
                    tGz,
                    tTemp,
                    tPressure,
                    tAltitude,
                )
                break

        tRawBytes = tSerial.readline()

        if not tRawBytes:
            plt.pause(0.02)
            continue

        tLine = tRawBytes.decode(errors="ignore").strip()

        if not tLine:
            plt.pause(0.02)
            continue

        print(tLine)

        if tLine == "S":
            tRecording = True
            tStartTime = time.time()
            tLastDataTime = time.time()
            print("Recording started.")
            continue

        if tLine == "T":
            print("STOP packet received. Saving outputs and exiting...")
            save_outputs(
                tFigure,
                tTime,
                tAx,
                tAy,
                tAz,
                tGx,
                tGy,
                tGz,
                tTemp,
                tPressure,
                tAltitude,
            )
            break

        tData = parse_packet(tLine)

        if tData is None:
            plt.pause(0.02)
            continue

        if not tRecording:
            tRecording = True
            tStartTime = time.time()
            print("Recording auto-started from first data packet.")

        tLastDataTime = time.time()
        tNow = tLastDataTime - tStartTime

        tTime.append(tNow)

        tAx.append(tData[0])
        tAy.append(tData[1])
        tAz.append(tData[2])

        tGx.append(tData[3])
        tGy.append(tData[4])
        tGz.append(tData[5])

        tTemp.append(tData[6])
        tPressure.append(tData[7])
        tAltitude.append(tData[8])

        tLineAx.set_data(tTime, tAx)
        tLineAy.set_data(tTime, tAy)
        tLineAz.set_data(tTime, tAz)

        tLineGx.set_data(tTime, tGx)
        tLineGy.set_data(tTime, tGy)
        tLineGz.set_data(tTime, tGz)

        tLineTemp.set_data(tTime, tTemp)

        tLinePressure.set_data(tTime, tPressure)
        tLineAltitude.set_data(tTime, tAltitude)

        for tAxis in tAxes:
            tAxis.relim()
            tAxis.autoscale_view()

        tFigure.canvas.draw()
        tFigure.canvas.flush_events()
        plt.pause(0.02)

    tSerial.close()
    plt.ioff()
    print("Program finished.")


if __name__ == "__main__":
    main()