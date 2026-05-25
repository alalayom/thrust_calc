import csv
import math
import time
from pathlib import Path
from datetime import datetime

import serial
import matplotlib.pyplot as plt

from vpython import (
    canvas,
    cylinder,
    cone,
    box,
    vector,
    rate,
    color,
    arrow,
    label,
)

SERIAL_PORT = "COM9"
BAUD_RATE = 115200
DATA_TIMEOUT_SECONDS = 4.0

BASE_DIR = Path(__file__).resolve().parents[2]

PLOTS_DIR = BASE_DIR / "data" / "plots"
TELEMETRY_DIR = BASE_DIR / "data" / "telemetry"

PLOTS_DIR.mkdir(parents=True, exist_ok=True)
TELEMETRY_DIR.mkdir(parents=True, exist_ok=True)

gAlpha = 0.97
gGyroDeadband = 0.012

gGyroSensitivity = 131.0  # MPU6050 default +-250 deg/s için

gBodyLength = 4.0
gNoseLength = 0.9
gRocketRadius = 0.35

gSideTiltSign = -1.0
gForwardTiltSign = 1.0
gSpinSign = 1.0


def make_output_name():
    return datetime.now().strftime("%Y-%m-%d_%H%M") + "_telemetry"


def extract_raw_packet(pLine):
    if not pLine.startswith("Raw packet:"):
        return None

    return pLine.replace("Raw packet:", "").strip()


def parse_packet(pPacket):
    if not pPacket.startswith("D,"):
        return None

    tParts = pPacket.split(",")

    if len(tParts) != 10:
        return None

    try:
        return [float(tPart) for tPart in tParts[1:]]
    except ValueError:
        return None


def raw_gyro_to_rad_s(pRawValue):
    return math.radians(pRawValue / gGyroSensitivity)


def rotate_around_axis(pVector, pAxis, pAngle):
    tAxis = pAxis.norm()
    tCos = math.cos(pAngle)
    tSin = math.sin(pAngle)

    return (
        pVector * tCos
        + tAxis.cross(pVector) * tSin
        + tAxis * (tAxis.dot(pVector)) * (1.0 - tCos)
    )


def rotate_tilt_only(pVector, pSideTilt, pForwardTilt):
    tRotated = rotate_around_axis(pVector, vector(0, 0, 1), pSideTilt)
    tRotated = rotate_around_axis(tRotated, vector(1, 0, 0), pForwardTilt)
    return tRotated


def rotate_rocket_vector(pVector, pSideTilt, pForwardTilt, pSpin):
    tRotated = rotate_tilt_only(pVector, pSideTilt, pForwardTilt)
    tBodyAxis = rotate_tilt_only(vector(0, 1, 0), pSideTilt, pForwardTilt)
    return rotate_around_axis(tRotated, tBodyAxis, pSpin)


def apply_deadband(pValue, pDeadband):
    if abs(pValue) < pDeadband:
        return 0.0

    return pValue


def update_part(pPart, pBasePos, pBaseAxis, pBaseUp, pSideTilt, pForwardTilt, pSpin):
    pPart.pos = rotate_rocket_vector(pBasePos, pSideTilt, pForwardTilt, pSpin)
    pPart.axis = rotate_rocket_vector(pBaseAxis, pSideTilt, pForwardTilt, pSpin)
    pPart.up = rotate_rocket_vector(pBaseUp, pSideTilt, pForwardTilt, pSpin)


def save_figure(pFigure, pBaseName):
    tPath = PLOTS_DIR / f"{pBaseName}.png"
    pFigure.savefig(tPath, dpi=200, bbox_inches="tight")
    print(f"Saved plot: {tPath}")


def save_csv(pBaseName, pTime, pAx, pAy, pAz, pGx, pGy, pGz, pTemp, pPressure, pAltitude):
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


def save_outputs(pFigure, pTime, pAx, pAy, pAz, pGx, pGy, pGz, pTemp, pPressure, pAltitude):
    if len(pTime) == 0:
        print("No telemetry data received. Nothing saved.")
        return

    tBaseName = make_output_name()

    if pFigure is None:
        tFigure, tAxes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
        tFigure.suptitle("LoRa Telemetry")

        tAxes[0].plot(pTime, pAx, label="AX")
        tAxes[0].plot(pTime, pAy, label="AY")
        tAxes[0].plot(pTime, pAz, label="AZ")

        tAxes[1].plot(pTime, pGx, label="GX")
        tAxes[1].plot(pTime, pGy, label="GY")
        tAxes[1].plot(pTime, pGz, label="GZ")

        tAxes[2].plot(pTime, pTemp, label="Temperature C")

        tAxes[3].plot(pTime, pPressure, label="Pressure hPa")
        tAxes[3].plot(pTime, pAltitude, label="Altitude m")

        tAxes[0].set_ylabel("Accel Raw")
        tAxes[1].set_ylabel("Gyro Raw")
        tAxes[2].set_ylabel("Temp C")
        tAxes[3].set_ylabel("Pressure / Altitude")
        tAxes[3].set_xlabel("Time (s)")

        for tAxis in tAxes:
            tAxis.grid(True)
            tAxis.legend()

        pFigure = tFigure

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

def create_graph_window():
    plt.ion()

    tFigure, tAxes = plt.subplots(4, 1, figsize=(9, 8), sharex=True)
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

    return {
        "figure": tFigure,
        "axes": tAxes,
        "line_ax": tLineAx,
        "line_ay": tLineAy,
        "line_az": tLineAz,
        "line_gx": tLineGx,
        "line_gy": tLineGy,
        "line_gz": tLineGz,
        "line_temp": tLineTemp,
        "line_pressure": tLinePressure,
        "line_altitude": tLineAltitude,
    }


def create_rocket_scene():
    tScene = canvas(
        title="MPU6050 ESP32-S3 Rocket Simulation",
        width=900,
        height=750,
        background=color.black,
    )

    tScene.forward = vector(-1.0, -0.45, -1.0)
    tScene.center = vector(0, 0, 0)
    tScene.range = 4.8

    arrow(pos=vector(0, 0, 0), axis=vector(2.5, 0, 0), shaftwidth=0.02, color=color.red)
    arrow(pos=vector(0, 0, 0), axis=vector(0, 2.5, 0), shaftwidth=0.02, color=color.green)
    arrow(pos=vector(0, 0, 0), axis=vector(0, 0, 2.5), shaftwidth=0.02, color=color.blue)

    label(pos=vector(2.75, 0, 0), text="X", height=18, color=color.red, box=False)
    label(pos=vector(0, 2.75, 0), text="Y", height=18, color=color.green, box=False)
    label(pos=vector(0, 0, 2.75), text="Z", height=18, color=color.blue, box=False)

    tBodyBasePos = vector(0, -gBodyLength / 2.0, 0)
    tBodyBaseAxis = vector(0, gBodyLength, 0)
    tBodyBaseUp = vector(1, 0, 0)

    tBody = cylinder(
        pos=tBodyBasePos,
        axis=tBodyBaseAxis,
        radius=gRocketRadius,
        color=color.cyan,
    )

    tNoseBasePos = vector(0, gBodyLength / 2.0, 0)
    tNoseBaseAxis = vector(0, gNoseLength, 0)
    tNoseBaseUp = vector(1, 0, 0)

    tNose = cone(
        pos=tNoseBasePos,
        axis=tNoseBaseAxis,
        radius=gRocketRadius,
        color=color.orange,
    )

    tFinRadialSize = 0.34
    tFinLength = 0.52
    tFinThickness = 0.055
    tFinCenterY = -gBodyLength / 2.0 + 0.35
    tFinDistance = gRocketRadius + tFinRadialSize / 2.0
    tFinSize = vector(tFinRadialSize, tFinLength, tFinThickness)

    tFinXPositive = box(pos=vector(tFinDistance, tFinCenterY, 0), size=tFinSize, color=color.red)
    tFinXNegative = box(pos=vector(-tFinDistance, tFinCenterY, 0), size=tFinSize, color=color.red)
    tFinZPositive = box(pos=vector(0, tFinCenterY, tFinDistance), size=tFinSize, color=color.red)
    tFinZNegative = box(pos=vector(0, tFinCenterY, -tFinDistance), size=tFinSize, color=color.red)

    tText = label(
        pos=vector(0, -4.35, 0),
        text="Waiting for telemetry...",
        height=16,
        color=color.white,
        box=False,
    )

    return {
        "body": tBody,
        "body_base_pos": tBodyBasePos,
        "body_base_axis": tBodyBaseAxis,
        "body_base_up": tBodyBaseUp,
        "nose": tNose,
        "nose_base_pos": tNoseBasePos,
        "nose_base_axis": tNoseBaseAxis,
        "nose_base_up": tNoseBaseUp,
        "fin_x_positive": tFinXPositive,
        "fin_x_negative": tFinXNegative,
        "fin_z_positive": tFinZPositive,
        "fin_z_negative": tFinZNegative,
        "fin_radial_size": tFinRadialSize,
        "fin_center_y": tFinCenterY,
        "fin_distance": tFinDistance,
        "text": tText,
    }


def update_graph(pGraph, pTime, pAx, pAy, pAz, pGx, pGy, pGz, pTemp, pPressure, pAltitude):
    pGraph["line_ax"].set_data(pTime, pAx)
    pGraph["line_ay"].set_data(pTime, pAy)
    pGraph["line_az"].set_data(pTime, pAz)

    pGraph["line_gx"].set_data(pTime, pGx)
    pGraph["line_gy"].set_data(pTime, pGy)
    pGraph["line_gz"].set_data(pTime, pGz)

    pGraph["line_temp"].set_data(pTime, pTemp)

    pGraph["line_pressure"].set_data(pTime, pPressure)
    pGraph["line_altitude"].set_data(pTime, pAltitude)

    for tAxis in pGraph["axes"]:
        tAxis.relim()
        tAxis.autoscale_view()

    pGraph["figure"].canvas.draw()
    pGraph["figure"].canvas.flush_events()
    plt.pause(0.001)


def update_rocket(pRocket, pSideTilt, pForwardTilt, pSpin):
    update_part(
        pRocket["body"],
        pRocket["body_base_pos"],
        pRocket["body_base_axis"],
        pRocket["body_base_up"],
        pSideTilt,
        pForwardTilt,
        pSpin,
    )

    update_part(
        pRocket["nose"],
        pRocket["nose_base_pos"],
        pRocket["nose_base_axis"],
        pRocket["nose_base_up"],
        pSideTilt,
        pForwardTilt,
        pSpin,
    )

    tFinRadialSize = pRocket["fin_radial_size"]
    tFinCenterY = pRocket["fin_center_y"]
    tFinDistance = pRocket["fin_distance"]

    update_part(
        pRocket["fin_x_positive"],
        vector(tFinDistance, tFinCenterY, 0),
        vector(tFinRadialSize, 0, 0),
        vector(0, 1, 0),
        pSideTilt,
        pForwardTilt,
        pSpin,
    )

    update_part(
        pRocket["fin_x_negative"],
        vector(-tFinDistance, tFinCenterY, 0),
        vector(tFinRadialSize, 0, 0),
        vector(0, 1, 0),
        pSideTilt,
        pForwardTilt,
        pSpin,
    )

    update_part(
        pRocket["fin_z_positive"],
        vector(0, tFinCenterY, tFinDistance),
        vector(0, 0, tFinRadialSize),
        vector(0, 1, 0),
        pSideTilt,
        pForwardTilt,
        pSpin,
    )

    update_part(
        pRocket["fin_z_negative"],
        vector(0, tFinCenterY, -tFinDistance),
        vector(0, 0, tFinRadialSize),
        vector(0, 1, 0),
        pSideTilt,
        pForwardTilt,
        pSpin,
    )

    pRocket["text"].text = (
        f"Forward-Back Tilt: {math.degrees(pForwardTilt):.1f} deg\n"
        f"Left-Right Tilt: {math.degrees(pSideTilt):.1f} deg\n"
        f"Body Spin: {math.degrees(pSpin):.1f} deg"
    )


def main():
    tRocket = create_rocket_scene()
    time.sleep(1.0)

    tSerial = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0)
    tSerial.reset_input_buffer()

    tRecording = False
    tStartTime = None
    tLastDataTime = None
    tStopDetectedTime = None
    tSaved = False

    tTime = []

    tAx, tAy, tAz = [], [], []
    tGx, tGy, tGz = [], [], []
    tTemp = []
    tPressure = []
    tAltitude = []

    tGyroCalibrationSamples = []
    tGyroOffsetX = 0.0
    tGyroOffsetY = 0.0
    tGyroOffsetZ = 0.0
    tGyroCalibrated = False
    tRequiredCalibrationCount = 30

    tRoll = 0.0
    tPitch = 0.0
    tYaw = 0.0

    tRollOffset = None
    tPitchOffset = None
    tYawOffset = None

    tLastSimulationTime = None

    print("Waiting for receiver raw packets...")
    print(f"Plot output directory: {PLOTS_DIR}")
    print(f"CSV output directory: {TELEMETRY_DIR}")

    while True:
        rate(50)

        tNowAbsolute = time.time()

        if tStopDetectedTime is not None and not tSaved:
            if tNowAbsolute - tStopDetectedTime > DATA_TIMEOUT_SECONDS:
                print("STOP cooldown finished. Saving outputs...")
                save_outputs(
                    None,
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
                tSaved = True
                break

        if tRecording and tLastDataTime is not None:
            if tNowAbsolute - tLastDataTime > DATA_TIMEOUT_SECONDS and not tSaved:
                print("Telemetry timeout. Saving outputs...")
                save_outputs(
                    None,
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
                tSaved = True
                break

        tRawBytes = tSerial.readline()

        if not tRawBytes:
            time.sleep(0.001)
            continue

        tLine = tRawBytes.decode(errors="ignore").strip()

        if not tLine:
            time.sleep(0.001)
            continue

        tPacket = extract_raw_packet(tLine)

        if tPacket is None:
            continue

        #print(f"Raw packet captured: {tPacket}")

        if tPacket == "S":
            tRecording = True
            tStartTime = time.time()
            tLastDataTime = time.time()
            tStopDetectedTime = None
            print("Recording started.")
            continue

        if tPacket == "T":
            tStopDetectedTime = time.time()
            print("STOP packet received. Waiting cooldown before saving...")
            continue

        tData = parse_packet(tPacket)

        if tData is None:
            continue

        if not tRecording:
            tRecording = True
            tStartTime = time.time()
            print("Recording auto-started from first data packet.")

        tLastDataTime = time.time()
        tStopDetectedTime = None

        tNow = tLastDataTime - tStartTime

        tCurrentAx = tData[0]
        tCurrentAy = tData[1]
        tCurrentAz = tData[2]
        tCurrentGxRaw = tData[3]
        tCurrentGyRaw = tData[4]
        tCurrentGzRaw = tData[5]
        tCurrentTemp = tData[6]
        tCurrentPressure = tData[7]
        tCurrentAltitude = tData[8]

        tTime.append(tNow)

        tAx.append(tCurrentAx)
        tAy.append(tCurrentAy)
        tAz.append(tCurrentAz)

        tGx.append(tCurrentGxRaw)
        tGy.append(tCurrentGyRaw)
        tGz.append(tCurrentGzRaw)

        tTemp.append(tCurrentTemp)
        tPressure.append(tCurrentPressure)
        tAltitude.append(tCurrentAltitude)

        #update_graph(
        #    tGraph,
        #    tTime,
        #    tAx,
        #    tAy,
        #    tAz,
        #    tGx,
        #    tGy,
        #    tGz,
        #    tTemp,
        #    tPressure,
        #    tAltitude,
        #)

        tCurrentGx = raw_gyro_to_rad_s(tCurrentGxRaw)
        tCurrentGy = raw_gyro_to_rad_s(tCurrentGyRaw)
        tCurrentGz = raw_gyro_to_rad_s(tCurrentGzRaw)

        if not tGyroCalibrated:
            tGyroCalibrationSamples.append((tCurrentGx, tCurrentGy, tCurrentGz))
            tRocket["text"].text = (
                f"Gyro calibration...\n"
                f"{len(tGyroCalibrationSamples)} / {tRequiredCalibrationCount}"
            )

            if len(tGyroCalibrationSamples) >= tRequiredCalibrationCount:
                tGyroOffsetX = sum(p[0] for p in tGyroCalibrationSamples) / len(tGyroCalibrationSamples)
                tGyroOffsetY = sum(p[1] for p in tGyroCalibrationSamples) / len(tGyroCalibrationSamples)
                tGyroOffsetZ = sum(p[2] for p in tGyroCalibrationSamples) / len(tGyroCalibrationSamples)

                tGyroCalibrated = True
                tLastSimulationTime = time.time()
                tRocket["text"].text = "Gyro calibration finished."

                print("Gyro calibration finished.")
                print(f"GX offset: {tGyroOffsetX:.5f}")
                print(f"GY offset: {tGyroOffsetY:.5f}")
                print(f"GZ offset: {tGyroOffsetZ:.5f}")

            continue

        tSimulationNow = time.time()

        if tLastSimulationTime is None:
            tLastSimulationTime = tSimulationNow
            continue

        tDeltaTime = tSimulationNow - tLastSimulationTime
        tLastSimulationTime = tSimulationNow

        if tDeltaTime <= 0:
            continue

        if tDeltaTime > 0.3:
            tDeltaTime = 0.1

        tCurrentGx = apply_deadband(tCurrentGx - tGyroOffsetX, gGyroDeadband)
        tCurrentGy = apply_deadband(tCurrentGy - tGyroOffsetY, gGyroDeadband)
        tCurrentGz = apply_deadband(tCurrentGz - tGyroOffsetZ, gGyroDeadband)

        tAccRoll = math.atan2(tCurrentAy, tCurrentAz)
        tAccPitch = math.atan2(-tCurrentAx, math.sqrt(tCurrentAy * tCurrentAy + tCurrentAz * tCurrentAz))

        tRoll = gAlpha * (tRoll + tCurrentGx * tDeltaTime) + (1.0 - gAlpha) * tAccRoll
        tPitch = gAlpha * (tPitch + tCurrentGy * tDeltaTime) + (1.0 - gAlpha) * tAccPitch
        tYaw += tCurrentGz * tDeltaTime

        if tRollOffset is None:
            tRollOffset = tRoll
            tPitchOffset = tPitch
            tYawOffset = tYaw

        tSideTilt = gSideTiltSign * (tPitch - tPitchOffset)
        tForwardTilt = gForwardTiltSign * (tRoll - tRollOffset)
        tSpin = gSpinSign * (tYaw - tYawOffset)

        update_rocket(tRocket, tSideTilt, tForwardTilt, tSpin)

    tSerial.close()
    plt.ioff()
    print("Program finished.")


if __name__ == "__main__":
    main()