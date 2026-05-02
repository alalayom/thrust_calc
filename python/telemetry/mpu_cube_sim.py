import math
import time
import serial
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

gPort = "COM7"
gBaudrate = 115200

gAlpha = 0.97
gGyroDeadband = 0.012

gBodyLength = 4.0
gNoseLength = 0.9
gRocketRadius = 0.35

# Sensör yerde yatay duruyor kabulü:
# Roll/Pitch karışırsa sadece bu işaretlerle oynayacağız.
gSideTiltSign = -1.0       # sağ-sol eğilme
gForwardTiltSign = 1.0    # ileri-geri eğilme
gSpinSign = 1.0           # kendi etrafında dönme


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
    # Roket başlangıçta Y ekseninde dik duruyor.
    # Side tilt: Z ekseni etrafında döndürür, roket sağa/sola yatar.
    # Forward tilt: X ekseni etrafında döndürür, roket ileri/geri yatar.
    tRotated = rotate_around_axis(pVector, vector(0, 0, 1), pSideTilt)
    tRotated = rotate_around_axis(tRotated, vector(1, 0, 0), pForwardTilt)

    return tRotated


def rotate_rocket_vector(pVector, pSideTilt, pForwardTilt, pSpin):
    tRotated = rotate_tilt_only(pVector, pSideTilt, pForwardTilt)

    # Spin, roketin eğilmiş gövde ekseni etrafında dönmeli.
    tBodyAxis = rotate_tilt_only(vector(0, 1, 0), pSideTilt, pForwardTilt)

    return rotate_around_axis(tRotated, tBodyAxis, pSpin)


def apply_deadband(pValue, pDeadband):
    if abs(pValue) < pDeadband:
        return 0.0

    return pValue


def read_imu_line(pSerial):
    while True:
        tLine = pSerial.readline().decode(errors="ignore").strip()

        if not tLine:
            continue

        if tLine.startswith("AX"):
            continue

        try:
            return tuple(map(float, tLine.split(",")))
        except ValueError:
            continue


def calibrate_gyro(pSerial, pSampleCount=200):
    print("Gyro calibration started. Keep MPU6050 completely still...")

    tSumGx = 0.0
    tSumGy = 0.0
    tSumGz = 0.0

    for _ in range(pSampleCount):
        tAx, tAy, tAz, tGx, tGy, tGz = read_imu_line(pSerial)

        tSumGx += tGx
        tSumGy += tGy
        tSumGz += tGz

        time.sleep(0.005)

    tOffsetGx = tSumGx / pSampleCount
    tOffsetGy = tSumGy / pSampleCount
    tOffsetGz = tSumGz / pSampleCount

    print("Gyro calibration finished.")
    print(f"GX offset: {tOffsetGx:.5f}")
    print(f"GY offset: {tOffsetGy:.5f}")
    print(f"GZ offset: {tOffsetGz:.5f}")

    return tOffsetGx, tOffsetGy, tOffsetGz


def update_part(pPart, pBasePos, pBaseAxis, pBaseUp, pSideTilt, pForwardTilt, pSpin):
    pPart.pos = rotate_rocket_vector(pBasePos, pSideTilt, pForwardTilt, pSpin)
    pPart.axis = rotate_rocket_vector(pBaseAxis, pSideTilt, pForwardTilt, pSpin)
    pPart.up = rotate_rocket_vector(pBaseUp, pSideTilt, pForwardTilt, pSpin)


def main():
    tSerial = serial.Serial(gPort, gBaudrate, timeout=1)
    time.sleep(2)

    tGyroOffsetX, tGyroOffsetY, tGyroOffsetZ = calibrate_gyro(tSerial)

    tScene = canvas(
        title="MPU6050 ESP32-S3 Rocket Simulation",
        width=1100,
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

    # Tüm kanatçıklar birebir aynı ölçüde.
    tFinRadialSize = 0.34
    tFinLength = 0.52
    tFinThickness = 0.055
    tFinCenterY = -gBodyLength / 2.0 + 0.35
    tFinDistance = gRocketRadius + tFinRadialSize / 2.0

    tFinSize = vector(tFinRadialSize, tFinLength, tFinThickness)

    tFinXPositive = box(
        pos=vector(tFinDistance, tFinCenterY, 0),
        size=tFinSize,
        color=color.red,
    )

    tFinXNegative = box(
        pos=vector(-tFinDistance, tFinCenterY, 0),
        size=tFinSize,
        color=color.red,
    )

    tFinZPositive = box(
        pos=vector(0, tFinCenterY, tFinDistance),
        size=tFinSize,
        color=color.red,
    )

    tFinZNegative = box(
        pos=vector(0, tFinCenterY, -tFinDistance),
        size=tFinSize,
        color=color.red,
    )

    tText = label(
        pos=vector(0, -4.35, 0),
        text="Starting...",
        height=16,
        color=color.white,
        box=False,
    )

    tRoll = 0.0
    tPitch = 0.0
    tYaw = 0.0

    tRollOffset = None
    tPitchOffset = None
    tYawOffset = None

    tLastTime = time.time()

    print("Rocket simulation started...")

    while True:
        rate(50)

        tAx, tAy, tAz, tGx, tGy, tGz = read_imu_line(tSerial)

        tGx = apply_deadband(tGx - tGyroOffsetX, gGyroDeadband)
        tGy = apply_deadband(tGy - tGyroOffsetY, gGyroDeadband)
        tGz = apply_deadband(tGz - tGyroOffsetZ, gGyroDeadband)

        tNow = time.time()
        tDeltaTime = tNow - tLastTime
        tLastTime = tNow

        if tDeltaTime <= 0 or tDeltaTime > 0.2:
            continue

        # MPU6050 masada yatay duruyor:
        # X/Y yatay düzlem, Z yukarı/aşağı yerçekimi ekseni.
        tAccRoll = math.atan2(tAy, tAz)
        tAccPitch = math.atan2(-tAx, math.sqrt(tAy * tAy + tAz * tAz))

        # Gyro + accelerometer complementary filter
        tRoll = gAlpha * (tRoll + tGx * tDeltaTime) + (1.0 - gAlpha) * tAccRoll
        tPitch = gAlpha * (tPitch + tGy * tDeltaTime) + (1.0 - gAlpha) * tAccPitch

        # Yaw/spin için MPU6050'de manyetometre yok; sadece gyro integration.
        tYaw += tGz * tDeltaTime

        if tRollOffset is None:
            tRollOffset = tRoll
            tPitchOffset = tPitch
            tYawOffset = tYaw

        # Eksen eşleme:
        # Pitch -> sağ/sol eğilme
        # Roll  -> ileri/geri eğilme
        # Yaw   -> roketin kendi gövdesi etrafında dönmesi
        tSideTilt = gSideTiltSign * (tPitch - tPitchOffset)
        tForwardTilt = gForwardTiltSign * (tRoll - tRollOffset)
        tSpin = gSpinSign * (tYaw - tYawOffset)

        update_part(tBody, tBodyBasePos, tBodyBaseAxis, tBodyBaseUp, tSideTilt, tForwardTilt, tSpin)
        update_part(tNose, tNoseBasePos, tNoseBaseAxis, tNoseBaseUp, tSideTilt, tForwardTilt, tSpin)

        update_part(
            tFinXPositive,
            vector(tFinDistance, tFinCenterY, 0),
            vector(tFinRadialSize, 0, 0),
            vector(0, 1, 0),
            tSideTilt,
            tForwardTilt,
            tSpin,
        )

        update_part(
            tFinXNegative,
            vector(-tFinDistance, tFinCenterY, 0),
            vector(tFinRadialSize, 0, 0),
            vector(0, 1, 0),
            tSideTilt,
            tForwardTilt,
            tSpin,
        )

        update_part(
            tFinZPositive,
            vector(0, tFinCenterY, tFinDistance),
            vector(0, 0, tFinRadialSize),
            vector(0, 1, 0),
            tSideTilt,
            tForwardTilt,
            tSpin,
        )

        update_part(
            tFinZNegative,
            vector(0, tFinCenterY, -tFinDistance),
            vector(0, 0, tFinRadialSize),
            vector(0, 1, 0),
            tSideTilt,
            tForwardTilt,
            tSpin,
        )

        tText.text = (
            f"Roll / Forward-Back Tilt: {math.degrees(tForwardTilt):.1f} deg\n"
            f"Pitch / Left-Right Tilt: {math.degrees(tSideTilt):.1f} deg\n"
            f"Yaw / Body Spin: {math.degrees(tSpin):.1f} deg"
        )

        print(
            f"ForwardTilt: {math.degrees(tForwardTilt):7.2f} | "
            f"SideTilt: {math.degrees(tSideTilt):7.2f} | "
            f"Spin: {math.degrees(tSpin):7.2f}"
        )


if __name__ == "__main__":
    main()