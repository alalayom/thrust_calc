from pathlib import Path

import pandas as pd


G_TO_NEWTON = 9.80665 / 1000.0


def process_dataframe(pDf: pd.DataFrame) -> pd.DataFrame:
    if pDf.empty:
        return pDf.copy()

    tProcessedDf = pDf.copy()

    tProcessedDf["time_s"] = tProcessedDf["time_ms"] / 1000.0
    tProcessedDf["time_s"] = tProcessedDf["time_s"] - tProcessedDf["time_s"].iloc[0]

    tProcessedDf["thrust_n"] = tProcessedDf["mass_g"] * G_TO_NEWTON
    tProcessedDf["thrust_n"] = tProcessedDf["thrust_n"].clip(lower=0)

    return tProcessedDf


def save_processed_dataframe(pDf: pd.DataFrame, pOutputPath: Path) -> None:
    pDf.to_csv(pOutputPath, index=False)


def calculate_metrics(pDf: pd.DataFrame) -> dict:
    if pDf.empty or len(pDf) < 2:
        return {
            "max_thrust_n": 0.0,
            "burn_time_s": 0.0,
            "total_impulse_ns": 0.0,
        }

    tMaxThrust = float(pDf["thrust_n"].max())
    tBurnTime = float(pDf["time_s"].iloc[-1] - pDf["time_s"].iloc[0])

    tTotalImpulse = 0.0
    for tIndex in range(1, len(pDf)):
        tDt = pDf["time_s"].iloc[tIndex] - pDf["time_s"].iloc[tIndex - 1]
        tAverageForce = (
            pDf["thrust_n"].iloc[tIndex] + pDf["thrust_n"].iloc[tIndex - 1]
        ) / 2.0
        tTotalImpulse += tAverageForce * tDt

    return {
        "max_thrust_n": tMaxThrust,
        "burn_time_s": tBurnTime,
        "total_impulse_ns": tTotalImpulse,
    }