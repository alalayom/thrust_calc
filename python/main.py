from pathlib import Path

from utils import ensure_directories, make_timestamp
from serial_reader import collect_serial_data
from process_data import (
    process_dataframe,
    save_processed_dataframe,
    calculate_metrics,
)
from plot_data import save_thrust_plot


def main() -> None:
    tPort = "COM4"
    tBaudrate = 57600

    ensure_directories()
    tTimestamp = make_timestamp()

    tRawCsvPath = Path(f"data/raw/{tTimestamp}_raw.csv")
    tProcessedCsvPath = Path(f"data/processed/{tTimestamp}_processed.csv")
    tPlotPath = Path(f"data/plots/{tTimestamp}_thrust.png")

    tRawDf = collect_serial_data(
        pPort=tPort,
        pBaudrate=tBaudrate,
        pRawCsvPath=tRawCsvPath,
    )

    if tRawDf.empty:
        print("Could not collect any data.")
        return

    tProcessedDf = process_dataframe(pDf=tRawDf)
    save_processed_dataframe(pDf=tProcessedDf, pOutputPath=tProcessedCsvPath)
    save_thrust_plot(pDf=tProcessedDf, pOutputPath=tPlotPath)

    tMetrics = calculate_metrics(pDf=tProcessedDf)

    print("\n--- Test Summary ---")
    print(f"Raw data       : {tRawCsvPath}")
    print(f"Processed data : {tProcessedCsvPath}")
    print(f"Graph          : {tPlotPath}")
    print(f"Max thrust     : {tMetrics['max_thrust_n']:.4f} N")
    print(f"Burn time      : {tMetrics['burn_time_s']:.4f} s")
    print(f"Total impulse  : {tMetrics['total_impulse_ns']:.4f} N.s")


if __name__ == "__main__":
    main()