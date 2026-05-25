from pathlib import Path

from rocket_panel.config.paths import (
    get_data_dir,
    ensure_data_directories,
    make_timestamp,
)
from rocket_panel.serial_io.serial_receiver import collect_serial_data
from rocket_panel.parsers.thrust_parser import (
    process_dataframe,
    save_processed_dataframe,
    calculate_metrics,
)
from rocket_panel.plotting.plot_manager import save_thrust_plot


def run_motor_test(
    port: str,
    baudrate: int = 57600,
    warmup_duration_s: float = 5.0,
) -> dict:
    data_dir = get_data_dir()
    ensure_data_directories(base_data_dir=data_dir)

    timestamp = make_timestamp()

    raw_csv_path = data_dir / "raw" / f"{timestamp}_raw.csv"
    processed_csv_path = data_dir / "processed" / f"{timestamp}_processed.csv"
    plot_path = data_dir / "plots" / f"{timestamp}_thrust.png"

    raw_df = collect_serial_data(
        pPort=port,
        pBaudrate=baudrate,
        pRawCsvPath=raw_csv_path,
        pWarmupDurationS=warmup_duration_s,
    )

    if raw_df.empty:
        return {
            "success": False,
            "message": "Could not collect any data.",
        }

    processed_df = process_dataframe(pDf=raw_df)
    save_processed_dataframe(pDf=processed_df, pOutputPath=processed_csv_path)
    save_thrust_plot(pDf=processed_df, pOutputPath=plot_path)

    metrics = calculate_metrics(pDf=processed_df)

    return {
        "success": True,
        "raw_csv_path": raw_csv_path,
        "processed_csv_path": processed_csv_path,
        "plot_path": plot_path,
        "metrics": metrics,
    }