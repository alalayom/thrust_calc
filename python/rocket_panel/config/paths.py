from datetime import datetime
from pathlib import Path

def get_project_root() -> Path:
    return Path(__file__).resolve().parents[3]


def get_data_dir() -> Path:
    return get_project_root() / "data"

def ensure_directories(pBaseDataDir: Path) -> None:
    tDirectories = [
        pBaseDataDir / "raw",
        pBaseDataDir / "processed",
        pBaseDataDir / "plots",
        pBaseDataDir / "exports",
        pBaseDataDir / "telemetry",
    ]

    for tDirectory in tDirectories:
        tDirectory.mkdir(parents=True, exist_ok=True)


def make_timestamp() -> str:
    return datetime.now().strftime("%Y-%m-%d_%H%M%S")