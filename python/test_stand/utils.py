from datetime import datetime
from pathlib import Path


def ensure_directories(pBaseDataDir: Path) -> None:
    tDirectories = [
        pBaseDataDir / "raw",
        pBaseDataDir / "processed",
        pBaseDataDir / "plots",
        pBaseDataDir / "exports",
    ]

    for tDirectory in tDirectories:
        tDirectory.mkdir(parents=True, exist_ok=True)


def make_timestamp() -> str:
    return datetime.now().strftime("%Y-%m-%d_%H%M%S")