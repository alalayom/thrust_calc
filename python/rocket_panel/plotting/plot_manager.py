from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd

def save_thrust_plot(pDf: pd.DataFrame, pOutputPath: Path) -> None:
    if pDf.empty:
        print("Could not create graph: data is empty.")
        return

    plt.figure(figsize=(10, 5))
    plt.plot(pDf["time_s"], pDf["thrust_n"])
    plt.xlabel("Time (s)")
    plt.ylabel("Thrust (N)")
    plt.title("Thrust vs Time")
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(pOutputPath, dpi=150)
    plt.show()