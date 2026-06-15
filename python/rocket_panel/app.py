import csv
import io
import math
import queue
import threading
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Optional

import matplotlib

matplotlib.use("TkAgg")
import matplotlib.image as mpimg

import tkinter as tk
from tkinter import ttk

import serial
from serial.tools import list_ports

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DATA_DIR = PROJECT_ROOT / "data"
PLOTS_DIR = DATA_DIR / "plots"
RAW_DIR = DATA_DIR / "raw"
PROCESSED_DIR = DATA_DIR / "processed"
TELEMETRY_DIR = DATA_DIR / "telemetry"

G_TO_NEWTON = 9.80665 / 1000.0
MAX_VISIBLE_POINTS = 1200
STATIC_WARMUP_SECONDS = 3.0
STATIC_BASELINE_SAMPLE_COUNT = 30
STATIC_BASELINE_MAX_RANGE_G = 50.0
MIN_MAP_VIEW_SPAN_DEG = 0.002
MAP_TILE_THROTTLE_SECONDS = 0.35
MAX_MAP_TILES = 24
MIN_OSM_ZOOM = 1
MAX_OSM_ZOOM = 19
DEFAULT_OSM_ZOOM = 13
OSM_TILE_SIZE_PX = 256
MAP_CENTER_UPDATE_THRESHOLD_M = 50.0
EARTH_RADIUS_M = 6371000.0

MODE_STATIC = "staticTest"
MODE_TELEMETRY = "Telemetry"

IGNITION_ON_COMMAND = b"FIRE\n"
IGNITION_OFF_COMMAND = b"SAFE\n"


def ensure_output_dirs() -> None:
    for directory in (PLOTS_DIR, RAW_DIR, PROCESSED_DIR, TELEMETRY_DIR):
        directory.mkdir(parents=True, exist_ok=True)


def make_timestamp() -> str:
    return datetime.now().strftime("%Y-%m-%d_%H%M%S")


def available_ports() -> list[str]:
    return [port.device for port in list_ports.comports()]


def parse_float_parts(parts: list[str]) -> Optional[list[float]]:
    try:
        return [float(part.strip()) for part in parts]
    except ValueError:
        return None


def is_gps_sentinel(value: Optional[float]) -> bool:
    if value is None:
        return True
    return abs(value - 999.0) < 0.0001 or abs(value + 999.0) < 0.0001


def is_gps_connected(latitude: Optional[float], longitude: Optional[float]) -> bool:
    if is_gps_sentinel(latitude) or is_gps_sentinel(longitude):
        return False
    return -90.0 <= latitude <= 90.0 and -180.0 <= longitude <= 180.0


def gps_distance_m(lat_a: float, lon_a: float, lat_b: float, lon_b: float) -> float:
    lat_a_rad = math.radians(lat_a)
    lat_b_rad = math.radians(lat_b)
    delta_lat = math.radians(lat_b - lat_a)
    delta_lon = math.radians(lon_b - lon_a)
    haversine = (
        math.sin(delta_lat / 2.0) ** 2
        + math.cos(lat_a_rad) * math.cos(lat_b_rad) * math.sin(delta_lon / 2.0) ** 2
    )
    haversine = max(0.0, min(1.0, haversine))
    return EARTH_RADIUS_M * 2.0 * math.atan2(math.sqrt(haversine), math.sqrt(1.0 - haversine))


def extract_telemetry_packet(line: str) -> Optional[str]:
    line = line.strip()

    if line.startswith("Raw packet:"):
        return line.replace("Raw packet:", "", 1).strip()

    if line in ("S", "T"):
        return line

    if line.startswith("D,"):
        return line

    parts = line.split(",")
    if len(parts) in (9, 13) and parse_float_parts(parts) is not None:
        return "D," + line

    return None


@dataclass
class SerialEvent:
    kind: str
    payload: object


class SerialReader(threading.Thread):
    def __init__(self, port: str, baudrate: int, mode: str, output_queue: queue.Queue):
        super().__init__(daemon=True)
        self.port = port
        self.baudrate = baudrate
        self.mode = mode
        self.output_queue = output_queue
        self.stop_event = threading.Event()
        self.serial_handle = None
        self.start_time = None
        self.static_baseline_g = None
        self.static_baseline_samples = []

    def stop(self) -> None:
        self.stop_event.set()
        if self.serial_handle is not None:
            try:
                self.serial_handle.close()
            except serial.SerialException:
                pass

    def run(self) -> None:
        try:
            self.serial_handle = serial.Serial(self.port, self.baudrate, timeout=0.05)
            self.serial_handle.reset_input_buffer()
        except (OSError, serial.SerialException) as exc:
            self.output_queue.put(
                SerialEvent(
                    "error",
                    (
                        f"Could not open {self.port}: {exc}. "
                        "Close Arduino Serial Monitor, other Python scripts, and any app using this COM port."
                    ),
                )
            )
            return

        self.start_time = time.time()
        self.output_queue.put(SerialEvent("status", f"Connected to {self.port} at {self.baudrate} baud"))

        while not self.stop_event.is_set():
            try:
                raw_bytes = self.serial_handle.readline()
            except (OSError, serial.SerialException) as exc:
                self.output_queue.put(SerialEvent("error", f"Serial read failed: {exc}"))
                break

            if not raw_bytes:
                continue

            line = raw_bytes.decode(errors="ignore").strip()
            if not line:
                continue

            if self.mode == MODE_TELEMETRY:
                self.handle_telemetry_line(line)
            else:
                self.handle_static_line(line)

        self.output_queue.put(SerialEvent("status", "Data reader stopped"))

    def elapsed(self) -> float:
        if self.start_time is None:
            return 0.0
        return time.time() - self.start_time

    def handle_static_line(self, line: str) -> None:
        parts = line.split(",")
        if len(parts) != 2:
            return

        values = parse_float_parts(parts)
        if values is None:
            return

        time_ms, mass_g = values
        elapsed_s = self.elapsed()

        if elapsed_s < STATIC_WARMUP_SECONDS:
            return

        if self.static_baseline_g is None:
            self.static_baseline_samples.append(mass_g)

            if len(self.static_baseline_samples) > STATIC_BASELINE_SAMPLE_COUNT:
                self.static_baseline_samples.pop(0)

            if len(self.static_baseline_samples) < STATIC_BASELINE_SAMPLE_COUNT:
                self.output_queue.put(
                    SerialEvent(
                        "status",
                        f"Static baseline: {len(self.static_baseline_samples)} / {STATIC_BASELINE_SAMPLE_COUNT}",
                    )
                )
                return

            baseline_range = max(self.static_baseline_samples) - min(self.static_baseline_samples)
            if baseline_range > STATIC_BASELINE_MAX_RANGE_G:
                self.output_queue.put(
                    SerialEvent(
                        "status",
                        f"Waiting for stable load cell baseline. Range: {baseline_range:.1f} g",
                    )
                )
                return

            sorted_samples = sorted(self.static_baseline_samples)
            midpoint = len(sorted_samples) // 2
            self.static_baseline_g = sorted_samples[midpoint]
            self.start_time = time.time()
            self.output_queue.put(
                SerialEvent(
                    "status",
                    f"Static baseline ready: {self.static_baseline_g:.2f} g",
                )
            )
            return

        corrected_mass_g = mass_g - self.static_baseline_g
        thrust_n = max(0.0, corrected_mass_g * G_TO_NEWTON)

        self.output_queue.put(
            SerialEvent(
                "static",
                {
                    "time_ms": time_ms,
                    "time_s": self.elapsed(),
                    "mass_g": corrected_mass_g,
                    "raw_mass_g": mass_g,
                    "thrust_n": thrust_n,
                },
            )
        )

    def handle_telemetry_line(self, line: str) -> None:
        packet = extract_telemetry_packet(line)
        if packet is None:
            return

        if packet == "S":
            self.start_time = time.time()
            self.output_queue.put(SerialEvent("telemetry_control", "S"))
            return

        if packet == "T":
            self.output_queue.put(SerialEvent("telemetry_control", "T"))
            return

        parts = packet.split(",")
        if len(parts) not in (10, 14) or parts[0] != "D":
            return

        values = parse_float_parts(parts[1:])
        if values is None:
            return

        ax, ay, az, gx, gy, gz, temp_c, pressure_hpa, altitude_m = values[:9]

        latitude = None
        longitude = None
        gps_satellites = None
        gps_altitude_m = None

        if len(values) == 13:
            latitude = values[9]
            longitude = values[10]
            gps_satellites = int(values[11])
            gps_altitude_m = values[12]

            if is_gps_sentinel(latitude):
                latitude = None

            if is_gps_sentinel(longitude):
                longitude = None

            if gps_satellites < 0 or gps_satellites >= 999:
                gps_satellites = None

            if is_gps_sentinel(gps_altitude_m):
                gps_altitude_m = None

        gps_connected = is_gps_connected(latitude, longitude)

        self.output_queue.put(
            SerialEvent(
                "telemetry",
                {
                    "time_s": self.elapsed(),
                    "ax": ax,
                    "ay": ay,
                    "az": az,
                    "gx": gx,
                    "gy": gy,
                    "gz": gz,
                    "temperature_c": temp_c,
                    "pressure_hpa": pressure_hpa,
                    "altitude_m": altitude_m,
                    "latitude": latitude,
                    "longitude": longitude,
                    "gps_satellites": gps_satellites,
                    "gps_altitude_m": gps_altitude_m,
                    "gps_connected": gps_connected,
                },
            )
        )


class RocketEstimator:
    def __init__(self) -> None:
        self.alpha = 0.97
        self.gyro_deadband = 0.012
        self.gyro_sensitivity = 65.5
        self.side_tilt_sign = 1.0
        self.forward_tilt_sign = 1.0
        self.spin_sign = 1.0
        self.reset()

    def reset(self) -> None:
        self.calibration_samples = []
        self.required_calibration_count = 30
        self.offset_x = 0.0
        self.offset_y = 0.0
        self.offset_z = 0.0
        self.calibrated = False
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.roll_offset = None
        self.pitch_offset = None
        self.yaw_offset = None
        self.last_time = None
        self.status = "Waiting for telemetry"

    def raw_gyro_to_rad_s(self, raw_value: float) -> float:
        return math.radians(raw_value / self.gyro_sensitivity)

    def apply_deadband(self, value: float) -> float:
        if abs(value) < self.gyro_deadband:
            return 0.0
        return value

    def update(self, sample: dict) -> tuple[float, float, float]:
        gx = self.raw_gyro_to_rad_s(sample["gx"])
        gy = self.raw_gyro_to_rad_s(sample["gy"])
        gz = self.raw_gyro_to_rad_s(sample["gz"])

        if not self.calibrated:
            self.calibration_samples.append((gx, gy, gz))
            self.status = f"Gyro calibration {len(self.calibration_samples)} / {self.required_calibration_count}"

            if len(self.calibration_samples) >= self.required_calibration_count:
                count = len(self.calibration_samples)
                self.offset_x = sum(item[0] for item in self.calibration_samples) / count
                self.offset_y = sum(item[1] for item in self.calibration_samples) / count
                self.offset_z = sum(item[2] for item in self.calibration_samples) / count
                self.calibrated = True
                self.last_time = time.time()
                self.status = "Gyro calibration ready"

            return self.current_orientation()

        now = time.time()
        if self.last_time is None:
            self.last_time = now
            return self.current_orientation()

        delta_time = now - self.last_time
        self.last_time = now

        if delta_time <= 0:
            return self.current_orientation()

        if delta_time > 0.3:
            delta_time = 0.1

        gx = self.apply_deadband(gx - self.offset_x)
        gy = self.apply_deadband(gy - self.offset_y)
        gz = self.apply_deadband(gz - self.offset_z)

        ax = sample["ax"]
        ay = sample["ay"]
        az = sample["az"]

        acc_roll = math.atan2(ay, az)
        acc_pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))

        self.roll = self.alpha * (self.roll + gx * delta_time) + (1.0 - self.alpha) * acc_roll
        self.pitch = self.alpha * (self.pitch + gy * delta_time) + (1.0 - self.alpha) * acc_pitch
        self.yaw += gz * delta_time

        if self.roll_offset is None:
            self.roll_offset = self.roll
            self.pitch_offset = self.pitch
            self.yaw_offset = self.yaw

        self.status = "Simulation running"
        return self.current_orientation()

    def current_orientation(self) -> tuple[float, float, float]:
        if self.roll_offset is None:
            return 0.0, 0.0, 0.0

        side_tilt = self.side_tilt_sign * (self.pitch - self.pitch_offset)
        forward_tilt = self.forward_tilt_sign * (self.roll - self.roll_offset)
        spin = self.spin_sign * (self.yaw - self.yaw_offset)
        return side_tilt, forward_tilt, spin


class RocketPanelApp:
    def __init__(self) -> None:
        ensure_output_dirs()

        self.root = tk.Tk()
        self.root.title("Rocket Control Panel")
        self.root.state("zoomed")
        self.root.configure(bg="#111318")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.bind("<F11>", self.toggle_fullscreen)
        self.root.bind("<Escape>", self.exit_fullscreen)

        self.event_queue: queue.Queue = queue.Queue()
        self.reader: Optional[SerialReader] = None
        self.reader_running = False
        self.fullscreen = False
        self.mode_var = tk.StringVar(value=MODE_TELEMETRY)
        self.data_port_var = tk.StringVar()
        self.ignition_port_var = tk.StringVar()
        self.data_baud_var = tk.StringVar(value="9600")
        self.ignition_baud_var = tk.StringVar(value="9600")
        self.fuse_var = tk.BooleanVar(value=False)
        self.status_var = tk.StringVar(value="Ready")

        self.samples: list[dict] = []
        self.static_time_offset: Optional[float] = None
        self.last_plot_time = 0.0
        self.rocket_estimator = RocketEstimator()
        self.orientation = (0.0, 0.0, 0.0)
        self.gps_path: list[tuple[float, float]] = []
        self.map_center_lat = 0.0
        self.map_center_lon = 0.0
        self.map_initialized_from_gps = False
        self.map_tile_cache = {}
        self.map_tiles_enabled = True
        self.map_image_artists = []
        self.map_tile_request_queue: queue.Queue = queue.Queue()
        self.map_tile_response_queue: queue.Queue = queue.Queue()
        self.map_pending_tile_key = None
        self.map_current_tile_key = None
        self.map_last_tile_request_time = 0.0
        self.map_zoom_level = DEFAULT_OSM_ZOOM
        self.map_info_var = tk.StringVar(value="center: 0.00000, 0.00000 | zoom: 13")
        self.map_worker = threading.Thread(target=self.map_tile_worker, daemon=True)
        self.map_worker.start()

        self.live_values: dict[str, tk.StringVar] = {}
        self.gps_status_var = tk.StringVar(value="satellite not connected")
        self.gps_status_label = None

        self.build_styles()
        self.build_layout()
        self.refresh_ports()
        self.configure_mode_graph()
        self.configure_mode_visibility()

        self.root.after(40, self.process_events)

    def build_styles(self) -> None:
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TFrame", background="#111318")
        style.configure("Panel.TFrame", background="#181b22")
        style.configure("TLabel", background="#111318", foreground="#f2f4f8", font=("Segoe UI", 10))
        style.configure("Panel.TLabel", background="#181b22", foreground="#f2f4f8", font=("Segoe UI", 10))
        style.configure("Title.TLabel", background="#181b22", foreground="#ffffff", font=("Segoe UI", 13, "bold"))
        style.configure("Status.TLabel", background="#111318", foreground="#9fb3c8", font=("Segoe UI", 10))
        style.configure("TButton", font=("Segoe UI", 10), padding=(10, 5))
        style.configure("Danger.TButton", font=("Segoe UI", 11, "bold"), foreground="#ffffff", background="#b42318")
        style.map("Danger.TButton", background=[("active", "#d92d20")])
        style.configure("Safe.TButton", font=("Segoe UI", 11, "bold"), foreground="#ffffff", background="#1f7a4d")
        style.map("Safe.TButton", background=[("active", "#23875a")])
        style.configure("TRadiobutton", background="#181b22", foreground="#f2f4f8", font=("Segoe UI", 10))
        style.configure("TCombobox", padding=(5, 4))

    def build_layout(self) -> None:
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(1, weight=1)

        self.build_top_bar()

        content = ttk.Frame(self.root, style="TFrame", padding=(10, 6, 10, 6))
        content.grid(row=1, column=0, sticky="nsew")
        content.columnconfigure(0, weight=3)
        content.columnconfigure(1, weight=2)
        content.rowconfigure(0, weight=1)

        self.graph_frame = ttk.Frame(content, style="Panel.TFrame", padding=8)
        self.graph_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 6))
        self.graph_frame.rowconfigure(1, weight=1)
        self.graph_frame.columnconfigure(0, weight=1)
        ttk.Label(self.graph_frame, text="Graph", style="Title.TLabel").grid(row=0, column=0, sticky="w")

        self.figure = Figure(figsize=(8, 6), dpi=100, facecolor="#181b22")
        self.canvas = FigureCanvasTkAgg(self.figure, master=self.graph_frame)
        self.canvas.get_tk_widget().grid(row=1, column=0, sticky="nsew", pady=(8, 0))

        self.side_frame = ttk.Frame(content, style="Panel.TFrame", padding=8)
        self.side_frame.grid(row=0, column=1, sticky="nsew", padx=(6, 0))
        self.side_frame.columnconfigure(0, weight=1)
        self.side_frame.rowconfigure(1, weight=3)
        self.side_frame.rowconfigure(3, weight=2)

        self.map_header_frame = ttk.Frame(self.side_frame, style="Panel.TFrame")
        self.map_header_frame.grid(row=0, column=0, sticky="ew")
        self.map_header_frame.columnconfigure(1, weight=1)
        self.map_title_label = ttk.Label(self.map_header_frame, text="GPS Map", style="Title.TLabel")
        self.map_title_label.grid(row=0, column=0, sticky="w")
        self.map_info_label = tk.Label(
            self.map_header_frame,
            textvariable=self.map_info_var,
            bg="#222731",
            fg="#c8d4e3",
            font=("Segoe UI", 8),
            padx=8,
            pady=3,
        )
        self.map_info_label.grid(row=0, column=1, sticky="e", padx=(8, 0))
        self.map_figure = Figure(figsize=(5.0, 4.0), dpi=120, facecolor="#181b22")
        self.map_canvas = FigureCanvasTkAgg(self.map_figure, master=self.side_frame)
        self.map_widget = self.map_canvas.get_tk_widget()
        self.map_widget.grid(row=1, column=0, sticky="nsew", pady=(8, 10))
        self.build_map_plot()

        self.sim_title_label = ttk.Label(self.side_frame, text="Rocket Simulation", style="Title.TLabel")
        self.sim_title_label.grid(row=2, column=0, sticky="w")
        self.sim_figure = Figure(figsize=(5.0, 3.0), dpi=100, facecolor="#181b22")
        self.sim_canvas = FigureCanvasTkAgg(self.sim_figure, master=self.side_frame)
        self.sim_widget = self.sim_canvas.get_tk_widget()
        self.sim_widget.grid(row=3, column=0, sticky="nsew", pady=(8, 0))
        self.build_simulation_plot()

        self.build_live_data_bar()
        self.build_bottom_panel()

        status_bar = ttk.Frame(self.root, style="TFrame", padding=(10, 0, 10, 8))
        status_bar.grid(row=4, column=0, sticky="ew")
        ttk.Label(status_bar, textvariable=self.status_var, style="Status.TLabel").pack(side="left")

    def build_top_bar(self) -> None:
        top = ttk.Frame(self.root, style="TFrame", padding=(10, 8, 10, 4))
        top.grid(row=0, column=0, sticky="ew")

        ttk.Label(top, text="Data Port").pack(side="left", padx=(0, 6))
        self.data_port_combo = ttk.Combobox(top, textvariable=self.data_port_var, width=12, state="readonly")
        self.data_port_combo.pack(side="left", padx=(0, 8))

        ttk.Label(top, text="Data Baud").pack(side="left", padx=(0, 6))
        self.data_baud_combo = ttk.Combobox(
            top,
            textvariable=self.data_baud_var,
            width=9,
            values=("9600", "57600", "115200"),
            state="readonly",
        )
        self.data_baud_combo.pack(side="left", padx=(0, 12))

        ttk.Label(top, text="Ignition Port").pack(side="left", padx=(0, 6))
        self.ignition_port_combo = ttk.Combobox(top, textvariable=self.ignition_port_var, width=12, state="readonly")
        self.ignition_port_combo.pack(side="left", padx=(0, 8))

        ttk.Label(top, text="Ignition Baud").pack(side="left", padx=(0, 6))
        self.ignition_baud_combo = ttk.Combobox(
            top,
            textvariable=self.ignition_baud_var,
            width=9,
            values=("9600", "57600", "115200"),
            state="readonly",
        )
        self.ignition_baud_combo.pack(side="left", padx=(0, 12))

        ttk.Button(top, text="Refresh", command=self.refresh_ports).pack(side="left")

    def build_live_data_bar(self) -> None:
        self.live_data_frame = ttk.Frame(self.root, style="Panel.TFrame", padding=(10, 6, 10, 6))
        self.live_data_frame.grid(row=2, column=0, sticky="ew", padx=10, pady=(0, 6))
        self.live_data_frame.columnconfigure(1, weight=1)

        ttk.Label(self.live_data_frame, text="Live Data", style="Title.TLabel").grid(row=0, column=0, sticky="nw", padx=(0, 12))
        self.data_grid = ttk.Frame(self.live_data_frame, style="Panel.TFrame")
        self.data_grid.grid(row=0, column=1, sticky="ew")

    def build_bottom_panel(self) -> None:
        bottom = ttk.Frame(self.root, style="Panel.TFrame", padding=(12, 10, 12, 10))
        bottom.grid(row=3, column=0, sticky="ew", padx=10, pady=(0, 8))

        ttk.Label(bottom, text="selectMode", style="Panel.TLabel").pack(side="left", padx=(0, 10))
        ttk.Radiobutton(
            bottom,
            text="staticTest",
            value=MODE_STATIC,
            variable=self.mode_var,
            command=self.on_mode_changed,
        ).pack(side="left", padx=(0, 10))
        ttk.Radiobutton(
            bottom,
            text="Telemetry",
            value=MODE_TELEMETRY,
            variable=self.mode_var,
            command=self.on_mode_changed,
        ).pack(side="left", padx=(0, 24))

        ttk.Button(bottom, text="Start", command=self.start_reader).pack(side="left", padx=(0, 8))
        ttk.Button(bottom, text="Stop", command=self.stop_reader).pack(side="left", padx=(0, 8))
        ttk.Button(bottom, text="Save", command=self.save_current_session).pack(side="left", padx=(0, 24))

        self.fuse_button = ttk.Button(bottom, text="fuse OFF", style="Safe.TButton", command=self.toggle_fuse)
        self.fuse_button.pack(side="left", padx=(0, 12))

        ttk.Label(
            bottom,
            text="Fuse ON sends FIRE, OFF sends SAFE over ignition port.",
            style="Panel.TLabel",
        ).pack(side="left")

    def build_map_plot(self) -> None:
        self.map_figure.clear()
        self.map_ax = self.map_figure.add_subplot(111)
        self.map_ax.set_facecolor("#181b22")
        self.map_ax.set_axis_off()
        self.map_ax.set_aspect("auto")
        self.gps_route_line, = self.map_ax.plot([], [], color="#22c55e", linewidth=2.8, marker="o", markersize=3, zorder=5)
        self.gps_current_point, = self.map_ax.plot([], [], color="#facc15", marker="o", markersize=8, zorder=6)
        self.map_ax.set_xlim(-0.01, 0.01)
        self.map_ax.set_ylim(-0.01, 0.01)
        self.map_text = self.map_ax.text(
            0.5,
            0.5,
            "0.000000, 0.000000\nWaiting for GPS fix",
            transform=self.map_ax.transAxes,
            ha="center",
            va="center",
            color="#ef4444",
            fontsize=11,
            weight="bold",
        )
        self.map_figure.subplots_adjust(left=0, right=1, top=1, bottom=0)
        self.map_canvas.mpl_connect("scroll_event", self.on_map_scroll)
        self.update_map_info()
        self.map_canvas.draw_idle()

    def build_simulation_plot(self) -> None:
        self.sim_figure.clear()
        self.sim_ax = self.sim_figure.add_subplot(111, projection="3d")
        self.sim_ax.set_facecolor("#181b22")
        self.sim_ax.set_xlim(-3, 3)
        self.sim_ax.set_ylim(-3, 3)
        self.sim_ax.set_zlim(-3, 3)
        self.sim_ax.set_box_aspect((1, 1, 1))
        self.sim_ax.set_xlabel("X", color="#6ea8fe")
        self.sim_ax.set_ylabel("Y", color="#7bd88f")
        self.sim_ax.set_zlabel("Z", color="#f4d35e")
        self.sim_ax.tick_params(colors="#9fb3c8")
        self.sim_ax.view_init(elev=18, azim=-48)
        self.sim_ax.plot([-2.6, 2.6], [0, 0], [0, 0], color="#6ea8fe", linewidth=1.2, alpha=0.75)
        self.sim_ax.plot([0, 0], [-2.6, 2.6], [0, 0], color="#7bd88f", linewidth=1.2, alpha=0.75)
        self.sim_ax.plot([0, 0], [0, 0], [-2.6, 2.6], color="#f4d35e", linewidth=1.2, alpha=0.75)
        self.rocket_line, = self.sim_ax.plot([], [], [], color="#f8f9fa", linewidth=5)
        self.nose_line, = self.sim_ax.plot([], [], [], color="#ff9f1c", linewidth=4)
        self.fin_line_a, = self.sim_ax.plot([], [], [], color="#e76f51", linewidth=3)
        self.fin_line_b, = self.sim_ax.plot([], [], [], color="#e76f51", linewidth=3)
        self.sim_text = self.sim_ax.text2D(0.02, 0.94, "Simulation idle", transform=self.sim_ax.transAxes, color="#f2f4f8")
        self.update_simulation_plot()

    def configure_mode_graph(self) -> None:
        self.figure.clear()
        self.axes = []
        self.lines = {}

        if self.mode_var.get() == MODE_TELEMETRY:
            labels = [
                ("Accel Raw", ("ax", "ay", "az")),
                ("Gyro Raw", ("gx", "gy", "gz")),
                ("Temp C", ("temperature_c",)),
                ("Pressure / Altitude", ("pressure_hpa", "altitude_m", "gps_altitude_m")),
            ]
            for index, (ylabel, keys) in enumerate(labels, start=1):
                axis = self.figure.add_subplot(4, 1, index)
                axis.set_facecolor("#181b22")
                axis.set_ylabel(ylabel, color="#f2f4f8")
                axis.tick_params(colors="#9fb3c8")
                axis.grid(True, color="#303642")
                for key in keys:
                    line, = axis.plot([], [], label=key)
                    self.lines[key] = line
                axis.legend(loc="upper right", fontsize=8)
                self.axes.append(axis)
            self.axes[-1].set_xlabel("Time (s)", color="#f2f4f8")
            self.setup_value_labels(
                [
                    "samples",
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
                    "gps_altitude_m",
                    "latitude",
                    "longitude",
                    "connected_satellites",
                    "gps_status",
                    "roll_deg",
                    "pitch_deg",
                    "yaw_deg",
                    "simulation",
                ]
            )
        else:
            axis = self.figure.add_subplot(1, 1, 1)
            axis.set_facecolor("#181b22")
            axis.set_xlabel("Time (s)", color="#f2f4f8")
            axis.set_ylabel("Thrust (N)", color="#f2f4f8")
            axis.tick_params(colors="#9fb3c8")
            axis.grid(True, color="#303642")
            line, = axis.plot([], [], color="#48cae4", label="thrust_n")
            axis.legend(loc="upper right")
            self.axes.append(axis)
            self.lines["thrust_n"] = line
            self.setup_value_labels(["samples", "time_s", "mass_g", "thrust_n", "max_thrust_n", "burn_time_s", "total_impulse_ns"])

        self.figure.tight_layout()
        self.canvas.draw_idle()

    def configure_mode_visibility(self) -> None:
        if self.mode_var.get() == MODE_TELEMETRY:
            self.map_header_frame.grid(row=0, column=0, sticky="ew")
            self.map_widget.grid(row=1, column=0, sticky="nsew", pady=(8, 10))
            self.sim_title_label.grid(row=2, column=0, sticky="w")
            self.sim_widget.grid(row=3, column=0, sticky="nsew", pady=(8, 0))
            self.side_frame.rowconfigure(1, weight=3)
            self.side_frame.rowconfigure(3, weight=2)
        else:
            self.map_header_frame.grid_remove()
            self.map_widget.grid_remove()
            self.sim_title_label.grid(row=0, column=0, sticky="w")
            self.sim_widget.grid(row=1, column=0, sticky="nsew", pady=(8, 0))
            self.side_frame.rowconfigure(1, weight=1)
            self.side_frame.rowconfigure(3, weight=0)

    def setup_value_labels(self, keys: list[str]) -> None:
        for child in self.data_grid.winfo_children():
            child.destroy()

        self.live_values = {}
        self.gps_status_label = None

        for column, key in enumerate(keys):
            self.data_grid.columnconfigure(column, weight=1)
            cell = ttk.Frame(self.data_grid, style="Panel.TFrame", padding=(4, 0, 4, 0))
            cell.grid(row=0, column=column, sticky="ew", padx=(0, 4))

            ttk.Label(
                cell,
                text=self.format_live_key(key),
                style="Panel.TLabel",
                font=("Segoe UI", 8, "bold"),
            ).grid(row=0, column=0, sticky="w")

            if key == "gps_status":
                self.gps_status_var.set("satellite not connected")
                self.gps_status_label = tk.Label(
                    cell,
                    textvariable=self.gps_status_var,
                    bg="#181b22",
                    fg="#ef4444",
                    font=("Segoe UI", 8, "bold"),
                )
                self.gps_status_label.grid(row=1, column=0, sticky="w")
            else:
                value_var = tk.StringVar(value="-")
                self.live_values[key] = value_var
                ttk.Label(
                    cell,
                    textvariable=value_var,
                    style="Panel.TLabel",
                    font=("Segoe UI", 9),
                ).grid(row=1, column=0, sticky="w")

    @staticmethod
    def format_live_key(key: str) -> str:
        labels = {
            "samples": "samples",
            "time_s": "time",
            "temperature_c": "temp",
            "pressure_hpa": "pressure",
            "altitude_m": "bmp alt",
            "gps_altitude_m": "gps alt",
            "connected_satellites": "sat",
            "gps_status": "gps",
            "roll_deg": "roll",
            "pitch_deg": "pitch",
            "yaw_deg": "yaw",
            "simulation": "sim",
            "max_thrust_n": "max thrust",
            "burn_time_s": "burn",
            "total_impulse_ns": "impulse",
        }
        return labels.get(key, key)

    def refresh_ports(self) -> None:
        ports = available_ports()
        self.data_port_combo["values"] = ports
        self.ignition_port_combo["values"] = ports

        if ports and self.data_port_var.get() not in ports:
            self.data_port_var.set(ports[0])

        if ports and self.ignition_port_var.get() not in ports:
            self.ignition_port_var.set(ports[-1])

        self.status_var.set(f"Ports refreshed: {', '.join(ports) if ports else 'none'}")

    def on_mode_changed(self) -> None:
        if self.reader_running:
            self.stop_reader(save=False)

        if self.mode_var.get() == MODE_STATIC:
            self.data_baud_var.set("57600")
        else:
            self.data_baud_var.set("9600")

        self.reset_session()
        self.configure_mode_graph()
        self.configure_mode_visibility()
        self.status_var.set(f"Mode selected: {self.mode_var.get()}")

    def reset_session(self) -> None:
        self.samples = []
        self.static_time_offset = None
        self.rocket_estimator.reset()
        self.orientation = (0.0, 0.0, 0.0)
        self.gps_path = []
        self.map_center_lat = 0.0
        self.map_center_lon = 0.0
        self.map_initialized_from_gps = False
        self.map_pending_tile_key = None
        self.map_current_tile_key = None
        self.map_zoom_level = DEFAULT_OSM_ZOOM
        self.clear_map_tile_queues()
        if hasattr(self, "map_image_artists"):
            self.clear_map_background()
        self.last_plot_time = 0.0
        for value in self.live_values.values():
            value.set("-")
        self.gps_status_var.set("satellite not connected")
        if self.gps_status_label is not None:
            self.gps_status_label.configure(fg="#ef4444")
        if hasattr(self, "map_ax") and self.mode_var.get() == MODE_TELEMETRY:
            self.update_map_plot()
        self.update_simulation_plot()

    def start_reader(self) -> None:
        if self.reader_running:
            self.status_var.set("Reader is already running")
            return

        port = self.data_port_var.get().strip()
        if not port:
            self.status_var.set("Select a data port first")
            return

        if self.reader is not None and self.reader.is_alive():
            self.status_var.set("Previous reader is still closing. Try again in a second.")
            return

        try:
            baudrate = int(self.data_baud_var.get())
        except ValueError:
            self.status_var.set("Invalid data baud rate")
            return

        self.reset_session()
        self.configure_mode_graph()
        self.reader = SerialReader(port, baudrate, self.mode_var.get(), self.event_queue)
        self.reader.start()
        self.reader_running = True
        self.status_var.set(f"Starting {self.mode_var.get()} on {port}")

    def stop_reader(self, save: bool = True) -> None:
        if self.reader is not None:
            self.reader.stop()
            self.reader.join(timeout=1.0)
            self.reader = None
        self.reader_running = False
        self.status_var.set("Reader stopped")

        if save:
            self.save_current_session()

    def toggle_fuse(self) -> None:
        target_state = not self.fuse_var.get()
        port = self.ignition_port_var.get().strip()
        if not port:
            self.status_var.set("Select an ignition port first")
            return

        if self.reader_running and port == self.data_port_var.get().strip():
            self.status_var.set("Ignition port must be different from the active data port")
            return

        try:
            baudrate = int(self.ignition_baud_var.get())
            command = IGNITION_ON_COMMAND if target_state else IGNITION_OFF_COMMAND
            with serial.Serial(port, baudrate, timeout=1, write_timeout=1) as serial_handle:
                time.sleep(0.2)
                serial_handle.write(command)
                serial_handle.flush()
        except (ValueError, OSError, serial.SerialException) as exc:
            self.status_var.set(f"Ignition command failed: {exc}")
            return

        self.fuse_var.set(target_state)
        if target_state:
            self.fuse_button.configure(text="fuse ON", style="Danger.TButton")
            self.status_var.set(f"FIRE command sent on {port}")
        else:
            self.fuse_button.configure(text="fuse OFF", style="Safe.TButton")
            self.status_var.set(f"SAFE command sent on {port}")

    def process_events(self) -> None:
        while True:
            try:
                event = self.event_queue.get_nowait()
            except queue.Empty:
                break

            if event.kind == "error":
                self.reader_running = False
                self.reader = None
                self.status_var.set(str(event.payload))
            elif event.kind == "status":
                self.status_var.set(str(event.payload))
            elif event.kind == "telemetry_control":
                self.handle_telemetry_control(str(event.payload))
            elif event.kind == "telemetry":
                self.handle_telemetry_sample(event.payload)
            elif event.kind == "static":
                self.handle_static_sample(event.payload)

        self.process_map_tile_responses()

        now = time.time()
        if now - self.last_plot_time > 0.1:
            self.update_graph()
            if self.mode_var.get() == MODE_TELEMETRY:
                self.update_map_plot()
            self.update_simulation_plot()
            self.last_plot_time = now

        self.root.after(40, self.process_events)

    def handle_telemetry_control(self, command: str) -> None:
        if command == "S":
            self.status_var.set("Telemetry stream started")
        elif command == "T":
            self.status_var.set("Telemetry stream stopped")

    def handle_telemetry_sample(self, sample: dict) -> None:
        self.samples.append(sample)
        side, forward, spin = self.rocket_estimator.update(sample)
        self.orientation = (side, forward, spin)

        if sample["gps_connected"]:
            if not self.map_initialized_from_gps:
                self.gps_path = []
                self.map_center_lat = sample["latitude"]
                self.map_center_lon = sample["longitude"]
                self.map_initialized_from_gps = True

            self.gps_path.append((sample["latitude"], sample["longitude"]))
            self.gps_status_var.set("satellite connected")
            if self.gps_status_label is not None:
                self.gps_status_label.configure(fg="#22c55e")
        else:
            self.gps_status_var.set("satellite not connected")
            if self.gps_status_label is not None:
                self.gps_status_label.configure(fg="#ef4444")

        values = {
            "samples": len(self.samples),
            "time_s": sample["time_s"],
            "ax": sample["ax"],
            "ay": sample["ay"],
            "az": sample["az"],
            "gx": sample["gx"],
            "gy": sample["gy"],
            "gz": sample["gz"],
            "temperature_c": sample["temperature_c"],
            "pressure_hpa": sample["pressure_hpa"],
            "altitude_m": sample["altitude_m"],
            "gps_altitude_m": sample["gps_altitude_m"],
            "latitude": sample["latitude"],
            "longitude": sample["longitude"],
            "connected_satellites": sample["gps_satellites"],
            "roll_deg": math.degrees(forward),
            "pitch_deg": math.degrees(side),
            "yaw_deg": math.degrees(spin),
            "simulation": self.rocket_estimator.status,
        }
        self.update_live_values(values)

    def handle_static_sample(self, sample: dict) -> None:
        if self.static_time_offset is None:
            self.static_time_offset = sample["time_s"]

        sample = sample.copy()
        sample["time_s"] = sample["time_s"] - self.static_time_offset
        self.samples.append(sample)

        metrics = self.calculate_static_metrics()
        values = {
            "samples": len(self.samples),
            "time_s": sample["time_s"],
            "mass_g": sample["mass_g"],
            "thrust_n": sample["thrust_n"],
            "max_thrust_n": metrics["max_thrust_n"],
            "burn_time_s": metrics["burn_time_s"],
            "total_impulse_ns": metrics["total_impulse_ns"],
        }
        self.update_live_values(values)

    def update_live_values(self, values: dict) -> None:
        for key, value in values.items():
            if key not in self.live_values:
                continue
            if value is None:
                if key == "gps_altitude_m":
                    self.live_values[key].set("waiting")
                else:
                    self.live_values[key].set("-")
            elif key in ("latitude", "longitude"):
                self.live_values[key].set(f"{value:.4f}")
            elif isinstance(value, float):
                self.live_values[key].set(f"{value:.3f}")
            else:
                self.live_values[key].set(str(value))

    def update_graph(self) -> None:
        if not self.samples:
            return

        visible_samples = self.samples[-MAX_VISIBLE_POINTS:]

        if self.mode_var.get() == MODE_TELEMETRY:
            x_data = [item["time_s"] for item in visible_samples]
            for key, line in self.lines.items():
                line.set_data(x_data, [self.graph_value(item.get(key)) for item in visible_samples])
            for axis in self.axes:
                axis.relim()
                axis.autoscale_view()
        else:
            x_data = [item["time_s"] for item in visible_samples]
            y_data = [item["thrust_n"] for item in visible_samples]
            self.lines["thrust_n"].set_data(x_data, y_data)
            self.axes[0].relim()
            self.axes[0].autoscale_view()

        self.canvas.draw_idle()

    @staticmethod
    def graph_value(value):
        if value is None:
            return math.nan
        return value

    def update_map_plot(self) -> None:
        if not hasattr(self, "gps_route_line"):
            return

        if not self.gps_path:
            self.gps_route_line.set_data([], [])
            self.gps_current_point.set_data([], [])
            self.clear_map_background()
            lon_span, lat_span = self.map_spans_for_zoom(self.map_center_lat, self.map_zoom_level)
            self.set_map_view(
                self.map_center_lat,
                self.map_center_lon,
                lat_span,
                lon_span,
            )
            self.map_text.set_visible(True)
            self.map_text.set_text(f"{self.map_center_lat:.6f}, {self.map_center_lon:.6f}\nWaiting for GPS fix")
            self.map_text.set_color("#ef4444")
            self.update_map_info()
            self.map_canvas.draw_idle()
            return

        latitudes = [point[0] for point in self.gps_path]
        longitudes = [point[1] for point in self.gps_path]

        self.gps_route_line.set_data(longitudes, latitudes)
        self.gps_current_point.set_data([longitudes[-1]], [latitudes[-1]])
        self.map_text.set_visible(False)

        latest_lat = latitudes[-1]
        latest_lon = longitudes[-1]
        center_distance = gps_distance_m(self.map_center_lat, self.map_center_lon, latest_lat, latest_lon)

        if center_distance >= MAP_CENTER_UPDATE_THRESHOLD_M:
            self.map_center_lat = latest_lat
            self.map_center_lon = latest_lon

        lon_span, lat_span = self.map_spans_for_zoom(self.map_center_lat, self.map_zoom_level)
        self.set_map_view(self.map_center_lat, self.map_center_lon, lat_span, lon_span)

        lon_min, lon_max = self.map_ax.get_xlim()
        lat_min, lat_max = self.map_ax.get_ylim()
        self.request_map_background(lat_min, lat_max, lon_min, lon_max)
        self.update_map_info()
        self.map_canvas.draw_idle()

    def set_map_view(self, center_lat: float, center_lon: float, lat_span: float, lon_span: float) -> None:
        lat_span = max(lat_span, 0.00005)
        lon_span = max(lon_span, 0.00005)
        self.map_ax.set_xlim(center_lon - lon_span / 2.0, center_lon + lon_span / 2.0)
        self.map_ax.set_ylim(center_lat - lat_span / 2.0, center_lat + lat_span / 2.0)

    def map_spans_for_zoom(self, center_lat: float, zoom: int) -> tuple[float, float]:
        widget_width = max(self.map_widget.winfo_width(), OSM_TILE_SIZE_PX)
        widget_height = max(self.map_widget.winfo_height(), OSM_TILE_SIZE_PX)
        visible_tiles_x = max(widget_width / OSM_TILE_SIZE_PX, 1.0)
        visible_tiles_y = max(widget_height / OSM_TILE_SIZE_PX, 1.0)

        tile_lon_span = 360.0 / (2 ** zoom)
        lat_scale = max(math.cos(math.radians(center_lat)), 0.2)
        lon_span = max(tile_lon_span * visible_tiles_x, MIN_MAP_VIEW_SPAN_DEG)
        lat_span = max(tile_lon_span * lat_scale * visible_tiles_y, MIN_MAP_VIEW_SPAN_DEG)
        return lon_span, lat_span

    def update_map_info(self) -> None:
        if not hasattr(self, "map_ax"):
            return

        lon_min, lon_max = self.map_ax.get_xlim()
        lat_min, lat_max = self.map_ax.get_ylim()
        center_lat = (lat_min + lat_max) / 2.0
        center_lon = (lon_min + lon_max) / 2.0

        self.map_info_var.set(f"center: {center_lat:.5f}, {center_lon:.5f} | zoom: {self.map_zoom_level}")

    def on_map_scroll(self, event) -> None:
        if event.inaxes != self.map_ax:
            return

        zoom_delta = 1 if event.button == "up" else -1
        new_zoom = max(MIN_OSM_ZOOM, min(MAX_OSM_ZOOM, self.map_zoom_level + zoom_delta))

        if new_zoom == self.map_zoom_level:
            self.update_map_info()
            return

        x_min, x_max = self.map_ax.get_xlim()
        y_min, y_max = self.map_ax.get_ylim()
        center_lon = (x_min + x_max) / 2.0
        center_lat = (y_min + y_max) / 2.0
        new_width, new_height = self.map_spans_for_zoom(center_lat, new_zoom)

        self.map_zoom_level = new_zoom
        self.set_map_view(center_lat, center_lon, new_height, new_width)

        lon_min, lon_max = self.map_ax.get_xlim()
        lat_min, lat_max = self.map_ax.get_ylim()
        self.request_map_background(lat_min, lat_max, lon_min, lon_max, force=True)
        self.update_map_info()
        self.map_canvas.draw_idle()

    def clear_map_background(self) -> None:
        for artist in self.map_image_artists:
            artist.remove()
        self.map_image_artists = []
        self.map_ax.grid(False)

    def clear_map_tile_queues(self) -> None:
        for tile_queue in (self.map_tile_request_queue, self.map_tile_response_queue):
            while not tile_queue.empty():
                try:
                    tile_queue.get_nowait()
                except queue.Empty:
                    break

    def request_map_background(
        self,
        lat_min: float,
        lat_max: float,
        lon_min: float,
        lon_max: float,
        force: bool = False,
    ) -> None:
        if not self.map_tiles_enabled:
            self.map_ax.grid(True, color="#303642")
            return

        request = self.build_tile_request(lat_min, lat_max, lon_min, lon_max)
        if request is None:
            self.map_ax.grid(True, color="#303642")
            return

        request_key = request["key"]
        now = time.time()

        if request_key == self.map_current_tile_key or request_key == self.map_pending_tile_key:
            return

        if not force and now - self.map_last_tile_request_time < MAP_TILE_THROTTLE_SECONDS:
            return

        self.map_pending_tile_key = request_key
        self.map_last_tile_request_time = now

        while not self.map_tile_request_queue.empty():
            try:
                self.map_tile_request_queue.get_nowait()
            except queue.Empty:
                break

        self.map_tile_request_queue.put(request)

    def apply_map_background(self, response: dict) -> None:
        if response["key"] != self.map_pending_tile_key:
            return

        for artist in self.map_image_artists:
            artist.remove()
        self.map_image_artists = []

        tiles = response["tiles"]

        if not tiles:
            self.map_ax.grid(True, color="#303642")
            self.map_pending_tile_key = None
            return

        for image, extent in tiles:
            artist = self.map_ax.imshow(
                image,
                extent=extent,
                origin="upper",
                alpha=1.0,
                zorder=0,
                interpolation="nearest",
                aspect="auto",
            )
            self.map_image_artists.append(artist)

        self.map_ax.grid(False)
        self.gps_route_line.set_zorder(3)
        self.gps_current_point.set_zorder(4)
        self.map_current_tile_key = response["key"]
        self.map_pending_tile_key = None
        self.map_canvas.draw_idle()

    def process_map_tile_responses(self) -> None:
        latest_response = None

        while True:
            try:
                latest_response = self.map_tile_response_queue.get_nowait()
            except queue.Empty:
                break

        if latest_response is not None:
            self.apply_map_background(latest_response)

    def map_tile_worker(self) -> None:
        while True:
            request = self.map_tile_request_queue.get()
            tiles = []

            for tile_x, tile_y, zoom in request["tiles"]:
                tile = self.get_osm_tile(tile_x, tile_y, zoom)
                if tile is not None:
                    tiles.append(tile)

            self.map_tile_response_queue.put({
                "key": request["key"],
                "tiles": tiles,
            })

    def build_tile_request(self, lat_min: float, lat_max: float, lon_min: float, lon_max: float) -> Optional[dict]:
        zoom = self.map_zoom_level
        north_west_x, north_west_y = self.lat_lon_to_tile(lat_max, lon_min, zoom)
        south_east_x, south_east_y = self.lat_lon_to_tile(lat_min, lon_max, zoom)
        x_start = min(north_west_x, south_east_x)
        x_end = max(north_west_x, south_east_x)
        y_start = min(north_west_y, south_east_y)
        y_end = max(north_west_y, south_east_y)
        tile_count = (x_end - x_start + 1) * (y_end - y_start + 1)

        if tile_count > MAX_MAP_TILES:
            return None

        tiles = [
            (tile_x, tile_y, zoom)
            for tile_x in range(x_start, x_end + 1)
            for tile_y in range(y_start, y_end + 1)
        ]

        if not tiles:
            return None

        return {
            "key": (
                zoom,
                x_start,
                x_end,
                y_start,
                y_end,
            ),
            "tiles": tiles,
        }

    def get_osm_tile(self, tile_x: int, tile_y: int, zoom: int):
        cache_key = (zoom, tile_x, tile_y)

        if cache_key in self.map_tile_cache:
            return self.map_tile_cache[cache_key]

        url = f"https://tile.openstreetmap.org/{zoom}/{tile_x}/{tile_y}.png"

        try:
            request = urllib.request.Request(url, headers={"User-Agent": "thrust-calc-panel/1.0"})
            with urllib.request.urlopen(request, timeout=3.0) as response:
                image_data = response.read()
        except (urllib.error.URLError, TimeoutError, OSError):
            return None

        image = mpimg.imread(io.BytesIO(image_data), format="png")
        north, west = self.tile_to_lat_lon(tile_x, tile_y, zoom)
        south, east = self.tile_to_lat_lon(tile_x + 1, tile_y + 1, zoom)
        extent = [west, east, south, north]
        tile = (image, extent)
        self.map_tile_cache[cache_key] = tile
        return tile

    @staticmethod
    def lat_lon_to_tile(latitude: float, longitude: float, zoom: int) -> tuple[int, int]:
        latitude = max(min(latitude, 85.05112878), -85.05112878)
        longitude = max(min(longitude, 180.0), -180.0)
        lat_rad = math.radians(latitude)
        tile_count = 2 ** zoom
        tile_x = int((longitude + 180.0) / 360.0 * tile_count)
        tile_y = int((1.0 - math.log(math.tan(lat_rad) + 1.0 / math.cos(lat_rad)) / math.pi) / 2.0 * tile_count)
        tile_x = max(0, min(tile_count - 1, tile_x))
        tile_y = max(0, min(tile_count - 1, tile_y))
        return tile_x, tile_y

    @staticmethod
    def tile_to_lat_lon(tile_x: int, tile_y: int, zoom: int) -> tuple[float, float]:
        tile_count = 2 ** zoom
        longitude = tile_x / tile_count * 360.0 - 180.0
        lat_rad = math.atan(math.sinh(math.pi * (1.0 - 2.0 * tile_y / tile_count)))
        latitude = math.degrees(lat_rad)
        return latitude, longitude

    def update_simulation_plot(self) -> None:
        side, forward, spin = self.orientation
        body_start, body_end, nose_end, fin_a_start, fin_a_end, fin_b_start, fin_b_end = self.compute_rocket_segments(
            side,
            forward,
            spin,
        )

        self.rocket_line.set_data([body_start[0], body_end[0]], [body_start[1], body_end[1]])
        self.rocket_line.set_3d_properties([body_start[2], body_end[2]])
        self.nose_line.set_data([body_end[0], nose_end[0]], [body_end[1], nose_end[1]])
        self.nose_line.set_3d_properties([body_end[2], nose_end[2]])
        self.fin_line_a.set_data([fin_a_start[0], fin_a_end[0]], [fin_a_start[1], fin_a_end[1]])
        self.fin_line_a.set_3d_properties([fin_a_start[2], fin_a_end[2]])
        self.fin_line_b.set_data([fin_b_start[0], fin_b_end[0]], [fin_b_start[1], fin_b_end[1]])
        self.fin_line_b.set_3d_properties([fin_b_start[2], fin_b_end[2]])
        self.sim_text.set_text(self.rocket_estimator.status)
        self.sim_canvas.draw_idle()

    def compute_rocket_segments(self, side: float, forward: float, spin: float):
        matrix = self.rotation_y(side)
        matrix = self.matrix_multiply(matrix, self.rotation_x(forward))
        matrix = self.matrix_multiply(matrix, self.rotation_z(spin))

        body_axis = self.apply_matrix(matrix, (0.0, 0.0, 1.0))
        x_axis = self.apply_matrix(matrix, (1.0, 0.0, 0.0))
        y_axis = self.apply_matrix(matrix, (0.0, 1.0, 0.0))

        body_start = self.scale(body_axis, -1.6)
        body_end = self.scale(body_axis, 1.6)
        nose_end = self.scale(body_axis, 2.25)

        fin_center = self.scale(body_axis, -1.25)
        fin_a_start = self.add(fin_center, self.scale(x_axis, -0.65))
        fin_a_end = self.add(fin_center, self.scale(x_axis, 0.65))
        fin_b_start = self.add(fin_center, self.scale(y_axis, -0.65))
        fin_b_end = self.add(fin_center, self.scale(y_axis, 0.65))

        return body_start, body_end, nose_end, fin_a_start, fin_a_end, fin_b_start, fin_b_end

    @staticmethod
    def rotation_x(angle: float):
        c = math.cos(angle)
        s = math.sin(angle)
        return ((1, 0, 0), (0, c, -s), (0, s, c))

    @staticmethod
    def rotation_y(angle: float):
        c = math.cos(angle)
        s = math.sin(angle)
        return ((c, 0, s), (0, 1, 0), (-s, 0, c))

    @staticmethod
    def rotation_z(angle: float):
        c = math.cos(angle)
        s = math.sin(angle)
        return ((c, -s, 0), (s, c, 0), (0, 0, 1))

    @staticmethod
    def matrix_multiply(left, right):
        return tuple(
            tuple(sum(left[row][i] * right[i][col] for i in range(3)) for col in range(3))
            for row in range(3)
        )

    @staticmethod
    def apply_matrix(matrix, vector):
        return tuple(sum(matrix[row][col] * vector[col] for col in range(3)) for row in range(3))

    @staticmethod
    def scale(vector, factor: float):
        return tuple(item * factor for item in vector)

    @staticmethod
    def add(left, right):
        return tuple(left[index] + right[index] for index in range(3))

    def calculate_static_metrics(self) -> dict:
        if len(self.samples) < 2:
            max_thrust = self.samples[0]["thrust_n"] if self.samples else 0.0
            return {"max_thrust_n": max_thrust, "burn_time_s": 0.0, "total_impulse_ns": 0.0}

        max_thrust = max(item["thrust_n"] for item in self.samples)
        burn_time = self.samples[-1]["time_s"] - self.samples[0]["time_s"]
        total_impulse = 0.0

        for index in range(1, len(self.samples)):
            previous = self.samples[index - 1]
            current = self.samples[index]
            delta_time = current["time_s"] - previous["time_s"]
            average_force = (current["thrust_n"] + previous["thrust_n"]) / 2.0
            total_impulse += average_force * delta_time

        return {
            "max_thrust_n": max_thrust,
            "burn_time_s": burn_time,
            "total_impulse_ns": total_impulse,
        }

    def save_current_session(self) -> None:
        if not self.samples:
            self.status_var.set("No data to save")
            return

        timestamp = make_timestamp()

        if self.mode_var.get() == MODE_TELEMETRY:
            csv_path = TELEMETRY_DIR / f"{timestamp}_telemetry.csv"
            plot_path = PLOTS_DIR / f"{timestamp}_telemetry.png"
            fieldnames = [
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
                "latitude",
                "longitude",
                "gps_satellites",
                "gps_altitude_m",
                "gps_connected",
            ]
            self.write_csv(csv_path, fieldnames, self.samples)
        else:
            raw_path = RAW_DIR / f"{timestamp}_raw.csv"
            processed_path = PROCESSED_DIR / f"{timestamp}_processed.csv"
            plot_path = PLOTS_DIR / f"{timestamp}_thrust.png"
            self.write_csv(raw_path, ["time_ms", "mass_g"], self.samples)
            self.write_csv(processed_path, ["time_s", "mass_g", "thrust_n"], self.samples)
            csv_path = processed_path

        self.figure.savefig(plot_path, dpi=150, facecolor=self.figure.get_facecolor(), bbox_inches="tight")
        self.status_var.set(f"Saved {csv_path.name} and {plot_path.name}")

    @staticmethod
    def write_csv(path: Path, fieldnames: list[str], rows: list[dict]) -> None:
        with open(path, "w", newline="", encoding="utf-8") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            writer.writeheader()
            for row in rows:
                writer.writerow({field: "" if row.get(field) is None else row.get(field, "") for field in fieldnames})

    def toggle_fullscreen(self, _event=None) -> None:
        self.fullscreen = not self.fullscreen
        self.root.attributes("-fullscreen", self.fullscreen)

    def exit_fullscreen(self, _event=None) -> None:
        self.fullscreen = False
        self.root.attributes("-fullscreen", False)

    def on_close(self) -> None:
        if self.reader is not None:
            self.reader.stop()
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def run() -> None:
    app = RocketPanelApp()
    app.run()
