#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PCAN-USB / PCAN-USB FD viewer for UltrasonicDistanceCmd_t.

CAN message
- CAN ID: 0x200
- CAN FD DLC 12 => 24-byte frame
- Payload uses first 23 bytes

Payload layout, little-endian:
B0~B19  : 10 x uint16 final published ultrasonic values
B20~B21 : int16 imuYaw [-180..180]
B22     : uint8 vehicleSpeed [0.1 km/h]
B23     : padding / unused

Distance value policy, matching App_Ultrasonic:
- 0x0000..0x07D0 : valid filtered distance, 0..2000 mm
- 0xFFFB         : out of range
- 0xFFFC         : not updated yet
- 0xFFFD         : stale
- 0xFFFE         : repeated bad measurement
- 0xFFFF         : hardware/config/error escalation
- 0x07D1..0xFFFA : reserved/unknown
"""

from __future__ import annotations

import math
import queue
import struct
import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Any, Dict, Optional, Tuple

import tkinter as tk
from tkinter import messagebox, ttk

try:
    import can  # type: ignore
except ImportError:
    can = None


# ================================================================
# Default run settings
# ================================================================
PCAN_CHANNEL_DEFAULT = "PCAN_USBBUS1"
CAN_ID_DEFAULT = 0x200
AUTO_CONNECT_ON_START = True

# PCAN CAN FD bit timing preset: nominal 500 kbit/s, data 2 Mbit/s, f_clock 80 MHz.
PCAN_FD_TIMING = {
    "f_clock_mhz": 80,
    "nom_brp": 10,
    "nom_tseg1": 12,
    "nom_tseg2": 3,
    "nom_sjw": 3,
    "data_brp": 5,
    "data_tseg1": 6,
    "data_tseg2": 1,
    "data_sjw": 1,
}

VALID_MAX_MM = 2000
WARN_MM = 700
DANGER_MM = 300

ULTRASONIC_OUT_OF_RANGE = 0xFFFB
ULTRASONIC_NOT_UPDATED = 0xFFFC
ULTRASONIC_STALE = 0xFFFD
ULTRASONIC_BAD_MEASUREMENT = 0xFFFE
ULTRASONIC_ERROR = 0xFFFF

PAYLOAD_FORMAT = "<10HhB"  # 10 uint16, 1 int16, 1 uint8 = 23 bytes
PAYLOAD_SIZE = struct.calcsize(PAYLOAD_FORMAT)

SENSOR_IDS = ("FC", "FR", "RF", "RM", "RR", "BC", "RL", "LM", "LF", "FL")
SENSOR_NAMES = (
    "Front Center",
    "Front Right",
    "Right Front",
    "Right Middle",
    "Rear Right",
    "Back Center",
    "Rear Left",
    "Left Middle",
    "Left Front",
    "Front Left",
)

# Canvas direction vectors in sensor ID order: FC, FR, RF, RM, RR, BC, RL, LM, LF, FL.
SENSOR_DIRS = (
    (0.0, -1.0),
    (0.72, -0.72),
    (1.0, -0.28),
    (1.0, 0.28),
    (0.72, 0.72),
    (0.0, 1.0),
    (-0.72, 0.72),
    (-1.0, 0.28),
    (-1.0, -0.28),
    (-0.72, -0.72),
)


class DistanceStatus(Enum):
    VALID = "valid"
    OUT_OF_RANGE = "out_of_range"
    NOT_UPDATED = "not_updated"
    STALE = "stale"
    BAD_MEASUREMENT = "bad_measurement"
    ERROR = "error"
    RESERVED = "reserved"


STATUS_META = {
    DistanceStatus.VALID: ("VALID", "#34c759"),
    DistanceStatus.OUT_OF_RANGE: ("OUT OF RANGE", "#8e98a8"),
    DistanceStatus.NOT_UPDATED: ("NOT UPDATED", "#6b7280"),
    DistanceStatus.STALE: ("STALE", "#9b7cff"),
    DistanceStatus.BAD_MEASUREMENT: ("BAD MEAS", "#ffb020"),
    DistanceStatus.ERROR: ("ERROR", "#ff4fd8"),
    DistanceStatus.RESERVED: ("RESERVED", "#ff6b6b"),
}


@dataclass(frozen=True)
class SensorValue:
    sensor_id: str
    name: str
    value: int
    status: DistanceStatus
    raw_hex: str

    @property
    def is_valid(self) -> bool:
        return self.status is DistanceStatus.VALID

    @property
    def label(self) -> str:
        if self.is_valid:
            return f"{self.value} mm"
        return STATUS_META[self.status][0]

    @property
    def value_text(self) -> str:
        return f"{self.value} / 0x{self.value:04X}"


@dataclass(frozen=True)
class UltrasonicDistanceCmd:
    values: Tuple[SensorValue, ...]
    imu_yaw_deg: int
    vehicle_speed_kmh: float
    timestamp: float
    raw: bytes

    @property
    def valid_count(self) -> int:
        return sum(1 for item in self.values if item.status is DistanceStatus.VALID)

    @property
    def abnormal_values(self) -> Tuple[SensorValue, ...]:
        return tuple(item for item in self.values if item.status is not DistanceStatus.VALID)

    @property
    def error_values(self) -> Tuple[SensorValue, ...]:
        return tuple(item for item in self.values if item.status is DistanceStatus.ERROR)


def classify_distance(value: int) -> DistanceStatus:
    if 0 <= value <= VALID_MAX_MM:
        return DistanceStatus.VALID
    if value == ULTRASONIC_OUT_OF_RANGE:
        return DistanceStatus.OUT_OF_RANGE
    if value == ULTRASONIC_NOT_UPDATED:
        return DistanceStatus.NOT_UPDATED
    if value == ULTRASONIC_STALE:
        return DistanceStatus.STALE
    if value == ULTRASONIC_BAD_MEASUREMENT:
        return DistanceStatus.BAD_MEASUREMENT
    if value == ULTRASONIC_ERROR:
        return DistanceStatus.ERROR
    return DistanceStatus.RESERVED


def raw_distance_hex(raw: bytes, sensor_index: int) -> str:
    base = sensor_index * 2
    if len(raw) < base + 2:
        return "-- --"
    return f"{raw[base]:02X} {raw[base + 1]:02X}"


def decode_ultrasonic_payload(data: bytes, timestamp: Optional[float] = None) -> UltrasonicDistanceCmd:
    if len(data) < PAYLOAD_SIZE:
        raise ValueError(f"payload too short: {len(data)} bytes, need at least {PAYLOAD_SIZE}")

    unpacked = struct.unpack(PAYLOAD_FORMAT, data[:PAYLOAD_SIZE])
    raw_distances = tuple(int(v) for v in unpacked[:10])
    yaw = int(unpacked[10])
    speed = int(unpacked[11]) * 0.1
    raw = bytes(data)

    values = tuple(
        SensorValue(
            sensor_id=SENSOR_IDS[index],
            name=SENSOR_NAMES[index],
            value=value,
            status=classify_distance(value),
            raw_hex=raw_distance_hex(raw, index),
        )
        for index, value in enumerate(raw_distances)
    )

    return UltrasonicDistanceCmd(
        values=values,
        imu_yaw_deg=yaw,
        vehicle_speed_kmh=speed,
        timestamp=time.time() if timestamp is None else timestamp,
        raw=raw,
    )


def make_pcan_fd_bus(channel: str) -> Any:
    if can is None:
        raise RuntimeError(
            "python-can is not installed.\n"
            "Run: python -m pip install python-can\n"
            "For PCAN on Windows, install PEAK PCAN-Basic and the PCAN driver too."
        )

    return can.Bus(  # type: ignore[union-attr]
        interface="pcan",
        channel=channel,
        fd=True,
        **PCAN_FD_TIMING,
    )


class CanReaderThread(threading.Thread):
    def __init__(self, channel: str, can_id: int, out_queue: queue.Queue, status_queue: queue.Queue):
        super().__init__(daemon=True)
        self.channel = channel
        self.can_id = can_id
        self.out_queue = out_queue
        self.status_queue = status_queue
        self._stop_event = threading.Event()
        self._bus: Optional[Any] = None

    def stop(self) -> None:
        self._stop_event.set()
        if self._bus is not None:
            try:
                self._bus.shutdown()
            except Exception:
                pass

    def run(self) -> None:
        try:
            self._bus = make_pcan_fd_bus(self.channel)
            self.status_queue.put(("connected", f"PCAN connected: {self.channel}, CAN ID 0x{self.can_id:X}"))
        except Exception as exc:
            self.status_queue.put(("error", f"PCAN connect failed: {exc}"))
            return

        while not self._stop_event.is_set():
            try:
                msg = self._bus.recv(timeout=0.1)
            except Exception as exc:
                self.status_queue.put(("error", f"CAN receive error: {exc}"))
                time.sleep(0.5)
                continue

            if msg is None:
                continue
            if msg.arbitration_id != self.can_id:
                continue
            if msg.is_extended_id:
                continue

            try:
                decoded = decode_ultrasonic_payload(bytes(msg.data), timestamp=msg.timestamp)
            except ValueError as exc:
                self.status_queue.put(("warn", f"Ignored frame: {exc}"))
                continue

            self.out_queue.put(decoded)


class UltrasonicViewer(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("Ultrasonic CAN FD Viewer - g_distancesMm")
        self.geometry("1120x780")
        self.minsize(980, 690)

        self.max_mm = VALID_MAX_MM
        self.warn_mm = WARN_MM
        self.danger_mm = DANGER_MM

        self.data_queue: queue.Queue[UltrasonicDistanceCmd] = queue.Queue()
        self.status_queue: queue.Queue[Tuple[str, str]] = queue.Queue()
        self.reader: Optional[CanReaderThread] = None

        self.last_data: Optional[UltrasonicDistanceCmd] = None
        self.last_rx_wall_time = 0.0
        self.rx_count = 0

        self.channel_var = tk.StringVar(value=PCAN_CHANNEL_DEFAULT)
        self.can_id_var = tk.StringVar(value=f"0x{CAN_ID_DEFAULT:X}")

        self._build_ui()
        self.after(50, self._poll_queues)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

        if AUTO_CONNECT_ON_START:
            self.after(200, self.connect_can)

    def _build_ui(self) -> None:
        main = ttk.Frame(self, padding=12)
        main.pack(fill=tk.BOTH, expand=True)

        controls = ttk.Frame(main)
        controls.pack(fill=tk.X)

        ttk.Label(controls, text="PCAN Channel").pack(side=tk.LEFT)
        ttk.Entry(controls, textvariable=self.channel_var, width=18).pack(side=tk.LEFT, padx=(6, 14))

        ttk.Label(controls, text="CAN ID").pack(side=tk.LEFT)
        ttk.Entry(controls, textvariable=self.can_id_var, width=10).pack(side=tk.LEFT, padx=(6, 14))

        self.connect_button = ttk.Button(controls, text="Connect / Reconnect", command=self.connect_can)
        self.connect_button.pack(side=tk.LEFT)
        ttk.Button(controls, text="Disconnect", command=self.disconnect_can).pack(side=tk.LEFT, padx=(6, 0))

        self.meta_var = tk.StringVar(value="RX: 0 | Yaw: -- deg | Speed: -- km/h | Age: -- ms | Valid: 0/10")
        ttk.Label(controls, textvariable=self.meta_var, font=("Segoe UI", 10)).pack(side=tk.RIGHT)

        status_row = ttk.Frame(main)
        status_row.pack(fill=tk.X, pady=(8, 0))
        self.status_var = tk.StringVar(value="Ready. Waiting for CAN FD frame ID 0x200.")
        ttk.Label(status_row, textvariable=self.status_var, font=("Segoe UI", 11, "bold")).pack(side=tk.LEFT)

        body = ttk.Frame(main)
        body.pack(fill=tk.BOTH, expand=True, pady=(10, 0))

        self.canvas = tk.Canvas(body, bg="#11151c", highlightthickness=0)
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        side = ttk.Frame(body, width=340)
        side.pack(side=tk.RIGHT, fill=tk.Y, padx=(12, 0))
        side.pack_propagate(False)

        ttk.Label(side, text="Sensor Values", font=("Segoe UI", 12, "bold")).pack(anchor="w", pady=(0, 8))
        self.sensor_vars: Dict[str, tk.StringVar] = {}
        for sensor_id, name in zip(SENSOR_IDS, SENSOR_NAMES):
            key = f"{sensor_id} {name}"
            var = tk.StringVar(value=f"{sensor_id:<2} {name:<13}: ----")
            self.sensor_vars[key] = var
            ttk.Label(side, textvariable=var, font=("Consolas", 9)).pack(anchor="w", pady=2)

        ttk.Separator(side).pack(fill=tk.X, pady=12)
        ttk.Label(side, text="Status Codes", font=("Segoe UI", 11, "bold")).pack(anchor="w")
        ttk.Label(
            side,
            text=(
                "0000..07D0 : valid 0..2000 mm\n"
                "FF FB      : OUT_OF_RANGE\n"
                "FF FC      : NOT_UPDATED\n"
                "FF FD      : STALE\n"
                "FF FE      : BAD_MEASUREMENT\n"
                "FF FF      : ERROR"
            ),
            font=("Consolas", 9),
        ).pack(anchor="w", pady=(4, 0))

        ttk.Separator(side).pack(fill=tk.X, pady=12)
        ttk.Label(side, text="Abnormal Sensors", font=("Segoe UI", 11, "bold")).pack(anchor="w")
        self.abnormal_list_var = tk.StringVar(value="None")
        ttk.Label(side, textvariable=self.abnormal_list_var, font=("Consolas", 9), wraplength=320).pack(
            anchor="w",
            pady=(4, 0),
        )

        ttk.Separator(side).pack(fill=tk.X, pady=12)
        ttk.Label(side, text="Payload", font=("Segoe UI", 11, "bold")).pack(anchor="w")
        self.raw_var = tk.StringVar(value="--")
        ttk.Label(side, textvariable=self.raw_var, font=("Consolas", 9), wraplength=320).pack(anchor="w", pady=(4, 0))

        ttk.Separator(side).pack(fill=tk.X, pady=12)
        ttk.Label(side, text="Bit Timing", font=("Segoe UI", 11, "bold")).pack(anchor="w")
        ttk.Label(side, text="Nominal 500 kbit/s\nData 2 Mbit/s\nClock 80 MHz", font=("Consolas", 9)).pack(
            anchor="w",
            pady=(4, 0),
        )

        self.canvas.bind("<Configure>", lambda _event: self._redraw())

    def connect_can(self) -> None:
        self.disconnect_can(show_status=False)

        try:
            can_id = int(self.can_id_var.get().strip(), 0)
        except ValueError:
            messagebox.showerror("CAN ID error", "Enter a numeric CAN ID, for example 0x200 or 512.")
            return

        channel = self.channel_var.get().strip() or PCAN_CHANNEL_DEFAULT
        self.data_queue = queue.Queue()
        self.status_queue = queue.Queue()
        self.reader = CanReaderThread(channel, can_id, self.data_queue, self.status_queue)
        self.reader.start()
        self.status_var.set(f"Connecting {channel}, ID 0x{can_id:X} ...")

    def disconnect_can(self, show_status: bool = True) -> None:
        if self.reader is not None:
            self.reader.stop()
            self.reader = None
        if show_status:
            self.status_var.set("Disconnected")

    def _poll_queues(self) -> None:
        while True:
            try:
                kind, text = self.status_queue.get_nowait()
            except queue.Empty:
                break
            prefix = {"connected": "OK", "warn": "WARN", "error": "ERROR"}.get(kind, "INFO")
            self.status_var.set(f"{prefix}: {text}")

        updated = False
        while True:
            try:
                self.last_data = self.data_queue.get_nowait()
            except queue.Empty:
                break
            self.rx_count += 1
            self.last_rx_wall_time = time.time()
            updated = True

        if updated:
            self._update_labels()
            self._redraw()
        else:
            self._update_meta_only()

        self.after(50, self._poll_queues)

    def _update_labels(self) -> None:
        if self.last_data is None:
            return

        for item in self.last_data.values:
            key = f"{item.sensor_id} {item.name}"
            if item.is_valid:
                cm = item.value / 10.0
                text = f"{item.sensor_id:<2} {item.name:<13}: {item.value:5d} mm ({cm:6.1f} cm)  {item.raw_hex}"
            else:
                text = f"{item.sensor_id:<2} {item.name:<13}: {item.label:<15} {item.value_text:<14} {item.raw_hex}"
            self.sensor_vars[key].set(text)

        abnormal = self.last_data.abnormal_values
        if not abnormal:
            self.abnormal_list_var.set("None")
        else:
            self.abnormal_list_var.set(
                ", ".join(f"{item.sensor_id}:{item.label}({item.value_text})" for item in abnormal)
            )

        self.raw_var.set(" ".join(f"{byte:02X}" for byte in self.last_data.raw[:24]))
        self._update_meta_only()

    def _update_meta_only(self) -> None:
        if self.last_data is None:
            return

        age_ms = int((time.time() - self.last_rx_wall_time) * 1000) if self.last_rx_wall_time else 0
        data = self.last_data
        self.meta_var.set(
            f"RX: {self.rx_count} | Yaw: {data.imu_yaw_deg:+d} deg | "
            f"Speed: {data.vehicle_speed_kmh:.1f} km/h | Age: {age_ms} ms | "
            f"Valid: {data.valid_count}/10 | Error: {len(data.error_values)}"
        )

    def _color_for_value(self, item: SensorValue) -> str:
        if item.status is not DistanceStatus.VALID:
            return STATUS_META[item.status][1]
        if item.value < self.danger_mm:
            return "#ff3b30"
        if item.value < self.warn_mm:
            return "#ffb020"
        return "#34c759"

    def _distance_radius(self, item: SensorValue, min_r: float, max_r: float) -> float:
        if item.status is DistanceStatus.OUT_OF_RANGE:
            return max_r
        if item.status is not DistanceStatus.VALID:
            return min_r

        clipped = max(0, min(item.value, self.max_mm))
        return min_r + (clipped / self.max_mm) * (max_r - min_r)

    def _redraw(self) -> None:
        canvas = self.canvas
        canvas.delete("all")
        width = max(canvas.winfo_width(), 1)
        height = max(canvas.winfo_height(), 1)
        cx, cy = width / 2, height / 2
        scale = min(width, height)

        for ratio, label in ((0.20, "near"), (0.32, "mid"), (0.44, "2m")):
            radius = scale * ratio
            canvas.create_oval(cx - radius, cy - radius, cx + radius, cy + radius, outline="#2b3340", width=1)
            canvas.create_text(cx + radius + 22, cy, text=label, fill="#566172", font=("Segoe UI", 8))

        car_w, car_h = scale * 0.18, scale * 0.32
        canvas.create_rectangle(
            cx - car_w / 2,
            cy - car_h / 2,
            cx + car_w / 2,
            cy + car_h / 2,
            fill="#d9dee7",
            outline="#ffffff",
            width=2,
        )
        canvas.create_polygon(
            cx,
            cy - car_h / 2 - 18,
            cx - 18,
            cy - car_h / 2 + 10,
            cx + 18,
            cy - car_h / 2 + 10,
            fill="#7aa2ff",
            outline="#ffffff",
        )
        canvas.create_text(cx, cy, text="CAR", fill="#121722", font=("Segoe UI", 15, "bold"))

        if self.last_data is None:
            canvas.create_text(
                cx,
                cy + car_h / 2 + 50,
                text="Waiting for CAN FD frame ID 0x200...",
                fill="#d9dee7",
                font=("Segoe UI", 14, "bold"),
            )
            return

        min_r = scale * 0.18
        max_r = scale * 0.43

        for item, vector in zip(self.last_data.values, SENSOR_DIRS):
            color = self._color_for_value(item)
            radius = self._distance_radius(item, min_r, max_r)
            x = cx + vector[0] * radius
            y = cy + vector[1] * radius

            dash = () if item.status is DistanceStatus.VALID else (6, 5)
            canvas.create_line(cx, cy, x, y, fill=color, width=4, dash=dash)

            dot = 14
            canvas.create_oval(x - dot, y - dot, x + dot, y + dot, fill=color, outline="#f4f7fb", width=2)

            if item.status is DistanceStatus.ERROR:
                canvas.create_line(x - 10, y - 10, x + 10, y + 10, fill="#11151c", width=4)
                canvas.create_line(x + 10, y - 10, x - 10, y + 10, fill="#11151c", width=4)

            label_r = radius + 42
            lx = cx + vector[0] * label_r
            ly = cy + vector[1] * label_r
            canvas.create_text(
                lx,
                ly,
                text=f"{item.sensor_id}\n{item.label}",
                fill="#f4f7fb",
                font=("Segoe UI", 9, "bold"),
                justify=tk.CENTER,
            )

        yaw = math.radians(self.last_data.imu_yaw_deg - 90)
        needle_len = scale * 0.13
        nx = cx + math.cos(yaw) * needle_len
        ny = cy + math.sin(yaw) * needle_len
        canvas.create_line(cx, cy, nx, ny, fill="#00d1ff", width=3, arrow=tk.LAST)
        canvas.create_text(
            cx,
            cy + car_h / 2 + 28,
            text=f"Yaw {self.last_data.imu_yaw_deg:+d} deg   Speed {self.last_data.vehicle_speed_kmh:.1f} km/h",
            fill="#f4f7fb",
            font=("Segoe UI", 12, "bold"),
        )

    def _on_close(self) -> None:
        self.disconnect_can(show_status=False)
        self.destroy()


def main() -> None:
    app = UltrasonicViewer()
    app.mainloop()


if __name__ == "__main__":
    main()
