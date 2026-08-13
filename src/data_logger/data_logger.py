"""
data_logger.py — Session CSV data logger for Vigil_Sense.

Records confirmed radar frame metrics to a timestamped CSV file.
"""

import csv
import os
import time
import subprocess
import platform
from logger import get_logger

log = get_logger(__name__)


class DataLogger:
    """Writes one CSV row per confirmed radar frame target."""

    # Column names written to the CSV header
    FIELDNAMES = [
        "timestamp",
        "frame_num",
        "target_id",
        "x_m",
        "y_m",
        "z_m",
        "height_m",
        "zone",
        "in_chamber_a",
        "in_chamber_b",
        "mach_a_prox",
        "mach_b_prox",
    ]

    def __init__(self, output_dir: str | None = None):
        """
        Args:
            output_dir: Directory in which to write the CSV.
                        Defaults to the project root.
        """
        if output_dir is None:
            my_dir = os.path.dirname(os.path.abspath(__file__))
            if os.path.basename(my_dir) == "data_logger":
                parent = os.path.dirname(my_dir)
                if os.path.basename(parent) == "src":
                    root_dir = os.path.dirname(parent)
                else:
                    root_dir = parent
            else:
                root_dir = my_dir
            output_dir = os.path.join(root_dir, "data")

        os.makedirs(output_dir, exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S")
        self.filepath = os.path.join(output_dir, f"session_{ts}.csv")
        self._file   = None
        self._writer = None
        self._running = False
        log.debug("DataLogger initialised → %s", self.filepath)

    # ── Lifecycle ────────────────────────────────────────────────
    def start(self):
        """Open the CSV file and write the header row."""
        if self._running:
            return
        self._file   = open(self.filepath, "w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._file, fieldnames=self.FIELDNAMES)
        self._writer.writeheader()
        self._running = True
        log.info("DataLogger started → %s", self.filepath)

    def stop(self):
        """Flush and close the CSV file."""
        if not self._running:
            return
        self._running = False
        if self._file:
            self._file.flush()
            self._file.close()
            self._file   = None
            self._writer = None
        log.info("DataLogger stopped  → %s", self.filepath)

    # ── Writing ──────────────────────────────────────────────────
    def log_frame(
        self,
        frame_num: int,
        targets: list,
        persons: dict,
        zone_counts: dict,
        ch_a_breach: bool,
        ch_b_breach: bool,
        mach_a_prox: bool,
        mach_b_prox: bool,
    ):
        """Write one row per confirmed target."""
        if not self._running or not targets:
            return

        ts = time.strftime("%Y-%m-%d %H:%M:%S")

        for t in targets:
            tid = t["id"]
            per = persons.get(tid)
            ht  = round(per.height, 3) if per else 0.0

            # Determine per-person zone label
            if ch_a_breach and per and _near_chamber(per.x, per.y, "A"):
                zone = "chamber_a"
            elif ch_b_breach and per and _near_chamber(per.x, per.y, "B"):
                zone = "chamber_b"
            else:
                zone = _zone_label(per)

            self._writer.writerow({
                "timestamp":    ts,
                "frame_num":    frame_num,
                "target_id":    tid,
                "x_m":          round(t["x"], 3),
                "y_m":          round(t["y"], 3),
                "z_m":          round(t["z"], 3),
                "height_m":     ht,
                "zone":         zone,
                "in_chamber_a": int(ch_a_breach),
                "in_chamber_b": int(ch_b_breach),
                "mach_a_prox":  int(mach_a_prox),
                "mach_b_prox":  int(mach_b_prox),
            })

        self._file.flush()

    # ── Utility ──────────────────────────────────────────────────
    def open_in_os(self):
        """Open the CSV with the OS default application."""
        if not os.path.exists(self.filepath):
            log.warning("DataLogger.open_in_os: file not found: %s", self.filepath)
            return
        log.info("Opening %s in OS default app", self.filepath)
        try:
            _sys = platform.system()
            if _sys == "Windows":
                os.startfile(self.filepath)                # noqa: S606
            elif _sys == "Darwin":
                subprocess.Popen(["open", self.filepath])  # noqa: S603
            else:
                subprocess.Popen(["xdg-open", self.filepath])  # noqa: S603
        except Exception as exc:
            log.error("Could not open file: %s", exc)

    @property
    def is_running(self) -> bool:
        return self._running


# ── Private helpers ──────────────────────────────────────────────
def _zone_label(person) -> str:
    """Map a PersonState to a zone name string."""
    if person is None:
        return "unknown"
    dist = (person.x ** 2 + person.y ** 2) ** 0.5
    # These thresholds mirror the defaults in config.py
    if dist > 5.0:
        return "restricted"
    if dist > 3.0:
        return "alert"
    return "safe"


def _near_chamber(x: float, y: float, unit: str) -> bool:
    """Rough chamber membership check for zone labelling."""
    if unit == "A":
        return -4.5 <= x <= -2.0 and 2.5 <= y <= 5.0
    if unit == "B":
        return 2.0 <= x <= 4.5 and 2.5 <= y <= 5.0
    return False 
