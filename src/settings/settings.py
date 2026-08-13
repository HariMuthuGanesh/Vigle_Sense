"""
settings.py — Persist and restore user preferences for Vigil_Sense.

Saves the operator's settings to ``settings.json`` in the project root.
"""

import json
import os
from logger import get_logger

log = get_logger(__name__)

# Settings file path resolution - resolves to project root when nested
_my_dir = os.path.dirname(os.path.abspath(__file__))
if os.path.basename(_my_dir) == "settings":
    _parent = os.path.dirname(_my_dir)
    if os.path.basename(_parent) == "src":
        _root_dir = os.path.dirname(_parent)
    else:
        _root_dir = _parent
else:
    _root_dir = _my_dir

_SETTINGS_FILE = os.path.join(_root_dir, "settings.json")

# ── Defaults (mirror the hard-coded values in Vigil_Sense.py) ───
_DEFAULTS: dict = {
    # Serial ports
    "cli_port":  "",
    "data_port": "",

    # Zone radii (metres)
    "safe_min":       0.0,
    "safe_max":       3.0,
    "alert_min":      3.0,
    "alert_max":      5.0,
    "restricted_min": 5.0,
    "restricted_max": 7.5,

    # Chamber A bounds (metres)
    "chamber_a_x0": -4.5,
    "chamber_a_x1": -2.0,
    "chamber_a_y0":  2.5,
    "chamber_a_y1":  5.0,

    # Chamber B bounds (metres)
    "chamber_b_x0":  2.0,
    "chamber_b_x1":  4.5,
    "chamber_b_y0":  2.5,
    "chamber_b_y1":  5.0,

    # Machine proximity radius (metres)
    "machine_prox_radius": 1.0,

    # UI preferences
    "alerts_muted": False,
}


class Settings:
    """
    Thin wrapper around a dictionary backed by a JSON file.

    All keys are defined in ``_DEFAULTS``; unknown keys are ignored.
    """

    def __init__(self, filepath: str = _SETTINGS_FILE):
        self._filepath = filepath
        self._data: dict = dict(_DEFAULTS)

    # ── Persistence ──────────────────────────────────────────────
    def load(self) -> bool:
        """Load settings from disk."""
        if not os.path.exists(self._filepath):
            log.debug("Settings file not found — using defaults: %s", self._filepath)
            return False
        try:
            with open(self._filepath, "r", encoding="utf-8") as fh:
                on_disk = json.load(fh)
            # Only accept keys that exist in _DEFAULTS (safe merge)
            for k, v in on_disk.items():
                if k in _DEFAULTS:
                    self._data[k] = v
            log.info("Settings loaded from %s", self._filepath)
            return True
        except (json.JSONDecodeError, OSError) as exc:
            log.warning("Could not load settings (%s) — using defaults", exc)
            return False

    def save(self) -> bool:
        """Write current settings to disk."""
        try:
            with open(self._filepath, "w", encoding="utf-8") as fh:
                json.dump(self._data, fh, indent=2)
            log.info("Settings saved to %s", self._filepath)
            return True
        except OSError as exc:
            log.error("Could not save settings: %s", exc)
            return False

    # ── Access ───────────────────────────────────────────────────
    def get(self, key: str, default=None):
        """Return the value for *key*, or *default* if not present."""
        return self._data.get(key, default)

    def set(self, key: str, value) -> None:
        """Set *key* to *value*."""
        self._data[key] = value

    def as_dict(self) -> dict:
        """Return a copy of the settings dictionary."""
        return dict(self._data)

    def reset_to_defaults(self) -> None:
        """Restore all values to defaults."""
        self._data = dict(_DEFAULTS)
        log.info("Settings reset to defaults")

    # ── Context manager support ──────────────────────────────────
    def __enter__(self):
        self.load()
        return self

    def __exit__(self, *_):
        self.save() 
