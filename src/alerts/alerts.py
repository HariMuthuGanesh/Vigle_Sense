"""
alerts.py — Cross-platform audio alerts for Vigil_Sense.

Plays a short beep when a safety event is first detected (rising
edge only — not every frame — to avoid alarm fatigue).
"""

import platform
import threading
from enum import Enum, auto
from logger import get_logger

log = get_logger(__name__)

_SYSTEM = platform.system()


class AlertEvent(Enum):
    CHAMBER_BREACH  = auto()
    MACHINE_PROX    = auto()
    ZONE_RESTRICTED = auto()


# ── Platform-specific beep implementations ───────────────────────

def _beep_windows(frequency: int = 880, duration_ms: int = 180):
    """Use winsound.Beep — synchronous but runs in a thread."""
    try:
        import winsound
        winsound.Beep(frequency, duration_ms)
    except Exception as exc:
        log.debug("winsound.Beep failed: %s", exc)


def _beep_macos():
    """Play the system alert sound via afplay."""
    import subprocess
    try:
        subprocess.Popen(
            ["afplay", "/System/Library/Sounds/Sosumi.aiff"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except Exception as exc:
        log.debug("afplay failed: %s", exc)


def _beep_linux():
    """Try aplay with a short WAV, fall back to terminal bell."""
    import subprocess
    try:
        # Most Linux distros have 'paplay' or 'aplay'
        subprocess.Popen(
            ["paplay", "/usr/share/sounds/freedesktop/stereo/bell.oga"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except Exception:
        try:
            print("\a", end="", flush=True)  # terminal bell fallback
        except Exception as exc:
            log.debug("Linux beep fallback failed: %s", exc)


def _play_beep(event: AlertEvent):
    """Dispatch to the correct OS beep, non-blocking via a daemon thread."""
    freq = {
        AlertEvent.CHAMBER_BREACH:  1200,
        AlertEvent.MACHINE_PROX:    1800,   # higher pitch = more urgent
        AlertEvent.ZONE_RESTRICTED: 880,
    }.get(event, 880)

    def _run():
        if _SYSTEM == "Windows":
            _beep_windows(freq, 220)
        elif _SYSTEM == "Darwin":
            _beep_macos()
        else:
            _beep_linux()

    t = threading.Thread(target=_run, daemon=True)
    t.start()


# ── AlertManager ─────────────────────────────────────────────────

class AlertManager:
    """
    Tracks safety event states across frames and fires a beep on
    each rising edge (False → True transition) only.

    Attributes:
        muted (bool): When True, beeps are suppressed but internal
                      state tracking still happens.
    """

    def __init__(self):
        self.muted = False

        # Previous-frame state for edge detection
        self._prev: dict[str, bool] = {
            "ch_a":       False,
            "ch_b":       False,
            "mach_a":     False,
            "mach_b":     False,
            "restricted": False,
        }

    def on_frame(
        self,
        ch_a_breach:    bool,
        ch_b_breach:    bool,
        mach_a_prox:    bool,
        mach_b_prox:    bool,
        any_restricted: bool,
    ):
        """
        Call once per radar frame with current safety flags.
        Fires beep only on the first frame where a flag goes True.
        """
        cur = {
            "ch_a":       ch_a_breach,
            "ch_b":       ch_b_breach,
            "mach_a":     mach_a_prox,
            "mach_b":     mach_b_prox,
            "restricted": any_restricted,
        }

        # Define which event each key maps to
        event_map = {
            "ch_a":       AlertEvent.CHAMBER_BREACH,
            "ch_b":       AlertEvent.CHAMBER_BREACH,
            "mach_a":     AlertEvent.MACHINE_PROX,
            "mach_b":     AlertEvent.MACHINE_PROX,
            "restricted": AlertEvent.ZONE_RESTRICTED,
        }

        for key, state in cur.items():
            rising_edge = state and not self._prev[key]
            if rising_edge:
                ev = event_map[key]
                log.info("ALERT [%s] → %s", key.upper(), ev.name)
                if not self.muted:
                    _play_beep(ev)

        self._prev = cur

    def reset(self):
        """Reset all tracked states (call on sensor stop/disconnect)."""
        for k in self._prev:
            self._prev[k] = False 
