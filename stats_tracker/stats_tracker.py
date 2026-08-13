"""
stats_tracker.py — Real-time performance & tracking statistics for Vigil_Sense.
"""

import time
from collections import deque


class StatsTracker:
    """
    Lightweight rolling-window statistics tracker.

    Args:
        fps_window (int): Number of recent frame timestamps to keep
                          for the rolling FPS calculation.  Default 60.
    """

    def __init__(self, fps_window: int = 60):
        self._ts: deque[float] = deque(maxlen=fps_window)
        self.last_points: int = 0
        self.last_tracks: int = 0
        self._session_start: float = time.time()
        self._total_frames:  int   = 0

    # ── Update ───────────────────────────────────────────────────
    def update(self, point_count: int, track_count: int) -> None:
        """
        Record a new frame.

        Args:
            point_count: Number of raw point-cloud points in this frame.
            track_count: Number of confirmed tracker targets.
        """
        now = time.time()
        self._ts.append(now)
        self.last_points = point_count
        self.last_tracks = track_count
        self._total_frames += 1

    # ── Properties ───────────────────────────────────────────────
    @property
    def fps(self) -> float:
        """Rolling frames-per-second based on the last *fps_window* frames."""
        if len(self._ts) < 2:
            return 0.0
        span = self._ts[-1] - self._ts[0]
        if span <= 0:
            return 0.0
        return round((len(self._ts) - 1) / span, 1)

    @property
    def session_duration_s(self) -> float:
        """Elapsed seconds since this StatsTracker was created."""
        return time.time() - self._session_start

    @property
    def total_frames(self) -> int:
        """Total frames received since this StatsTracker was created."""
        return self._total_frames

    # ── Formatted strings (ready for StatCard) ───────────────────
    @property
    def fps_str(self) -> str:
        return f"{self.fps:.1f}"

    @property
    def points_str(self) -> str:
        return str(self.last_points)

    @property
    def tracks_str(self) -> str:
        return str(self.last_tracks)

    # ── Summary ──────────────────────────────────────────────────
    def summary(self) -> dict:
        """Return all stats as a plain dictionary."""
        return {
            "fps":            self.fps,
            "last_points":    self.last_points,
            "last_tracks":    self.last_tracks,
            "total_frames":   self.total_frames,
            "session_s":      round(self.session_duration_s, 1),
        }

    def reset(self) -> None:
        """Clear all accumulated state."""
        self._ts.clear()
        self.last_points = 0
        self.last_tracks = 0
        self._total_frames  = 0
        self._session_start = time.time() 
