"""
logger.py — Structured logging for Vigil_Sense.

Sets up a rotating file handler so every session's events are
persisted to disk, in addition to the UI console output.
"""

import logging
import os
from logging.handlers import RotatingFileHandler

# Log file path resolution - resolves to project root when nested
_my_dir = os.path.dirname(os.path.abspath(__file__))
if os.path.basename(_my_dir) == "logger":
    _parent = os.path.dirname(_my_dir)
    if os.path.basename(_parent) == "src":
        _root_dir = os.path.dirname(_parent)
    else:
        _root_dir = _parent
else:
    _root_dir = _my_dir

_logs_dir = os.path.join(_root_dir, "logs")
os.makedirs(_logs_dir, exist_ok=True)
_LOG_FILE    = os.path.join(_logs_dir, "vigil_sense.log")
_MAX_BYTES   = 5 * 1024 * 1024   # 5 MB per log file
_BACKUP_COUNT = 3                 # keep up to 3 rotated files

# ── Module-level flag so we only configure handlers once ─────────
_configured = False


def _configure():
    """Attach handlers to the root logger exactly once."""
    global _configured
    if _configured:
        return

    root = logging.getLogger()
    root.setLevel(logging.DEBUG)

    # ── 1. Rotating file handler (full DEBUG level) ──────────────
    fh = RotatingFileHandler(
        _LOG_FILE,
        maxBytes=_MAX_BYTES,
        backupCount=_BACKUP_COUNT,
        encoding="utf-8",
    )
    fh.setLevel(logging.DEBUG)
    fh.setFormatter(logging.Formatter(
        fmt="%(asctime)s  %(levelname)-8s  %(name)s  %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    ))
    root.addHandler(fh)

    # ── 2. Console handler (INFO and above) ──────────────────────
    ch = logging.StreamHandler()
    ch.setLevel(logging.INFO)
    ch.setFormatter(logging.Formatter(
        fmt="%(levelname)-8s  %(name)s  %(message)s",
    ))
    root.addHandler(ch)

    _configured = True


def get_logger(name: str) -> logging.Logger:
    """
    Return a named logger.  Calling this function also ensures the
    root logger is configured with file + console handlers.

    Args:
        name: typically ``__name__`` of the calling module.

    Returns:
        A standard :class:`logging.Logger` instance.
    """
    _configure()
    return logging.getLogger(name) 
