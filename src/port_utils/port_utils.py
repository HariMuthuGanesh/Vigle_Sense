"""
port_utils.py — Serial port enumeration helpers for Vigil_Sense.
"""

import re
from dataclasses import dataclass
from typing import Optional

import serial.tools.list_ports
from logger import get_logger

log = get_logger(__name__)


@dataclass
class PortInfo:
    """Lightweight wrapper around a serial port entry."""
    device: str         # e.g. "COM3"
    description: str    # e.g. "XDS110 Class Application/User UART"
    display: str        # ready-to-show label for a combo box
    number: int         # numeric part of device name (for sorting)


def _port_number(device: str) -> int:
    """Extract the integer from a COM/ttyUSB name for natural sort order."""
    m = re.search(r"\d+", device)
    return int(m.group()) if m else 0


def _make_display(port) -> str:
    """Build a human-readable combo-box label for a port."""
    desc = port.description or port.device
    if "XDS110" in desc:
        if any(kw in desc for kw in ("App", "User", "UART")):
            return f"{port.device}  [XDS110 CLI]"
        if any(kw in desc for kw in ("Auxiliary", "Data", "Bulk")):
            return f"{port.device}  [XDS110 Data]"
        return f"{port.device}  [{desc[:20]}]"
    return f"{port.device}  [{desc[:25]}]"


def list_ports() -> list[PortInfo]:
    """
    Return all available serial ports sorted by device number (ascending).

    Returns:
        A list of :class:`PortInfo` objects, lowest port number first.
        Returns an empty list if no ports are found.
    """
    raw = serial.tools.list_ports.comports()
    ports = [
        PortInfo(
            device      = p.device,
            description = p.description or "",
            display     = _make_display(p),
            number      = _port_number(p.device),
        )
        for p in raw
    ]
    ports.sort(key=lambda p: p.number)
    log.debug("list_ports: found %d port(s): %s",
              len(ports), [p.device for p in ports])
    return ports


def fill_combo(combo, ports: list[PortInfo]) -> None:
    """
    Populate a QComboBox with *ports*.

    The port device string (e.g. ``"COM3"``) is stored as the item's
    ``userData`` so callers can retrieve it with
    ``combo.currentData()``.

    Args:
        combo:  A ``QComboBox`` instance.
        ports:  Output of :func:`list_ports`.
    """
    combo.clear()
    if ports:
        for p in ports:
            combo.addItem(p.display, userData=p.device)
    else:
        combo.addItem("(none found)", userData="")


def auto_assign(ports: list[PortInfo]) -> tuple[Optional[int], Optional[int]]:
    """
    Suggest (cli_index, data_index) for the IWR6843AOP EVM.

    The EVM enumerates two consecutive COM ports:
    - Lower number  → CLI  port (XDS110 Application UART)
    - Higher number → Data port (XDS110 Auxiliary/Bulk UART)

    Args:
        ports: Sorted list from :func:`list_ports`.

    Returns:
        Tuple of combo-box indices ``(cli_index, data_index)``.
        Returns ``(None, None)`` if fewer than two ports are present.
    """
    if len(ports) < 2:
        log.warning("auto_assign: need ≥2 ports, found %d", len(ports))
        return None, None

    # Prefer explicitly labelled ports
    cli_idx  = next((i for i, p in enumerate(ports) if "CLI"  in p.display), 0)
    data_idx = next((i for i, p in enumerate(ports) if "Data" in p.display),
                    len(ports) - 1)

    # Avoid selecting the same index for both
    if cli_idx == data_idx:
        cli_idx  = 0
        data_idx = len(ports) - 1

    log.debug("auto_assign → CLI:%s (idx %d)  Data:%s (idx %d)",
              ports[cli_idx].device,  cli_idx,
              ports[data_idx].device, data_idx)
    return cli_idx, data_idx 
