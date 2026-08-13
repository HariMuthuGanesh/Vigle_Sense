# ── Vigil_Sense v2 — integrated with companion modules ───────────
import sys, os, time, struct, threading, math, platform
import numpy as np

# Ensure `src/` packages are on Python import path
_src_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "src")
if _src_dir not in sys.path:
    sys.path.insert(0, _src_dir)

import serial
import serial.tools.list_ports
from collections import deque

# ── Companion module imports ──────────────────────────────────────
from config        import *                        # all constants
from logger        import get_logger               # structured logging
from alerts        import AlertManager             # audio alerts
from data_logger   import DataLogger               # CSV session export
from settings      import Settings                 # JSON persistence
from stats_tracker import StatsTracker             # FPS / pts / tracks
from port_utils    import list_ports, fill_combo, auto_assign  # port utils

_log = get_logger("vigil_sense_v2")

# ── Cross-platform font resolver ──────────────────────────────────
def _mono_font():
    """Return the best available monospace font for the current OS."""
    _sys = platform.system()
    if _sys == "Windows":
        return "Courier New"
    elif _sys == "Darwin":           # macOS
        return "Menlo"
    else:                            # Linux / other
        return "DejaVu Sans Mono"

MONO_FONT = _mono_font()

# ── Cross-platform DPI / HiDPI setup (called before QApplication) ─
def _setup_hidpi():
    """Enable HiDPI scaling on all platforms, including Windows fractional DPI."""
    # Qt 5.6+: auto-scale for HiDPI
    os.environ.setdefault("QT_AUTO_SCREEN_SCALE_FACTOR", "1")
    # Windows-specific: let Qt query the DPI from the OS
    if platform.system() == "Windows":
        os.environ.setdefault("QT_ENABLE_HIGHDPI_SCALING", "1")
        os.environ.setdefault("QT_SCALE_FACTOR_ROUNDING_POLICY", "PassThrough")

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QComboBox, QGroupBox, QGridLayout,
    QTextEdit, QFrame, QScrollArea, QDoubleSpinBox, QSplitter,
    QTabWidget, QSizePolicy, QSpacerItem
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QObject, QTimer, QMutex, QPointF, QRectF
from PyQt5.QtGui import (QFont, QColor, QPainter, QBrush, QPen,
                         QFontDatabase, QPainterPath, QLinearGradient, QPolygonF, QRadialGradient)

# ══════════════════════════════════════════════════════════════════
#  BACKEND CONSTANTS  (sourced from config.py)
# ══════════════════════════════════════════════════════════════════
# All constants are imported via `from config import *` above.
# Edit config.py to tune any parameter.

# ══════════════════════════════════════════════════════════════════
#  BACKEND CLASSES  
# ══════════════════════════════════════════════════════════════════
class RadarFrame:
    __slots__ = ["frame_num", "points", "targets"]
    def __init__(self):
        self.frame_num = 0
        self.points    = np.empty((0, 4), dtype=np.float32)
        self.targets   = []

class RadarParser:
    def parse_buffer(self, buf: bytes):
        frames = []
        while True:
            idx = buf.find(MAGIC_WORD)
            if idx < 0:
                buf = buf[-7:] if len(buf) >= 7 else buf
                break
            if idx > 0:
                buf = buf[idx:]
            if len(buf) < HEADER_SIZE:
                break
            if len(buf) < 16:
                break
            total_len = struct.unpack_from("<I", buf, 12)[0]
            if total_len < HEADER_SIZE or total_len > 65536:
                buf = buf[1:]
                continue
            if len(buf) < total_len:
                break
            frame = self._parse_frame(buf[:total_len])
            if frame is not None:
                frames.append(frame)
            buf = buf[total_len:]
        return frames, buf

    def _parse_frame(self, data: bytes):
        if data[:8] != MAGIC_WORD:
            return None
        f = RadarFrame()
        off = 8
        try:
            _ver        = struct.unpack_from("<I", data, off)[0]; off += 4
            _total_len  = struct.unpack_from("<I", data, off)[0]; off += 4
            _plat       = struct.unpack_from("<I", data, off)[0]; off += 4
            f.frame_num = struct.unpack_from("<I", data, off)[0]; off += 4
            _cpu_time   = struct.unpack_from("<I", data, off)[0]; off += 4
            num_det     = struct.unpack_from("<I", data, off)[0]; off += 4
            num_tlvs    = struct.unpack_from("<I", data, off)[0]; off += 4
            _sub        = struct.unpack_from("<I", data, off)[0]; off += 4
        except struct.error:
            return None

        height_map = {}

        for _ in range(num_tlvs):
            if off + 8 > len(data):
                break
            tlv_type = struct.unpack_from("<I", data, off)[0]; off += 4
            tlv_len  = struct.unpack_from("<I", data, off)[0]; off += 4
            if off + tlv_len > len(data):
                break
            tlv_data = data[off : off + tlv_len]; off += tlv_len

            if tlv_type == TLV_POINT_CLOUD:
                f.points = self._parse_points(tlv_data, num_det)
            elif tlv_type == TLV_TARGET_LIST:
                f.targets = self._parse_targets(tlv_data)
            elif tlv_type == TLV_TARGET_HEIGHT:
                height_map = self._parse_height(tlv_data)
            elif tlv_type == TLV_TARGET_IDX:
                pass
            elif tlv_type == TLV_PRESENCE:
                pass

        if height_map and f.targets:
            for t in f.targets:
                if t["id"] in height_map:
                    t["fw_height"] = height_map[t["id"]]

        return f

    def _parse_points(self, data: bytes, n: int):
        stride = 16
        if n == 0:
            n = len(data) // stride
        pts = []
        for i in range(n):
            if (i + 1) * stride > len(data):
                break
            x, y, z, d = struct.unpack_from("<ffff", data, i * stride)
            pts.append([x, y, z, d])
        return np.array(pts, dtype=np.float32) if pts else np.empty((0, 4), dtype=np.float32)

    def _parse_targets(self, data: bytes):
        targets = []
        if len(data) == 0:
            return targets
        if len(data) % 112 == 0 and len(data) // 112 >= 1:
            stride = 112
        elif len(data) % 40 == 0 and len(data) // 40 >= 1:
            stride = 40
        else:
            stride = 40
        n = len(data) // stride
        for i in range(n):
            off = i * stride
            if off + 4 > len(data):
                break
            tid = struct.unpack_from("<I", data, off)[0]
            if off + 28 > len(data):
                break
            # TLV 1010 targetStruct3D layout (after tid):
            #   [4]  posX   [8]  posY   [12] posZ
            #   [16] velX   [20] velY   [24] velZ  (native Doppler)
            x, y, z, vx, vy, vz = struct.unpack_from("<ffffff", data, off + 4)
            if not (-20 < x < 20 and 0 < y < 20 and -1 < z < 5):
                continue
            targets.append({
                "id": tid,
                "x": float(x), "y": float(y), "z": float(z),
                "vx": float(vx), "vy": float(vy), "vz": float(vz),
            })
        return targets

    def _parse_height(self, data: bytes):
        height_map = {}
        stride = 12
        n = len(data) // stride
        for i in range(n):
            off = i * stride
            if off + stride > len(data):
                break
            tid, max_z, min_z = struct.unpack_from("<Iff", data, off)
            height_map[tid] = float(max_z - min_z)
        return height_map

class HazardZone:
    def __init__(self, x0=-1.5, x1=1.5, y0=2.0, y1=5.0, z0=0.0, z1=3.0):
        self.update(x0, x1, y0, y1, z0, z1)

    def update(self, x0, x1, y0, y1, z0, z1):
        self.x0, self.x1 = x0, x1
        self.y0, self.y1 = y0, y1
        self.z0, self.z1 = z0, z1

    def contains(self, x, y, z):
        return (self.x0 <= x <= self.x1 and
                self.y0 <= y <= self.y1 and
                self.z0 <= z <= self.z1)

    def corners(self):
        c = np.array([
            [self.x0, self.y0, self.z0], [self.x1, self.y0, self.z0],
            [self.x1, self.y1, self.z0], [self.x0, self.y1, self.z0],
            [self.x0, self.y0, self.z1], [self.x1, self.y0, self.z1],
            [self.x1, self.y1, self.z1], [self.x0, self.y1, self.z1],
        ])
        return c

class PersonState:
    """
    Person tracking state.
      • HEIGHT_BIAS (+15 cm) corrects radar underestimation.
      • IIR alpha filter (HEIGHT_ALPHA) smooths height readings.
    """

    def __init__(self, tid):
        self.tid = tid

        # 3-D position and radar-native Doppler velocities
        self.x = self.y = self.z = 0.0
        self.vx = self.vy = self.vz = 0.0

        # Height tracking
        self.height     = 0.0
        self.max_height = 0.0   # historical maximum

        # Alert states
        self.in_hazard = False

        # Tuning parameters
        self.alpha = HEIGHT_ALPHA   # IIR smoothing weight
        self.bias  = HEIGHT_BIAS    # algorithmic bias correction

    def update(self, x, y, z, vx, vy, vz, points, zone: HazardZone, fw_height=None):
        self.x, self.y, self.z = x, y, z
        self.vx, self.vy, self.vz = vx, vy, vz

        # ── 1. Hazard zone geofencing ──────────────────────────────
        self.in_hazard = zone.contains(x, y, z)

        # ── 2. Height calculation with bias correction ─────────────
        raw_h = 0.0
        if fw_height is not None and fw_height > 0:
            raw_h = float(fw_height)
        elif points is not None and len(points) > 0:
            try:
                px = points[:, 0].astype(np.float64)
                py = points[:, 1].astype(np.float64)
                d2   = (px - float(x)) ** 2 + (py - float(y)) ** 2
                near = points[d2 < 0.49]
                if len(near) >= 2:
                    raw_h = float(np.max(near[:, 2]) - np.min(near[:, 2]))
                elif len(near) == 1:
                    raw_h = float(abs(near[0, 2]))
            except Exception:
                pass

        if raw_h > 0.01:
            raw_h += self.bias
            if self.height == 0.0:
                self.height = raw_h
            else:
                self.height = (self.alpha * raw_h) + ((1.0 - self.alpha) * self.height)

        if self.height > self.max_height:
            self.max_height = self.height

class WorkerSignals(QObject):
    log          = pyqtSignal(str, str)
    config_ok    = pyqtSignal(bool)
    data_started = pyqtSignal()
    frame        = pyqtSignal(object)
    error        = pyqtSignal(str)

class SerialWorker(QThread):
    def __init__(self, cli_port, data_port, config_text):
        super().__init__()
        self.sig         = WorkerSignals()
        self.cli_port    = cli_port
        self.data_port   = data_port
        self.config_text = config_text
        self._stop_evt   = threading.Event()
        self.parser      = RadarParser()
        self._ser_cli    = None
        self._ser_data   = None

    def _probe_cli(self, port) -> bool:
        self.sig.log.emit(f"  Probing {port} as CLI…", "info")
        try:
            s = serial.Serial(port, CLI_BAUD,
                              bytesize=serial.EIGHTBITS,
                              parity=serial.PARITY_NONE,
                              stopbits=serial.STOPBITS_ONE,
                              timeout=1, write_timeout=2)
            time.sleep(0.25)
            s.reset_input_buffer(); s.reset_output_buffer()
            s.write(b"sensorStop\n"); s.flush()
            t0 = time.time(); resp = b""
            while time.time() - t0 < 2.5:
                if s.in_waiting:
                    resp += s.read(s.in_waiting)
                    text = resp.decode("ascii", errors="ignore")
                    if "Done" in text or "mmWave" in text or \
                       "sensorStop" in text or "Error" in text:
                        self.sig.log.emit(
                            f"  ✔ CLI confirmed on {port}: "
                            f"{text.strip().splitlines()[0][:60]}", "ok")
                        if self._ser_cli and self._ser_cli.is_open:
                            try: self._ser_cli.close()
                            except: pass
                        self._ser_cli = s
                        return True
                time.sleep(0.015)
            s.close()
            self.sig.log.emit(f"  ✗ No response on {port}", "dim")
            return False
        except serial.SerialException as e:
            self.sig.log.emit(f"  Cannot open {port}: {e}", "warn")
            return False

    def run(self):
        try:
            if self.cli_port == self.data_port:
                self.sig.log.emit("⚠  CLI and Data ports must be different!", "error")
                self.sig.config_ok.emit(False)
                return

            self.sig.log.emit("━━━ Auto-detecting CLI port ━━━", "info")
            if self._probe_cli(self.cli_port):
                pass
            elif self._probe_cli(self.data_port):
                self.cli_port, self.data_port = self.data_port, self.cli_port
                self.sig.log.emit(
                    f"  Ports swapped → CLI={self.cli_port}  Data={self.data_port}", "warn")
            else:
                self.sig.log.emit("✘  No CLI response on either port. Checklist:", "error")
                self.sig.log.emit("  1. EVM powered? (green/blue LEDs on the board)", "warn")
                self.sig.log.emit("  2. People Tracking firmware flashed?", "warn")
                self.sig.log.emit("  3. Close TI Demo Visualizer if open (port conflict)", "warn")
                self.sig.log.emit("  4. Unplug USB → replug → click CONNECT again", "warn")
                self.sig.config_ok.emit(False)
                return

            time.sleep(0.2)
            self._ser_cli.reset_input_buffer()

            self.sig.log.emit("━━━ Sending AOP_6m People Tracking config ━━━", "info")
            if not self._send_config():
                return

            self.sig.log.emit(f"Opening Data port {self.data_port} @ {DATA_BAUD}", "info")
            try:
                self._ser_data = serial.Serial(
                    self.data_port, DATA_BAUD,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=0.05)
                self._ser_data.reset_input_buffer()
            except serial.SerialException as e:
                self.sig.log.emit(f"Cannot open Data port: {e}", "error")
                self.sig.config_ok.emit(False)
                return

            self.sig.log.emit("Waiting for radar data frames…", "info")
            sniff_buf = b""; magic_found = False; t0 = time.time()
            while time.time() - t0 < 10.0 and not self._stop_evt.is_set():
                c = self._ser_data.read(1024)
                if c:
                    sniff_buf += c
                    if MAGIC_WORD in sniff_buf:
                        magic_found = True
                        break

            if not magic_found:
                self.sig.log.emit("✘  No data frames on Data port after 10 s.", "error")
                self.sig.log.emit("   → Sensor may not have started correctly.", "warn")
                self.sig.log.emit("   → Try: STOP → power-cycle EVM → CONNECT", "warn")
                self.sig.config_ok.emit(False)
                return

            self.sig.log.emit(f"✔  Radar frames live on {self.data_port} ✓", "ok")
            self.sig.data_started.emit()

            buf = sniff_buf; last_t = time.time()
            while not self._stop_evt.is_set():
                try:
                    c = self._ser_data.read(8192)
                except serial.SerialException as e:
                    self.sig.log.emit(f"Read error: {e}", "error"); break
                if c:
                    last_t = time.time()
                    buf += c
                    frames, buf = self.parser.parse_buffer(buf)
                    for fr in frames:
                        self.sig.frame.emit(fr)
                else:
                    if time.time() - last_t > 5.0:
                        self.sig.log.emit("⚠  No frames for 5 s — sensor may have stopped", "warn")
                        last_t = time.time()

        except Exception as exc:
            self.sig.log.emit(f"Unexpected error: {exc}", "error")
            self.sig.config_ok.emit(False)
        finally:
            self._close()

    def _send_config(self):
        lines = [l.strip() for l in self.config_text.splitlines()
                 if l.strip() and not l.strip().startswith("%")]
        total = len(lines)
        for i, line in enumerate(lines):
            if self._stop_evt.is_set():
                return False
            try:
                self._ser_cli.write((line + "\n").encode("ascii"))
                self._ser_cli.flush()
            except serial.SerialException as e:
                self.sig.log.emit(f"Write error on '{line}': {e}", "error")
                self.sig.config_ok.emit(False)
                return False

            self.sig.log.emit(f"  [{i+1:02d}/{total}] TX ▶  {line}", "tx")

            ack_timeout = 6.0 if line.strip() == "sensorStart" else 4.0
            ack   = self._read_ack(timeout=ack_timeout)
            clean = ack.replace("\r", " ").replace("\n", " ").strip()

            if "Done" in ack or ("mmwDemo:/>" in ack and "Error" not in ack):
                self.sig.log.emit(f"         ◀ Done ✓", "ok")
            elif "0xffe" in ack or "Calibration" in ack or "calib" in ack.lower():
                self.sig.log.emit(f"         ◀ ⚠ CALIBRATION ERROR: {clean[:120]}", "error")
                self.sig.log.emit("  ╔══ DIAGNOSIS ═══════════════════════════════════╗", "error")
                self.sig.log.emit("  ║ sensorStart returned 0xffe (Init Calib failed) ║", "error")
                self.sig.log.emit("  ╚════════════════════════════════════════════════╝", "error")
            elif clean:
                self.sig.log.emit(f"         ◀ {clean[:90]}", "dim")
            else:
                self.sig.log.emit(f"         ◀ (timeout — ok)", "dim")

            time.sleep(0.06)

        self.sig.log.emit(f"━━━ Config sent ({total} commands) ✓ ━━━", "ok")
        self.sig.config_ok.emit(True)
        return True

    def _read_ack(self, timeout=4.0):
        deadline = time.time() + timeout
        buf = ""
        while time.time() < deadline:
            if self._stop_evt.is_set():
                break
            if self._ser_cli.in_waiting:
                buf += self._ser_cli.read(
                    self._ser_cli.in_waiting).decode("ascii", errors="ignore")
                if "Done" in buf or "done" in buf:
                    break
                if "not recognized" in buf.lower():
                    break
                if "Error" in buf:
                    break
                if "mmwDemo:/>" in buf:
                    break
            else:
                time.sleep(0.01)
        return buf

    def stop(self):
        self._stop_evt.set()
        self.wait(3000)

    def _close(self):
        for s, name in [(self._ser_cli, "CLI"), (self._ser_data, "Data")]:
            try:
                if s and s.is_open:
                    if name == "CLI":
                        try: s.write(b"sensorStop\n"); s.flush(); time.sleep(0.15)
                        except: pass
                    s.close()
                    self.sig.log.emit(f"{name} port closed.", "dim")
            except Exception:
                pass


# ══════════════════════════════════════════════════════════════════
#  INDUSTRIAL FLOOR PLAN WIDGET  (new main view)
#  Large factory room with 2 restricted chambers
# ══════════════════════════════════════════════════════════════════
class IndustrialFloorPlanWidget(QWidget):
    """
    Full isometric 3D view of a large industrial facility.
    Uses cabinet/oblique projection:
      - X axis  → horizontal on screen
      - Y axis  → receding depth (oblique upper-right)
      - Z axis  → vertical (height)
    Features:
      - Thick 3D walls with top caps and inner faces
      - Two restricted chambers as solid 3D boxes with glowing walls
      - Structural columns as 3D pillars
      - Workbenches, inspection table as 3D objects
      - Persons as 3D pillars with coloured height bars
      - Point cloud dots positioned in 3D space
      - Animated chamber breach: pulsing red walls + flash overlay
    """

    # Chamber definitions in radar world-space metres (x0, x1, y0, y1)
    CHAMBER_A = (-4.5, -2.0, 2.5, 5.0)
    CHAMBER_B = ( 2.0,  4.5, 2.5, 7.0)

    # Full facility bounds (metres, sensor at 0,0)
    FACILITY_X = (-6.0, 6.0)
    FACILITY_Y = (-0.5, 6.0)


    # ── Isometric projection constants ────────────────────────────
    # Cabinet oblique projection:  angle=30°, depth ratio=0.5
    _ISO_ANG   = math.radians(30)
    _ISO_DEPTH = 0.50          # foreshortening

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(520)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setStyleSheet(f"background:{BG};")
        self.setMouseTracking(True)

        self.safe_r       = 3.0
        self.alert_r      = 5.0
        self.restricted_r = 7.5
        self.view_range   = 9.5

        self._points   = np.empty((0, 4), dtype=np.float32)
        self._targets  = []
        self._persons  = {}

        self._chamber_a_breach = False
        self._chamber_b_breach = False

        self._anim_tick = 0
        self._anim_timer = QTimer(self)
        self._anim_timer.timeout.connect(self._tick_anim)
        self._anim_timer.start(80)

        # ── Interactive rotation / zoom state ─────────────────────
        # Oblique angle (horizontal sweep of depth axis, radians)
        self._view_ang   = math.radians(30)    # default 30°
        # Depth foreshortening ratio (0.1 = near-orthographic, 1.0 = full)
        self._view_depth = 0.50
        # Vertical tilt: fraction of full height compressed by perspective
        self._view_tilt  = 0.28                # ds_raw = sin(ang)*depth*tilt_scale
        # Zoom multiplier
        self._zoom       = 1.0

        self._drag_btn   = None   # Qt.LeftButton or Qt.RightButton
        self._drag_start = None   # QPoint where drag began
        self._ang_start  = None
        self._depth_start= None
        self._tilt_start = None
        self.setCursor(Qt.OpenHandCursor)

    def _tick_anim(self):
        self._anim_tick = (self._anim_tick + 1) % 60
        if self._chamber_a_breach or self._chamber_b_breach:
            self.update()

    # ── Mouse interaction ──────────────────────────────────────────
    def mousePressEvent(self, ev):
        self._drag_btn   = ev.button()
        self._drag_start = ev.pos()
        self._ang_start  = self._view_ang
        self._depth_start= self._view_depth
        self._tilt_start = self._view_tilt
        self.setCursor(Qt.ClosedHandCursor)
        ev.accept()

    def mouseMoveEvent(self, ev):
        if self._drag_start is None or self._drag_btn is None:
            return
        dx = ev.pos().x() - self._drag_start.x()
        dy = ev.pos().y() - self._drag_start.y()

        if self._drag_btn == Qt.LeftButton:
            # Horizontal drag → rotate oblique angle (−60° to +80°)
            self._view_ang = max(math.radians(-60),
                                 min(math.radians(80),
                                     self._ang_start + dx * 0.007))
            # Vertical drag → adjust tilt (depth vertical component)
            self._view_tilt = max(0.05,
                                  min(1.2,
                                      self._tilt_start - dy * 0.004))
        elif self._drag_btn == Qt.RightButton:
            # Horizontal drag → depth foreshortening
            self._view_depth = max(0.10,
                                   min(1.20,
                                       self._depth_start + dx * 0.004))
            # Vertical drag → zoom
            self._zoom = max(0.4, min(3.0,
                                      self._zoom * (1 - dy * 0.003)))
        self.update()
        ev.accept()

    def mouseReleaseEvent(self, ev):
        self._drag_btn   = None
        self._drag_start = None
        self.setCursor(Qt.OpenHandCursor)
        ev.accept()

    def mouseDoubleClickEvent(self, ev):
        """Double-click resets view to default."""
        self._view_ang   = math.radians(30)
        self._view_depth = 0.50
        self._view_tilt  = 0.28
        self._zoom       = 1.0
        self.update()
        ev.accept()

    def wheelEvent(self, ev):
        delta = ev.angleDelta().y()
        factor = 1.0 + delta / 1200.0
        self._zoom = max(0.4, min(3.0, self._zoom * factor))
        self.update()
        ev.accept()

    def update_scene(self, points, targets, persons):
        self._points  = points
        self._targets = targets
        self._persons = persons
        self._chamber_a_breach = False
        self._chamber_b_breach = False
        for t in targets:
            if self._in_chamber(t["x"], t["y"], self.CHAMBER_A):
                self._chamber_a_breach = True
            if self._in_chamber(t["x"], t["y"], self.CHAMBER_B):
                self._chamber_b_breach = True
        self.update()

    def set_machine_prox(self, prox_a: bool, prox_b: bool):
        """Called by MainWindow each frame with machine proximity flags."""
        if not hasattr(self, '_mach_prox_a'):
            self._mach_prox_a = False
            self._mach_prox_b = False
        changed = (prox_a != self._mach_prox_a) or (prox_b != self._mach_prox_b)
        self._mach_prox_a = prox_a
        self._mach_prox_b = prox_b
        if changed:
            self.update()

    def refresh_zones(self):
        self.update()

    def _in_chamber(self, x, y, chamber):
        x0, x1, y0, y1 = chamber
        return x0 <= x <= x1 and y0 <= y <= y1

    def _proj(self, x, y, z, ox, oy, sx, sz, dc, ds):
        """Project world (x,y,z) to screen QPointF."""
        return QPointF(
            ox + x * sx + y * dc,
            oy - z * sz - y * ds
        )

    def _compute_iso(self, W, H):
        """
        Compute projection using instance rotation/zoom state.
        Mouse drag changes _view_ang, _view_depth, _view_tilt, _zoom.
        """
        fx0, fx1 = self.FACILITY_X
        fy0, fy1 = self.FACILITY_Y
        wall_h = 2.8

        ang   = self._view_ang
        depth = self._view_depth
        tilt  = self._view_tilt

        dc_raw = math.cos(ang) * depth
        ds_raw = math.sin(ang) * depth * tilt * 2.0  # tilt controls vertical recession

        ML = 70;  MR = 65;  MT = 72;  MB = 95
        avail_w = W - ML - MR
        avail_h = H - MT - MB

        fac_w = fx1 - fx0
        fac_d = fy1 - fy0

        # Base scale that fits the facility, then apply zoom
        # Guard against near-zero ds_raw when view is nearly top-down
        denom_h = max(abs(fac_d * ds_raw) + wall_h, wall_h * 0.5)
        sx_base = min(avail_w / max(fac_w + abs(fac_d * dc_raw), 0.1),
                      avail_h / denom_h) * 0.80
        sx = sx_base * self._zoom
        sz = sx
        dc = dc_raw * sx
        ds = ds_raw * sz

        # Centre box horizontally
        box_left_rel  = fx0 * sx + fy0 * dc
        box_right_rel = fx1 * sx + fy1 * dc
        ox = ML + avail_w / 2 - (box_left_rel + box_right_rel) / 2

        # Place floor at 74% down available height (adjusted for tilt)
        oy = MT + avail_h * 0.74 - (-fy0 * ds)

        return ox, oy, sx, sz, dc, ds, wall_h

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        W = self.width(); H = self.height()

        p.fillRect(0, 0, W, H, QColor(BG))

        ox, oy, sx, sz, dc, ds, WH = self._compute_iso(W, H)
        tick = self._anim_tick

        def pt(x, y, z):
            return self._proj(x, y, z, ox, oy, sx, sz, dc, ds)

        fx0, fx1 = self.FACILITY_X
        fy0, fy1 = self.FACILITY_Y

        # ── Title (always in top-left margin, never overlaps scene) ─
        p.setPen(QPen(QColor(PHOSPHOR)))
        p.setFont(QFont(MONO_FONT, 11, QFont.Bold))
        p.drawText(16, 26, "INDUSTRIAL FACILITY  —  3D ISOMETRIC VIEW")
        p.setPen(QPen(QColor(SUBTEXT)))
        p.setFont(QFont(MONO_FONT, 8))
        p.drawText(16, 40, f"IWR6843AOP  ·  60 GHz mmWave  ·  Cabinet Oblique  ·  {W}×{H} px")
        # Thin separator line under title
        p.setPen(QPen(QColor(BORDER), 1))
        p.drawLine(16, 48, W - 16, 48)

        

        # ── 1. Ambient background ──────────────────────────────────
        grad = QLinearGradient(0, 0, 0, H)
        grad.setColorAt(0, QColor("#0a100a"))
        grad.setColorAt(1, QColor(BG))
        p.setBrush(QBrush(grad)); p.setPen(Qt.NoPen)
        p.drawRect(0, 0, W, H)

        # ── Helper: draw a flat poly face ──────────────────────────
        def face(corners_xyz, fill_hex, fill_alpha, edge_hex, edge_w=1):
            poly = QPolygonF([pt(*c) for c in corners_xyz])
            fc = QColor(fill_hex); fc.setAlpha(fill_alpha)
            p.setBrush(QBrush(fc))
            p.setPen(QPen(QColor(edge_hex), edge_w))
            p.drawPolygon(poly)

        # ── Helper: draw a filled rect wall face with grid lines ───
        def wall_face(corners_xyz, fill_hex, fill_alpha, edge_hex,
                      grid_hex="#1a2a1a", h_divs=3, v_divs=6):
            poly = QPolygonF([pt(*c) for c in corners_xyz])
            fc = QColor(fill_hex); fc.setAlpha(fill_alpha)
            p.setBrush(QBrush(fc))
            p.setPen(QPen(QColor(edge_hex), 1))
            p.drawPolygon(poly)

        # ── 2. Back wall (y = fy1) ─────────────────────────────────
        bw_corners = [
            (fx0, fy1, 0),  (fx1, fy1, 0),
            (fx1, fy1, WH), (fx0, fy1, WH),
        ]
        face(bw_corners, "#0c180c", 255, FLOOR_WALL_LIT, 1)
        # Horizontal mortar lines on back wall
        p.setPen(QPen(QColor("#182818"), 1))
        for zi in [0.7, 1.4, 2.1]:
            if zi < WH:
                p.drawLine(pt(fx0, fy1, zi), pt(fx1, fy1, zi))
        # Vertical mortar lines
        for xi in range(int(fx0), int(fx1)+1):
            p.setPen(QPen(QColor("#182818"), 1))
            p.drawLine(pt(xi, fy1, 0), pt(xi, fy1, WH))
        # Top cap
        face([(fx0,fy1,WH),(fx1,fy1,WH),(fx1,fy1-0.3,WH),(fx0,fy1-0.3,WH)],
             "#1e3a1e", 200, FLOOR_WALL_LIT, 1)

        # ── 3. Left wall (x = fx0) ─────────────────────────────────
        lw_corners = [
            (fx0, fy0, 0),  (fx0, fy1, 0),
            (fx0, fy1, WH), (fx0, fy0, WH),
        ]
        face(lw_corners, "#0a160a", 255, FLOOR_WALL_LIT, 1)
        for zi in [0.7, 1.4, 2.1]:
            if zi < WH:
                p.setPen(QPen(QColor("#182818"), 1))
                p.drawLine(pt(fx0, fy0, zi), pt(fx0, fy1, zi))
        for yi in range(int(fy0), int(fy1)+1):
            p.setPen(QPen(QColor("#182818"), 1))
            p.drawLine(pt(fx0, yi, 0), pt(fx0, yi, WH))
        face([(fx0,fy0,WH),(fx0,fy1,WH),(fx0+0.3,fy1,WH),(fx0+0.3,fy0,WH)],
             "#1e3a1e", 200, FLOOR_WALL_LIT, 1)

        # ── 4. Floor ───────────────────────────────────────────────
        floor_corners = [
            (fx0, fy0, 0), (fx1, fy0, 0),
            (fx1, fy1, 0), (fx0, fy1, 0),
        ]
        face(floor_corners, "#0c1a0c", 255, "#1a2a1a", 1)
        # Floor grid
        xi = math.ceil(fx0)
        while xi <= fx1:
            col_f = "#1e2e1e" if xi % 2 == 0 else "#162416"
            p.setPen(QPen(QColor(col_f), 1))
            p.drawLine(pt(xi, fy0, 0), pt(xi, fy1, 0))
            xi += 1
        yi = math.ceil(fy0)
        while yi <= fy1:
            col_f = "#1e2e1e" if yi % 2 == 0 else "#162416"
            p.setPen(QPen(QColor(col_f), 1))
            p.drawLine(pt(fx0, yi, 0), pt(fx1, yi, 0))
            yi += 1

        # Floor distance rings projected onto floor plane (faint)
        for r in [2, 4, 6, 8]:
            steps = 40
            ring_pts = []
            for i in range(steps + 1):
                ang = math.pi * i / steps  # front half only
                rx = r * math.cos(ang)
                ry = r * math.sin(ang)
                ring_pts.append(pt(rx, ry, 0))
            p.setPen(QPen(QColor("#1a2e1a"), 1, Qt.DotLine))
            for i in range(len(ring_pts) - 1):
                p.drawLine(ring_pts[i], ring_pts[i+1])
            
            # overlap the scene
            label_pt = pt(0, r, 0)
            p.setPen(QPen(QColor(SUBTEXT)))
            p.setFont(QFont(MONO_FONT, 7))
            p.drawText(QPointF(label_pt.x() + 6, label_pt.y() - 2), f"{r}m")

        # ── 5. Right wall (x = fx1) ────────────────────────────────
        rw_corners = [
            (fx1, fy0, 0),  (fx1, fy1, 0),
            (fx1, fy1, WH), (fx1, fy0, WH),
        ]
        face(rw_corners, "#0a160a", 255, FLOOR_WALL_LIT, 1)
        for zi in [0.7, 1.4, 2.1]:
            if zi < WH:
                p.setPen(QPen(QColor("#182818"), 1))
                p.drawLine(pt(fx1, fy0, zi), pt(fx1, fy1, zi))
        for yi in range(int(fy0), int(fy1)+1):
            p.setPen(QPen(QColor("#182818"), 1))
            p.drawLine(pt(fx1, yi, 0), pt(fx1, yi, WH))
        face([(fx1,fy0,WH),(fx1,fy1,WH),(fx1-0.3,fy1,WH),(fx1-0.3,fy0,WH)],
             "#1e3a1e", 200, FLOOR_WALL_LIT, 1)

        # ── 6c. Structural columns — removed ──────────────────────

        # ── 7. Restricted Chambers ─────────────────────────────────
        self._draw_chamber_3d(p, pt, face,
                              self.CHAMBER_A, "UNIT A",
                              self._chamber_a_breach, tick, WH, sx,
                              machine_prox=getattr(self, '_mach_prox_a', False))
        self._draw_chamber_3d(p, pt, face,
                              self.CHAMBER_B, "UNIT B",
                              self._chamber_b_breach, tick, WH, sx,
                              machine_prox=getattr(self, '_mach_prox_b', False))

        # ── 8. Inspection table — removed ─────────────────────────

        # ── 9. Point cloud ─────────────────────────────────────────
        if len(self._points) > 0:
            p.setPen(Qt.NoPen)
            for pp_ in self._points:
                x_, y_, z_ = float(pp_[0]), float(pp_[1]), float(pp_[2])
                sp = pt(x_, y_, max(z_, 0.02))
                g = int(70 + min(max(z_ / 3.0, 0), 1) * 170)
                dot_c = QColor(0, g, int(g * 0.25), 180)
                p.setBrush(QBrush(dot_c))
                p.drawEllipse(sp, 2, 2)

        # ── 10. Persons ────────────────────────────────────────────
        ZONE_COLS = [
            (PHOSPHOR, "#00aa33"),   # safe
            (AMBER,    "#cc7700"),   # alert
            (RED_ALERT,"#aa1010"),   # restricted
        ]
        for t in self._targets:
            per  = self._persons.get(t["id"])
            x_, y_, z_ = t["x"], t["y"], t["z"]
            ht   = max(per.height if per else 1.7, 0.45)

            in_ch_a = self._in_chamber(x_, y_, self.CHAMBER_A)
            in_ch_b = self._in_chamber(x_, y_, self.CHAMBER_B)

            # Floor Plan: colour based on chamber XY position ONLY
            if in_ch_a or in_ch_b:
                zi = 2
                ec, fc = CHAMBER_GLOW, "#880000"   # bright red = inside unit
            else:
                zi = 0
                ec, fc = ZONE_COLS[0]              # green = outside unit (safe)

            col  = QColor(ec)
            ht_c = min(ht, WH - 0.1)
            bot  = pt(x_, y_, 0.04)
            top_ = pt(x_, y_, ht_c)
            bw   = max(7, int(sx * 0.28))

            # Ground shadow ellipse
            glow_c = QColor(ec); glow_c.setAlpha(35)
            p.setBrush(QBrush(glow_c)); p.setPen(Qt.NoPen)
            p.drawEllipse(bot, bw + 10, 6)

            # Body – filled rect
            body = QRectF(bot.x() - bw/2, top_.y(), bw, bot.y() - top_.y())
            fill_c = QColor(fc); fill_c.setAlpha(160)
            p.setBrush(QBrush(fill_c))
            p.setPen(QPen(col, 2))
            p.drawRect(body)

            # Side shading strip on body
            side_c = QColor(fc); side_c.setAlpha(60)
            p.setBrush(QBrush(side_c)); p.setPen(Qt.NoPen)
            p.drawRect(QRectF(body.right(), body.top() + 2, 4, body.height() - 2))

            # Head circle
            hr = max(5, bw // 2)
            p.setBrush(QBrush(col))
            p.setPen(QPen(QColor("#ffffff"), 1))
            p.drawEllipse(QPointF(top_.x(), top_.y() - hr), hr, hr)

            # ID tag
            p.setPen(QPen(QColor(ec)))
            p.setFont(QFont(MONO_FONT, 7, QFont.Bold))
            p.drawText(QPointF(top_.x() - 10, top_.y() - hr * 2 - 4), f"#{t['id']:02d}")

            # Height label
            lbl_txt = "BRCH" if (in_ch_a or in_ch_b) else "SAFE"
            p.setPen(QPen(QColor("#ffffff")))
            p.setFont(QFont(MONO_FONT, 7, QFont.Bold))
            p.drawText(QPointF(top_.x() + hr + 4, top_.y()),
                       f"{ht:.1f}m [{lbl_txt}]")

            # Breach badge
            if in_ch_a or in_ch_b:
                pulse = abs(math.sin(tick * 0.105))
                bc_c = QColor(CHAMBER_GLOW); bc_c.setAlpha(int(160 + pulse * 95))
                p.setPen(QPen(bc_c))
                p.setFont(QFont(MONO_FONT, 8, QFont.Bold))
                p.drawText(QPointF(top_.x() - 28, top_.y() - hr * 2 - 16), "⚠ BREACH")
                p.setPen(QPen(bc_c, 2))
                p.setBrush(Qt.NoBrush)
                p.drawEllipse(bot, hr + 12, 8)
                p.drawEllipse(bot, hr + 20, 12)

        # ── 11. Sensor ─────────────────────────────────────────────
        org = pt(0, 0, 0)
        sg  = QRadialGradient(org, 24)
        sg1 = QColor(PHOSPHOR); sg1.setAlpha(70)
        sg.setColorAt(0, sg1); sg.setColorAt(1, QColor(0,0,0,0))
        p.setBrush(QBrush(sg)); p.setPen(Qt.NoPen)
        p.drawEllipse(org, 24, 14)
        p.setBrush(QBrush(QColor(PHOSPHOR)))
        p.setPen(QPen(QColor("#ffffff"), 1))
        p.drawEllipse(org, 6, 6)
        p.setPen(QPen(QColor(PHOSPHOR)))
        p.setFont(QFont(MONO_FONT, 8, QFont.Bold))
        p.drawText(QPointF(org.x() + 12, org.y() + 6), "SENSOR")
        p.setPen(QPen(QColor(DIM_GREEN)))
        p.setFont(QFont(MONO_FONT, 7))
        p.drawText(QPointF(org.x() + 12, org.y() + 17), "IWR6843AOP")

        # Sensor pole (vertical line up to wall height)
        p.setPen(QPen(QColor(DIM_GREEN), 1, Qt.DotLine))
        p.drawLine(org, pt(0, 0, 0.8))

        # ── 12. Front wall (partial – knee-high so we see inside) ──
        fw_h = 0.45   # front wall height — low so interior stays visible
        fw_corners = [
            (fx0, fy0, 0),  (fx1, fy0, 0),
            (fx1, fy0, fw_h), (fx0, fy0, fw_h),
        ]
        face(fw_corners, "#0e1c0e", 200, FLOOR_WALL_LIT, 2)
        # Top cap strip on front wall
        face([(fx0,fy0,fw_h),(fx1,fy0,fw_h),
              (fx1,fy0+0.2,fw_h),(fx0,fy0+0.2,fw_h)],
             "#2a4a2a", 200, FLOOR_WALL_LIT, 1)

        # Door gap in front wall (centre)
        door_w = 1.2
        
        face([(-door_w/2, fy0, 0),(door_w/2, fy0, 0),
              (door_w/2, fy0, fw_h+0.1),(-door_w/2, fy0, fw_h+0.1)],
             "#0c1a0c", 255, "#0c1a0c", 0)
        # Door frame pillars
        for dx in [-door_w/2, door_w/2]:
            face([(dx-0.06, fy0, 0),(dx+0.06, fy0, 0),
                  (dx+0.06, fy0, WH*0.3),(dx-0.06, fy0, WH*0.3)],
                 "#1e3a1e", 230, CYAN_INFO, 1)
        p.setPen(QPen(QColor(CYAN_INFO)))
        p.setFont(QFont(MONO_FONT, 7))
        door_lbl_pt = pt(0, fy0, 0)
        p.setPen(QPen(QColor(CYAN_INFO)))
        p.setFont(QFont(MONO_FONT, 7))
        p.drawText(QPointF(door_lbl_pt.x() - 26, door_lbl_pt.y() + 22), "MAIN ENTRY")

        # ── 13. Axis arrows + labels ───────────────────────────────
        # All axis lines and labels drawn OUTSIDE the building footprint
        AX = "#4aaa4a"

        # X axis: runs along the front wall bottom edge (y=fy0, z=0)
        ax_orig = pt(fx0, fy0, 0)
        ax_end  = pt(fx1 + 1.0, fy0, 0)
        p.setPen(QPen(QColor(AX), 2))
        p.drawLine(ax_orig, ax_end)
        # arrowhead
        p.drawLine(ax_end, QPointF(ax_end.x() - 6, ax_end.y() - 4))
        p.drawLine(ax_end, QPointF(ax_end.x() - 6, ax_end.y() + 4))
        p.setPen(QPen(QColor(AX)))
        p.setFont(QFont(MONO_FONT, 8, QFont.Bold))
        p.drawText(QPointF(ax_end.x() + 4, ax_end.y() + 4), "X (m)")

        # Y axis (depth): runs along the LEFT wall bottom edge (x=fx0, z=0)
        ay_end = pt(fx0, fy1 + 1.0, 0)
        p.setPen(QPen(QColor(AX), 2))
        p.drawLine(ax_orig, ay_end)
        p.drawLine(ay_end, QPointF(ay_end.x() - 4, ay_end.y() - 6))
        p.drawLine(ay_end, QPointF(ay_end.x() + 4, ay_end.y() - 6))
        p.setPen(QPen(QColor(AX)))
        p.setFont(QFont(MONO_FONT, 8, QFont.Bold))
        p.drawText(QPointF(ay_end.x() + 4, ay_end.y()), "Y (m)")

        # Z axis (height): vertical at left-front corner
        az_end = pt(fx0, fy0, WH + 0.5)
        p.setPen(QPen(QColor(AX), 2))
        p.drawLine(ax_orig, az_end)
        p.drawLine(az_end, QPointF(az_end.x() - 4, az_end.y() + 6))
        p.drawLine(az_end, QPointF(az_end.x() + 4, az_end.y() + 6))
        p.setPen(QPen(QColor(AX)))
        p.setFont(QFont(MONO_FONT, 8, QFont.Bold))
        p.drawText(QPointF(az_end.x() - 34, az_end.y() - 2), "Z (m)")

        # X tick labels — below the front wall baseline
        p.setFont(QFont(MONO_FONT, 7))
        p.setPen(QPen(QColor("#5a9a5a")))
        xi = math.ceil(fx0)
        while xi <= fx1:
            tp = pt(xi, fy0, 0)
            p.setPen(QPen(QColor("#3a6a3a"), 1))
            p.drawLine(QPointF(tp.x(), tp.y() + 2), QPointF(tp.x(), tp.y() + 7))
            p.setPen(QPen(QColor("#5a9a5a")))
            p.setFont(QFont(MONO_FONT, 6))
            p.drawText(QPointF(tp.x() - 5, tp.y() + 18), f"{xi}")
            xi += 2

        # Y (depth) tick labels — left of the left wall, along Y axis
        yi = 0
        while yi <= int(fy1):
            tp = pt(fx0, yi, 0)
            p.setPen(QPen(QColor("#3a6a3a"), 1))
            p.drawLine(QPointF(tp.x() - 2, tp.y()), QPointF(tp.x() - 7, tp.y()))
            p.setPen(QPen(QColor("#5a9a5a")))
            p.setFont(QFont(MONO_FONT, 6))
            p.drawText(QPointF(tp.x() - 28, tp.y() + 4), f"{yi}m")
            yi += 2

        # ── Scale bar (bottom-right) ──────────────────────────────
        sb_x = W - 110; sb_y = H - 46
        sb_len = int(2 * sx)
        p.setPen(QPen(QColor(SUBTEXT), 2))
        p.drawLine(QPointF(sb_x, sb_y), QPointF(sb_x + sb_len, sb_y))
        p.drawLine(QPointF(sb_x, sb_y - 4), QPointF(sb_x, sb_y + 4))
        p.drawLine(QPointF(sb_x + sb_len, sb_y - 4), QPointF(sb_x + sb_len, sb_y + 4))
        p.setFont(QFont(MONO_FONT, 7))
        p.setPen(QPen(QColor(SUBTEXT)))
        p.drawText(QPointF(sb_x + sb_len / 2 - 8, sb_y - 7), "2 m")

        # ── Legend (bottom-left, two lines so nothing overlaps) ────
        leg_items = [
            ("⛔ RESTRICTED UNIT", CHAMBER_GLOW),
            ("▲ TRACKED PERSON",      PHOSPHOR),
            ("● POINT CLOUD",         DIM_GREEN),
        ]
        lx = 16; ly = H - 46
        for lbl, col in leg_items:
            p.setPen(QPen(QColor(col)))
            p.setFont(QFont(MONO_FONT, 7))
            p.drawText(QPointF(lx, ly), lbl)
            lx += p.fontMetrics().horizontalAdvance(lbl) + 20
            if lx > W // 2:
                lx = 16; ly += 16

        # ── 14. Border ─────────────────────────────────────────────
        p.setPen(QPen(QColor(PHOSPHOR), 1))
        p.setBrush(Qt.NoBrush)
        p.drawRect(QRectF(1, 1, W - 2, H - 2))

        # Bottom info bar separator
        p.setPen(QPen(QColor(BORDER), 1))
        p.drawLine(1, H - 68, W - 1, H - 68)

        # ── Rotation hint (top-right corner) ───────────────────────
        hint_x = W - 16
        hint_y = 58
        p.setFont(QFont(MONO_FONT, 7))
        ang_deg   = math.degrees(self._view_ang)
        depth_val = self._view_depth
        zoom_val  = self._zoom
        for i, (txt, val) in enumerate([
            (f"ANG  {ang_deg:+.0f}°",      None),
            (f"DEPTH {depth_val:.2f}",       None),
            (f"ZOOM  {zoom_val:.2f}×",       None),
        ]):
            p.setPen(QPen(QColor(SUBTEXT)))
            fw = p.fontMetrics().horizontalAdvance(txt)
            p.drawText(QPointF(hint_x - fw, hint_y + i * 13), txt)

        # Interaction guide
        guide_lines = [
            "⟵⟶  drag  =  rotate",
            "↑↓  drag  =  tilt",
            "RMB drag  =  depth / zoom",
            "scroll   =  zoom",
            "dbl-click =  reset",
        ]
        p.setFont(QFont(MONO_FONT, 6))
        for i, gl in enumerate(guide_lines):
            p.setPen(QPen(QColor(SUBTEXT)))
            fw = p.fontMetrics().horizontalAdvance(gl)
            p.drawText(QPointF(hint_x - fw, hint_y + 50 + i * 11), gl)

        p.end()

    def _draw_chamber_3d(self, p, pt_fn, face_fn,
                         chamber, label, breach, tick, WH, sx=40,
                         machine_prox=False):
        """
        Draw one restricted chamber as a full 3D box.
        Faces drawn back-to-front: back, left/right sides, top, front.
        On breach: walls pulse red, flash overlay added.
        On machine_prox: machine block glows orange-white, critical ring drawn.
        """
        x0, x1, y0, y1 = chamber
        ch_h = WH       

        pulse = abs(math.sin(tick * 0.105))
        if breach:
            wall_col   = CHAMBER_GLOW
            wall_alpha = int(200 + pulse * 55)
            fill_base  = "#200000"
            edge_w     = 3
        else:
            wall_col   = CHAMBER_WALL
            wall_alpha = 220
            fill_base  = "#180000"
            edge_w     = 2

        # ── Back face of chamber (y = y1) ─────────────────────────
        back_poly = QPolygonF([
            pt_fn(x0,y1,0), pt_fn(x1,y1,0),
            pt_fn(x1,y1,ch_h), pt_fn(x0,y1,ch_h)
        ])
        fc = QColor(fill_base); fc.setAlpha(230)
        p.setBrush(QBrush(fc))
        p.setPen(QPen(QColor(wall_col), edge_w))
        p.drawPolygon(back_poly)

        # Hazard diagonal stripes on back face
        self._draw_hazard_stripes_face(p, pt_fn,
            [(x0,y1,0),(x1,y1,0),(x1,y1,ch_h),(x0,y1,ch_h)],
            breach)

        # ── Left face of chamber (x = x0) ─────────────────────────
        left_poly = QPolygonF([
            pt_fn(x0,y0,0), pt_fn(x0,y1,0),
            pt_fn(x0,y1,ch_h), pt_fn(x0,y0,ch_h)
        ])
        fc2 = QColor(fill_base); fc2.setAlpha(210)
        p.setBrush(QBrush(fc2))
        p.setPen(QPen(QColor(wall_col), edge_w))
        p.drawPolygon(left_poly)

        # ── Right face of chamber (x = x1) ────────────────────────
        right_poly = QPolygonF([
            pt_fn(x1,y0,0), pt_fn(x1,y1,0),
            pt_fn(x1,y1,ch_h), pt_fn(x1,y0,ch_h)
        ])
        p.setBrush(QBrush(fc2))
        p.setPen(QPen(QColor(wall_col), edge_w))
        p.drawPolygon(right_poly)

        # ── Top face (roof) ────────────────────────────────────────
        top_poly = QPolygonF([
            pt_fn(x0,y0,ch_h), pt_fn(x1,y0,ch_h),
            pt_fn(x1,y1,ch_h), pt_fn(x0,y1,ch_h)
        ])
        if breach:
            rc = QColor("#3a0000"); rc.setAlpha(int(160 + pulse * 80))
        else:
            rc = QColor("#250000"); rc.setAlpha(200)
        p.setBrush(QBrush(rc))
        p.setPen(QPen(QColor(wall_col), edge_w))
        p.drawPolygon(top_poly)

        # Roof hazard pattern
        
        steps = 6
        for i in range(steps):
            t_s = i / steps
            t_e = (i + 0.5) / steps
            a = pt_fn(x0 + (x1-x0)*t_s, y0, ch_h)
            b = pt_fn(x0 + (x1-x0)*t_e, y1, ch_h)
            lc = QColor(wall_col if breach else CHAMBER_WALL)
            lc.setAlpha(60)
            p.setPen(QPen(lc, 1))
            p.drawLine(a, b)

        # ── Front face (y = y0) — door opening ────────────────────
        # Full front fill first
        front_poly = QPolygonF([
            pt_fn(x0,y0,0), pt_fn(x1,y0,0),
            pt_fn(x1,y0,ch_h), pt_fn(x0,y0,ch_h)
        ])
        fc3 = QColor(fill_base); fc3.setAlpha(180)
        p.setBrush(QBrush(fc3))
        p.setPen(QPen(QColor(wall_col), edge_w))
        p.drawPolygon(front_poly)

        # Door opening: erase central section of front face
        door_w = (x1 - x0) * 0.5
        door_xc = (x0 + x1) / 2
        door_h  = ch_h * 0.75
        door_poly = QPolygonF([
            pt_fn(door_xc - door_w/2, y0, 0),
            pt_fn(door_xc + door_w/2, y0, 0),
            pt_fn(door_xc + door_w/2, y0, door_h),
            pt_fn(door_xc - door_w/2, y0, door_h),
        ])
        # Fill door with dark "inside" colour
        p.setBrush(QBrush(QColor("#0a0000")))
        p.setPen(QPen(QColor(wall_col), 1))
        p.drawPolygon(door_poly)

        # Door frame pillars
        pil_w = 0.1
        for dx in [-door_w/2, door_w/2 - pil_w]:
            pil = QPolygonF([
                pt_fn(door_xc+dx, y0, 0),
                pt_fn(door_xc+dx+pil_w, y0, 0),
                pt_fn(door_xc+dx+pil_w, y0, door_h),
                pt_fn(door_xc+dx, y0, door_h),
            ])
            fc4 = QColor(wall_col); fc4.setAlpha(200)
            p.setBrush(QBrush(fc4))
            p.setPen(Qt.NoPen)
            p.drawPolygon(pil)

        
        mx, my = (x0+x1)/2, (y0+y1)/2 + 0.5
        mh, mw = 1.2, 0.7

        # ── Machine proximity critical glow (drawn BEFORE machine body) ──
        if machine_prox:
            prox_pulse = abs(math.sin(tick * 0.18))   # faster pulse for critical
            # Danger ring on floor around machine
            mid_base = pt_fn(mx, my, 0)
            ring_rx  = max(6, int(sx * 0.55))
            for extra, alpha_base in [(0, 180), (10, 100), (20, 50)]:
                rc = QColor("#ff8c00")
                rc.setAlpha(int((alpha_base + prox_pulse * 60)))
                p.setPen(QPen(rc, 2))
                p.setBrush(Qt.NoBrush)
                p.drawEllipse(mid_base, ring_rx + extra, max(3, (ring_rx + extra) // 3))
            # Critical label above machine
            crit_pt = pt_fn(mx, my - mw/2 - 0.1, mh + 0.5)
            cc = QColor("#ff6600"); cc.setAlpha(int(200 + prox_pulse * 55))
            p.setPen(QPen(cc))
            p.setFont(QFont(MONO_FONT, 9, QFont.Bold))
            crit_str = "!! MACHINE HAZARD !!"
            cw = p.fontMetrics().horizontalAdvance(crit_str)
            p.drawText(QPointF(crit_pt.x() - cw/2, crit_pt.y()), crit_str)

        # Machine body — colour shifts orange-white when proximity active
        if machine_prox:
            mach_top_fill   = "#030119"
            mach_top_edge   = "#063403ac"
            mach_front_fill = "#021c15"
            mach_front_edge = "#cc4400"
            mach_side_fill  = "#200a00"
            mach_side_edge  = "#992200"
        else:
            mach_top_fill   = "#969FAEDF"
            mach_top_edge   = "#000000"
            mach_front_fill = "#969FAEDF"
            mach_front_edge = "#000000"
            mach_side_fill  = "#969FAEDF"
            mach_side_edge  = "#000000"

        # machine top
        face_fn([(mx-mw/2,my-mw/2,mh),(mx+mw/2,my-mw/2,mh),
                 (mx+mw/2,my+mw/2,mh),(mx-mw/2,my+mw/2,mh)],
                mach_top_fill, 220, mach_top_edge, 2 if machine_prox else 1)
        # machine front
        face_fn([(mx-mw/2,my-mw/2,0),(mx+mw/2,my-mw/2,0),
                 (mx+mw/2,my-mw/2,mh),(mx-mw/2,my-mw/2,mh)],
                mach_front_fill, 210, mach_front_edge, 2 if machine_prox else 1)
        # machine right side
        face_fn([(mx+mw/2,my-mw/2,0),(mx+mw/2,my+mw/2,0),
                 (mx+mw/2,my+mw/2,mh),(mx+mw/2,my-mw/2,mh)],
                mach_side_fill, 200, mach_side_edge, 2 if machine_prox else 1)

        # Machine label — turns orange on proximity
        ml_pt = pt_fn(mx, my - mw/2 - 0.05, mh + 0.1)
        p.setPen(QPen(QColor("#ff6600" if machine_prox else "#6a2020")))
        p.setFont(QFont(MONO_FONT, 6, QFont.Bold if machine_prox else QFont.Normal))
        p.drawText(QPointF(ml_pt.x() - 16, ml_pt.y()), "MACHINE")

        # ── Corner accent marks ────────────────────────────────────
        ca_c = QColor(wall_col); ca_c.setAlpha(wall_alpha)
        p.setPen(QPen(ca_c, 2))
        ca_len = max(8, int(sx * 0.3)) if hasattr(self, '_last_sx') else 10
        for (cx_c, cy_c, cz_c, da, db) in [
            (x0, y0, 0,    (1,0,0),  (0,0,1)),
            (x1, y0, 0,   (-1,0,0),  (0,0,1)),
            (x0, y0, ch_h, (1,0,0),  (0,0,-1)),
            (x1, y0, ch_h,(-1,0,0),  (0,0,-1)),
        ]:
            scr = pt_fn(cx_c, cy_c, cz_c)
            # We approximate corner marks as small screen-space lines
            # direction vectors mapped via partial projection
            # (simplified: just draw fixed-pixel lines from corner)
            pass  # corner marks skipped for clarity — wall outline covers it

        # ── Restricted label on top face ───────────────────────────
        lbl_ctr = pt_fn((x0+x1)/2, (y0+y1)/2, ch_h + 0.12)
        if breach:
            lc = QColor(CHAMBER_GLOW)
            lc.setAlpha(int(180 + pulse * 75))
        else:
            lc = QColor("#aa2222")
        p.setPen(QPen(lc))
        p.setFont(QFont(MONO_FONT, 8, QFont.Bold))
        lbl_str = "⛔ RESTRICTED"
        lw = p.fontMetrics().horizontalAdvance(lbl_str)
        p.drawText(QPointF(lbl_ctr.x() - lw/2, lbl_ctr.y()), lbl_str)

        p.setFont(QFont(MONO_FONT, 7, QFont.Bold))
        lbl2_w = p.fontMetrics().horizontalAdvance(label)
        p.drawText(QPointF(lbl_ctr.x() - lbl2_w/2, lbl_ctr.y() + 12), label)

        # ── Breach flash overlay ───────────────────────────────────
        if breach:
            flash_a = int(pulse * 50)
            for poly in [back_poly, left_poly, right_poly, front_poly, top_poly]:
                fl = QColor("#ff0000"); fl.setAlpha(flash_a)
                p.setBrush(QBrush(fl)); p.setPen(Qt.NoPen)
                p.drawPolygon(poly)

            # Pulsing glow rings around whole chamber base
            safe_sx = max(1.0, float(sx))
            ring_rx = max(4, int(safe_sx * (x1 - x0) / 2))
            for radius_extra in [12, 22]:
                mid_base = pt_fn((x0+x1)/2, (y0+y1)/2, 0)
                ring_c = QColor(CHAMBER_GLOW)
                ring_c.setAlpha(int(pulse * 80))
                p.setPen(QPen(ring_c, 2))
                p.setBrush(Qt.NoBrush)
                p.drawEllipse(mid_base, ring_rx + radius_extra,
                              max(2, int(radius_extra * 0.5)))

            # BREACH text above chamber
            bc_c = QColor(CHAMBER_GLOW); bc_c.setAlpha(int(180 + pulse * 75))
            p.setPen(QPen(bc_c))
            p.setFont(QFont(MONO_FONT, 10, QFont.Bold))
            breach_str = "! UNAUTHORIZED ENTRY DETECTED !"
            bw = p.fontMetrics().horizontalAdvance(breach_str)
            top_ctr = pt_fn((x0+x1)/2, (y0+y1)/2, ch_h + 0.4)
            p.drawText(QPointF(top_ctr.x() - bw/2, top_ctr.y()), breach_str)

    def _draw_hazard_stripes_face(self, p, pt_fn, corners_xyz, breach):
        """Draw diagonal hazard stripes on a given face (clipped to face polygon)."""
        poly = QPolygonF([pt_fn(*c) for c in corners_xyz])
        p.setClipRegion(p.clipRegion())  # keep current clip
        path = QPainterPath()
        for i, pt_ in enumerate(poly):
            if i == 0:
                path.moveTo(pt_)
            else:
                path.lineTo(pt_)
        path.closeSubpath()
        p.setClipPath(path)

        stripe_c = QColor("#3a0000" if breach else "#280000")
        stripe_c.setAlpha(90)
        p.setPen(QPen(stripe_c, 4))
        # Get bounding rect of face in screen coords
        xs = [pt_fn(*c).x() for c in corners_xyz]
        ys = [pt_fn(*c).y() for c in corners_xyz]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        span = max(max_x - min_x, max_y - min_y)
        stripe_gap = max(8, int(span / 8))
        for i in range(-int(span / stripe_gap), int(span / stripe_gap) + 2):
            ox_ = min_x + i * stripe_gap * 2
            p.drawLine(QPointF(ox_,               min_y - 10),
                       QPointF(ox_ + stripe_gap,   max_y + 10))
        p.setClipping(False)


# ══════════════════════════════════════════════════════════════════
#  LEGACY TOP-VIEW  RADAR FIELD mini widget)
# ══════════════════════════════════════════════════════════════════
class RadarViewWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(440)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setStyleSheet(f"background:{BG};")
        self.safe_r       = 3.0
        self.alert_r      = 5.0
        self.restricted_r = 7.5
        self.view_range = 9.0
        self._points  = np.empty((0, 4), dtype=np.float32)
        self._targets = []
        self._persons = {}

    def update_scene(self, points, targets, persons):
        self._points  = points
        self._targets = targets
        self._persons = persons
        self.update()

    def refresh_zones(self):
        self.update()

    def _world_to_px(self, x_m, y_m, cx, cy, scale):
        px = cx + x_m * scale
        py = cy - y_m * scale
        return QPointF(px, py)

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        W = self.width(); H = self.height()
        cx   = W / 2
        cy   = H * 0.90
        scale = (H * 0.88) / self.view_range
        p.fillRect(0, 0, W, H, QColor(BG))
        pen_grid = QPen(QColor(BORDER)); pen_grid.setWidth(1)
        pen_grid.setStyle(Qt.DotLine)
        p.setPen(pen_grid)
        font_small = QFont(MONO_FONT, 7)
        p.setFont(font_small)
        for d in range(1, int(self.view_range) + 1):
            r = d * scale
            rect = QRectF(cx - r, cy - r, 2*r, 2*r)
            p.drawArc(rect, 0 * 16, 180 * 16)
            p.setPen(QPen(QColor(SUBTEXT)))
            p.drawText(QPointF(cx + 4, cy - r + 4), f"{d}m")
            p.setPen(pen_grid)
        for angle_deg in range(0, 181, 30):
            a = math.radians(angle_deg)
            ex = cx + self.view_range * scale * math.cos(math.pi - a)
            ey = cy - self.view_range * scale * math.sin(a)
            p.setPen(pen_grid)
            p.drawLine(QPointF(cx, cy), QPointF(ex, ey))
        zones = [
            (self.safe_r,       PHOSPHOR,  "SAFE",       2),
            (self.alert_r,      AMBER,     "ALERT",      2),
            (self.restricted_r, RED_ALERT, "RESTRICTED", 2),
        ]
        for radius, color_hex, label, lw in zones:
            r = radius * scale
            pen_z = QPen(QColor(color_hex)); pen_z.setWidth(lw)
            pen_z.setStyle(Qt.DashLine)
            p.setPen(pen_z)
            rect = QRectF(cx - r, cy - r, 2*r, 2*r)
            p.drawArc(rect, 0 * 16, 180 * 16)
            tip_x = cx
            tip_y = cy - r - 4
            p.setPen(QPen(QColor(color_hex)))
            p.setFont(QFont(MONO_FONT, 7, QFont.Bold))
            p.drawText(QPointF(tip_x - 28, tip_y), f"{label} ({radius:.1f}m)")
        if len(self._points) > 0:
            p.setPen(Qt.NoPen)
            for pt in self._points:
                px_pt = self._world_to_px(pt[0], pt[1], cx, cy, scale)
                z_norm = min(max(pt[2] / 3.0, 0.0), 1.0)
                g = int(77 + z_norm * 178)
                p.setBrush(QBrush(QColor(0, g, 0, 160)))
                p.drawEllipse(px_pt, 3, 3)
        for t in self._targets:
            px_t = self._world_to_px(t["x"], t["y"], cx, cy, scale)
            per  = self._persons.get(t["id"])
            dist = math.sqrt(t["x"]**2 + t["y"]**2)
            if dist > self.restricted_r:
                col = QColor(RED_ALERT)
            elif dist > self.alert_r:
                col = QColor(AMBER)
            else:
                col = QColor(PHOSPHOR)
            tri_size = 8
            tri = QPolygonF([
                QPointF(px_t.x(),              px_t.y() - tri_size),
                QPointF(px_t.x() - tri_size*0.7, px_t.y() + tri_size*0.5),
                QPointF(px_t.x() + tri_size*0.7, px_t.y() + tri_size*0.5),
            ])
            p.setPen(QPen(col, 1))
            p.setBrush(QBrush(col))
            p.drawPolygon(tri)
            p.setPen(QPen(QColor(WHITE_TEXT)))
            p.setFont(QFont(MONO_FONT, 8, QFont.Bold))
            p.drawText(QPointF(px_t.x() + 10, px_t.y() + 4), f"#{t['id']}")
        p.setPen(QPen(QColor(PHOSPHOR), 2))
        p.setBrush(QBrush(QColor(PHOSPHOR)))
        p.drawEllipse(QPointF(cx, cy), 5, 5)
        p.setPen(QPen(QColor(PHOSPHOR)))
        p.setFont(QFont(MONO_FONT, 7, QFont.Bold))
        p.drawText(QPointF(cx + 8, cy + 4), "SENSOR")
        p.setPen(QPen(QColor(PHOSPHOR)))
        p.setFont(QFont(MONO_FONT, 9, QFont.Bold))
        p.drawText(10, 20, "RADAR FIELD  —  IWR6843AOP EVM  (Top View)")
        p.end()


# ══════════════════════════════════════════════════════════════════
#  ZONE LINE CHART  
# ══════════════════════════════════════════════════════════════════
class ZoneChartWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(320)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setStyleSheet(f"background:{PANEL};border-radius:4px;")
        self.history = []

    def set_history(self, history):
        self.history = history
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        W = self.width(); H = self.height()
        PAD_L = 40; PAD_R = 16; PAD_T = 30; PAD_B = 50
        chart_w = W - PAD_L - PAD_R
        chart_h = H - PAD_T - PAD_B
        p.fillRect(0, 0, W, H, QColor(PANEL))
        p.setPen(QPen(QColor(SUBTEXT)))
        p.setFont(QFont(MONO_FONT, 8))
        p.drawText(PAD_L, PAD_T - 8, "People per zone  (live session)")
        if not self.history:
            p.setPen(QPen(QColor(SUBTEXT)))
            p.setFont(QFont(MONO_FONT, 8))
            p.drawText(PAD_L + chart_w//2 - 60, PAD_T + chart_h//2, "Waiting for data…")
            p.end(); return
        n      = len(self.history)
        all_v  = [v for h in self.history for v in h]
        y_max  = max(max(all_v) if all_v else 0, 2) + 1
        def to_px(i, val):
            x = PAD_L + (i / max(n - 1, 1)) * chart_w if n > 1 else PAD_L + chart_w / 2
            y = PAD_T + chart_h - (val / y_max) * chart_h
            return QPointF(x, y)
        pen_ax = QPen(QColor(BORDER)); pen_ax.setWidth(1)
        p.setPen(pen_ax)
        p.drawLine(PAD_L, PAD_T, PAD_L, PAD_T + chart_h)
        p.drawLine(PAD_L, PAD_T + chart_h, PAD_L + chart_w, PAD_T + chart_h)
        data_max = max(max(all_v) if all_v else 0, 1)
        p.setFont(QFont(MONO_FONT, 7))
        for tick in range(0, data_max + 2):
            yp = PAD_T + chart_h - (tick / y_max) * chart_h
            if yp < PAD_T or yp > PAD_T + chart_h + 2:
                continue
            p.setPen(QPen(QColor(BORDER)))
            p.drawLine(PAD_L, int(yp), PAD_L + chart_w, int(yp))
            p.setPen(QPen(QColor(SUBTEXT)))
            p.drawText(QPointF(2, yp + 4), str(tick))
        tick_step = max(1, n // 10)
        for i in range(0, n, tick_step):
            xp = PAD_L + (i / max(n - 1, 1)) * chart_w if n > 1 else PAD_L + chart_w / 2
            p.setPen(QPen(QColor(SUBTEXT)))
            p.setFont(QFont(MONO_FONT, 7))
            p.drawText(QPointF(xp - 8, PAD_T + chart_h + 14), f"{i+1}")
        series = [
            (0, PHOSPHOR,  "Safe"),
            (1, AMBER,     "Alert"),
            (2, RED_ALERT, "Restricted"),
        ]
        for idx, color_hex, label in series:
            vals = [h[idx] for h in self.history]
            pts_xy = [to_px(i, vals[i]) for i in range(n)]
            pen_l = QPen(QColor(color_hex)); pen_l.setWidth(2)
            pen_l.setCapStyle(Qt.RoundCap); pen_l.setJoinStyle(Qt.RoundJoin)
            p.setPen(pen_l)
            for i in range(len(pts_xy) - 1):
                if vals[i] > 0 or vals[i + 1] > 0:
                    p.drawLine(pts_xy[i], pts_xy[i + 1])
            for i, pt in enumerate(pts_xy):
                if vals[i] > 0:
                    p.setPen(QPen(QColor("#ffffff"), 1))
                    p.setBrush(QBrush(QColor(color_hex)))
                    p.drawEllipse(pt, 6, 6)
                    p.setPen(QPen(QColor(color_hex)))
                    p.setFont(QFont(MONO_FONT, 7, QFont.Bold))
                    p.drawText(QPointF(pt.x() - 4, pt.y() - 9), str(vals[i]))
                else:
                    p.setPen(Qt.NoPen)
                    p.setBrush(QBrush(QColor(BORDER)))
                    p.drawEllipse(pt, 2, 2)
            legend_x = PAD_L + chart_w - 90
            legend_y = PAD_T + 14 * (idx + 1)
            p.drawEllipse(QPointF(legend_x, legend_y), 4, 4)
            p.setPen(QPen(QColor(color_hex)))
            p.setFont(QFont(MONO_FONT, 7))
            p.drawText(QPointF(legend_x + 8, legend_y + 4), label)
        p.end()


# ══════════════════════════════════════════════════════════════════
#  DUAL 3D VIEW  
# ══════════════════════════════════════════════════════════════════
class DualViewWidget(QWidget):
    _ZONE_DEF = [
        ("SAFE",       "#39ff14", "#00cc44"),
        ("ALERT",      "#ffb300", "#cc7700"),
        ("RESTRICTED", "#ff3030", "#cc1010"),
    ]
    
    CHAMBER_A = (-4.5, -2.0, 3.5, 7.0)
    CHAMBER_B = ( 2.0,  4.5, 3.5, 7.0)

    _ANG   = math.radians(26)
    _DEPTH = 0.46

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(440)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setStyleSheet("background:#040904;")
        self.setMouseTracking(True)
        self.safe_r       = 3.0
        self.alert_r      = 5.0
        self.restricted_r = 7.5
        self.view_range   = 9.0
        self.zone         = None
        self._points  = np.empty((0, 4), dtype=np.float32)
        self._targets = []
        self._persons = {}

        # ── Interactive rotation / zoom per-pane ──────────────────
        # Oblique angle for depth axis (shared between both panes)
        self._view_ang   = math.radians(26)
        # Depth foreshortening
        self._view_depth = 0.46
        # Zoom
        self._zoom       = 1.0

        self._drag_btn   = None
        self._drag_start = None
        self._ang_start  = None
        self._depth_start= None
        self.setCursor(Qt.OpenHandCursor)

    def mousePressEvent(self, ev):
        self._drag_btn    = ev.button()
        self._drag_start  = ev.pos()
        self._ang_start   = self._view_ang
        self._depth_start = self._view_depth
        self.setCursor(Qt.ClosedHandCursor)
        ev.accept()

    def mouseMoveEvent(self, ev):
        if self._drag_start is None:
            return
        dx = ev.pos().x() - self._drag_start.x()
        dy = ev.pos().y() - self._drag_start.y()

        if self._drag_btn == Qt.LeftButton:
            # Horizontal → rotate oblique angle
            self._view_ang = max(math.radians(-70),
                                 min(math.radians(80),
                                     self._ang_start + dx * 0.007))
            # Vertical → depth foreshortening
            self._view_depth = max(0.10,
                                   min(1.20,
                                       self._depth_start - dy * 0.004))
        elif self._drag_btn == Qt.RightButton:
            # Right drag → zoom
            self._zoom = max(0.3, min(3.0,
                                      self._zoom * (1 - dy * 0.003)))
        self.update()
        ev.accept()

    def mouseReleaseEvent(self, ev):
        self._drag_btn   = None
        self._drag_start = None
        self.setCursor(Qt.OpenHandCursor)
        ev.accept()

    def mouseDoubleClickEvent(self, ev):
        """Double-click resets to default view."""
        self._view_ang   = math.radians(26)
        self._view_depth = 0.46
        self._zoom       = 1.0
        self.update()
        ev.accept()

    def wheelEvent(self, ev):
        delta = ev.angleDelta().y()
        self._zoom = max(0.3, min(3.0,
                                  self._zoom * (1 + delta / 1200.0)))
        self.update()
        ev.accept()

    def update_scene(self, points, targets, persons):
        self._points  = points
        self._targets = targets
        self._persons = persons
        self.update()

    def refresh_zones(self):
        self.update()

    def _in_chamber(self, x, y):
        """Return 'A', 'B', or None depending on which chamber (x,y) is inside."""
        ax0, ax1, ay0, ay1 = self.CHAMBER_A
        bx0, bx1, by0, by1 = self.CHAMBER_B
        if ax0 <= x <= ax1 and ay0 <= y <= ay1:
            return "A"
        if bx0 <= x <= bx1 and by0 <= y <= by1:
            return "B"
        return None

    def _zone_idx(self, dist):
        if dist > self.restricted_r: return 2
        if dist > self.alert_r:      return 1
        return 0

    def _proj(self, x_m, y_m, z_m, ox, oy, sx, sz, dc, ds):
        return QPointF(
            ox + x_m * sx + y_m * dc,
            oy - z_m * sz - y_m * ds
        )

    def _draw_pane(self, painter, rect, title,
                   h_min, h_max, d_max, v_max,
                   h_label, d_label,
                   zone_d_limits, hz_box, pt_list, person_list,
                   chamber_bands=None):
        
        
        
        p = painter
        p.setClipRect(rect)
        rw = rect.width();  rh = rect.height()
        ML = 46; MR = 20; MT = 32; MB = 44
        DW = rw - ML - MR
        DH = rh - MT - MB
        h_span = h_max - h_min
        # Use instance rotation/zoom state instead of fixed class constants
        dc_raw = math.cos(self._view_ang) * self._view_depth
        ds_raw = math.sin(self._view_ang) * self._view_depth
        # Guard divide-by-zero for extreme angles
        denom_w = max(h_span + d_max * abs(dc_raw), 0.1)
        denom_h = max(v_max  + d_max * abs(ds_raw), 0.1)
        sx_base = min(DW / denom_w, DH / denom_h)
        sx = sx_base * self._zoom
        sz = sx
        dc = dc_raw * sx
        ds = ds_raw * sz
        box_screen_w = h_span * sx + d_max * dc
        box_screen_h = v_max  * sz + d_max * ds
        ox = rect.x() + ML + (DW - box_screen_w) / 2 - h_min * sx
        oy = rect.y() + MT + (DH - box_screen_h) / 2 + box_screen_h

        def pt(h, d, v):
            return self._proj(h, d, v, ox, oy, sx, sz, dc, ds)

        p.setBrush(QBrush(QColor("#040904")))
        p.setPen(Qt.NoPen)
        p.drawRect(rect)

        p.setPen(QPen(QColor(PHOSPHOR)))
        p.setFont(QFont(MONO_FONT, 9, QFont.Bold))
        p.drawText(QPointF(rect.x() + ML, rect.y() + 18), title)

        floor_poly = QPolygonF([
            pt(h_min,0,0), pt(h_max,0,0),
            pt(h_max,d_max,0), pt(h_min,d_max,0)
        ])
        p.setBrush(QBrush(QColor("#0a120a"))); p.setPen(Qt.NoPen)
        p.drawPolygon(floor_poly)

        for zi, (d_near, d_far) in enumerate(zone_d_limits):
            if d_near >= d_max: continue
            d_f = min(d_far, d_max)
            lbl, wc, fc = self._ZONE_DEF[zi]
            band_poly = QPolygonF([
                pt(h_min, d_near, 0), pt(h_max, d_near, 0),
                pt(h_max, d_f,    0), pt(h_min, d_f,    0)
            ])
            fill = QColor(fc); fill.setAlpha(38)
            p.setBrush(QBrush(fill)); p.setPen(Qt.NoPen)
            p.drawPolygon(band_poly)
            if d_f < d_max + 0.05:
                pen_z = QPen(QColor(wc), 1, Qt.DashLine)
                pen_z.setDashPattern([8, 4])
                p.setPen(pen_z)
                p.drawLine(pt(h_min, d_f, 0), pt(h_max, d_f, 0))
                mid_d = (d_near + d_f) / 2
                label_pt = QPointF(
                    (pt(h_min, mid_d, 0).x() + pt(h_max, mid_d, 0).x()) / 2 - 20,
                    (pt(h_min, mid_d, 0).y() + pt(h_max, mid_d, 0).y()) / 2 + 3
                )
                p.setPen(QPen(QColor(wc)))
                p.setFont(QFont(MONO_FONT, 7, QFont.Bold))
                p.drawText(label_pt, f"{lbl}")

        for dv in [i * 1.0 for i in range(0, int(d_max) + 2)]:
            if dv > d_max + 0.01: break
            pen = QPen(QColor("#1e301e"), 1, Qt.SolidLine if dv==round(dv) else Qt.DotLine)
            p.setPen(pen)
            p.drawLine(pt(h_min, dv, 0), pt(h_max, dv, 0))
        for hv in [h_min + i * 1.0 for i in range(0, int(h_span) + 2)]:
            if hv > h_max + 0.01: break
            pen = QPen(QColor("#1a281a"), 1, Qt.DotLine)
            p.setPen(pen)
            p.drawLine(pt(hv, 0, 0), pt(hv, d_max, 0))

        left_poly = QPolygonF([
            pt(h_min,0,0),     pt(h_min,d_max,0),
            pt(h_min,d_max,v_max), pt(h_min,0,v_max)
        ])
        p.setBrush(QBrush(QColor("#080e08"))); p.setPen(Qt.NoPen)
        p.drawPolygon(left_poly)

        for zi, (d_near, d_far) in enumerate(zone_d_limits):
            if d_near >= d_max: continue
            d_f = min(d_far, d_max)
            lbl, wc, fc = self._ZONE_DEF[zi]
            stripe = QPolygonF([
                pt(h_min, d_near, 0),     pt(h_min, d_f, 0),
                pt(h_min, d_f, v_max),    pt(h_min, d_near, v_max)
            ])
            fill = QColor(fc); fill.setAlpha(22)
            p.setBrush(QBrush(fill)); p.setPen(Qt.NoPen)
            p.drawPolygon(stripe)

        for dv in [i * 1.0 for i in range(0, int(d_max) + 2)]:
            if dv > d_max + 0.01: break
            pen = QPen(QColor("#182818" if dv==round(dv) else "#111c11"), 1,
                       Qt.SolidLine if dv==round(dv) else Qt.DotLine)
            p.setPen(pen)
            p.drawLine(pt(h_min,dv,0), pt(h_min,dv,v_max))
        for vv in [i * 0.5 for i in range(0, int(v_max/0.5)+2)]:
            if vv > v_max + 0.01: break
            pen = QPen(QColor("#1e2e1e" if vv==round(vv) else "#141e14"), 1,
                       Qt.SolidLine if vv==round(vv) else Qt.DotLine)
            p.setPen(pen)
            p.drawLine(pt(h_min,0,vv), pt(h_min,d_max,vv))
        p.setPen(QPen(QColor("#2a4a2a"), 1))
        p.setBrush(Qt.NoBrush)
        p.drawPolygon(left_poly)

        back_poly = QPolygonF([
            pt(h_min,d_max,0),    pt(h_max,d_max,0),
            pt(h_max,d_max,v_max),pt(h_min,d_max,v_max)
        ])
        p.setBrush(QBrush(QColor("#0a120a"))); p.setPen(Qt.NoPen)
        p.drawPolygon(back_poly)

        for hv in [h_min + i*1.0 for i in range(0, int(h_span)+2)]:
            if hv > h_max + 0.01: break
            pen = QPen(QColor("#1e2e1e"), 1, Qt.DotLine)
            p.setPen(pen)
            p.drawLine(pt(hv,d_max,0), pt(hv,d_max,v_max))
        for vv in [i*0.5 for i in range(0, int(v_max/0.5)+2)]:
            if vv > v_max + 0.01: break
            pen = QPen(QColor("#1e2e1e" if vv==round(vv) else "#141e14"), 1,
                       Qt.SolidLine if vv==round(vv) else Qt.DotLine)
            p.setPen(pen)
            p.drawLine(pt(h_min,d_max,vv), pt(h_max,d_max,vv))
        p.setPen(QPen(QColor("#2a4a2a"), 1))
        p.setBrush(Qt.NoBrush)
        p.drawPolygon(back_poly)

        p.setPen(QPen(QColor("#2a4a2a"), 1))
        for a, b in [
            (pt(h_max,0,0),     pt(h_max,d_max,0)),
            (pt(h_max,0,0),     pt(h_max,0,v_max)),
            (pt(h_max,d_max,0), pt(h_max,d_max,v_max)),
            (pt(h_max,0,v_max), pt(h_max,d_max,v_max)),
        ]:
            p.drawLine(a, b)

        p.setPen(QPen(QColor("#2a4a2a"), 1))
        for a, b in [
            (pt(h_min,0,v_max),     pt(h_max,0,v_max)),
            (pt(h_min,d_max,v_max), pt(h_max,d_max,v_max)),
            (pt(h_min,0,v_max),     pt(h_min,d_max,v_max)),
            (pt(h_max,0,v_max),     pt(h_max,d_max,v_max)),
        ]:
            p.drawLine(a, b)

        front_poly = QPolygonF([
            pt(h_min,0,0), pt(h_max,0,0),
            pt(h_max,0,v_max), pt(h_min,0,v_max)
        ])
        p.setBrush(QBrush(QColor("#0c160c"))); p.setPen(Qt.NoPen)
        p.drawPolygon(front_poly)

        for hv in [h_min + i*1.0 for i in range(0, int(h_span)+2)]:
            if hv > h_max+0.01: break
            is_zero = abs(hv) < 0.01
            pen = QPen(QColor("#3a5a3a" if is_zero else "#243824"), 1, Qt.SolidLine)
            p.setPen(pen)
            p.drawLine(pt(hv,0,0), pt(hv,0,v_max))
        for vv in [i*0.5 for i in range(0, int(v_max/0.5)+2)]:
            if vv > v_max+0.01: break
            pen = QPen(QColor("#2e4e2e" if vv==round(vv) else "#1e301e"), 1,
                       Qt.SolidLine if vv==round(vv) else Qt.DotLine)
            p.setPen(pen)
            p.drawLine(pt(h_min,0,vv), pt(h_max,0,vv))
        p.setPen(QPen(QColor("#3a6a3a"), 2))
        p.setBrush(Qt.NoBrush)
        p.drawPolygon(front_poly)

        if hz_box is not None:
            hx0,hx1,hy0,hy1,hz0,hz1 = hz_box
            hx0=max(hx0,h_min); hx1=min(hx1,h_max)
            hy0=max(hy0,0);     hy1=min(hy1,d_max)
            hz0=max(hz0,0);     hz1=min(hz1,v_max)
            hz_col = QColor("#cc1010")
            hz_edge= QColor("#ff3030")
            hz_faces = [
                QPolygonF([pt(hx0,hy1,hz0),pt(hx1,hy1,hz0),
                           pt(hx1,hy1,hz1),pt(hx0,hy1,hz1)]),
                QPolygonF([pt(hx0,hy0,hz0),pt(hx0,hy1,hz0),
                           pt(hx0,hy1,hz1),pt(hx0,hy0,hz1)]),
                QPolygonF([pt(hx1,hy0,hz0),pt(hx1,hy1,hz0),
                           pt(hx1,hy1,hz1),pt(hx1,hy0,hz1)]),
                QPolygonF([pt(hx0,hy0,hz0),pt(hx1,hy0,hz0),
                           pt(hx1,hy1,hz0),pt(hx0,hy1,hz0)]),
                QPolygonF([pt(hx0,hy0,hz1),pt(hx1,hy0,hz1),
                           pt(hx1,hy1,hz1),pt(hx0,hy1,hz1)]),
                QPolygonF([pt(hx0,hy0,hz0),pt(hx1,hy0,hz0),
                           pt(hx1,hy0,hz1),pt(hx0,hy0,hz1)]),
            ]
            for face in hz_faces:
                fill = QColor(hz_col); fill.setAlpha(38)
                p.setBrush(QBrush(fill))
                p.setPen(QPen(hz_edge, 1))
                p.drawPolygon(face)
            top_mid = QPointF(
                (pt(hx0,hy0,hz1).x()+pt(hx1,hy1,hz1).x())/2 - 24,
                min(pt(hx0,hy0,hz1).y(), pt(hx1,hy1,hz1).y()) - 6
            )
            if rect.y()+MT < top_mid.y() < rect.y()+rh-MB:
                p.setPen(QPen(hz_edge))
                p.setFont(QFont(MONO_FONT, 8, QFont.Bold))
                p.drawText(top_mid, "● HAZARD")

        # ── Chamber footprints and full-height wireframe boxes ─────
        if chamber_bands:
            for (ch0, ch1, cd0, cd1, ch_lbl) in chamber_bands:
                # Clamp to pane visible range
                ch0c = max(ch0, h_min); ch1c = min(ch1, h_max)
                cd0c = max(cd0, 0.0);   cd1c = min(cd1, d_max)
                if ch0c >= ch1c or cd0c >= cd1c:
                    continue   # chamber fully outside this pane's view

                # Floor footprint (filled red rectangle on floor, z=0)
                floor_band = QPolygonF([
                    pt(ch0c, cd0c, 0), pt(ch1c, cd0c, 0),
                    pt(ch1c, cd1c, 0), pt(ch0c, cd1c, 0),
                ])
                fc = QColor(CHAMBER_GLOW); fc.setAlpha(28)
                p.setBrush(QBrush(fc))
                p.setPen(Qt.NoPen)
                p.drawPolygon(floor_band)

                # Full-height wireframe box (all 12 edges)
                pen_ch = QPen(QColor(CHAMBER_GLOW), 1, Qt.DashLine)
                pen_ch.setDashPattern([6, 3])
                p.setPen(pen_ch)
                p.setBrush(Qt.NoBrush)
                ch_h = v_max  # draw to full height of the pane
                # Bottom rect
                p.drawPolygon(QPolygonF([
                    pt(ch0c,cd0c,0), pt(ch1c,cd0c,0),
                    pt(ch1c,cd1c,0), pt(ch0c,cd1c,0)]))
                # Top rect
                p.drawPolygon(QPolygonF([
                    pt(ch0c,cd0c,ch_h), pt(ch1c,cd0c,ch_h),
                    pt(ch1c,cd1c,ch_h), pt(ch0c,cd1c,ch_h)]))
                # Vertical edges (4 pillars)
                for hv, dv in [(ch0c,cd0c),(ch1c,cd0c),(ch1c,cd1c),(ch0c,cd1c)]:
                    p.drawLine(pt(hv,dv,0), pt(hv,dv,ch_h))

                # Solid bright outline on front face of chamber box
                pen_solid = QPen(QColor(CHAMBER_GLOW), 2)
                p.setPen(pen_solid)
                p.drawLine(pt(ch0c,cd0c,0),    pt(ch1c,cd0c,0))
                p.drawLine(pt(ch0c,cd0c,0),    pt(ch0c,cd0c,ch_h))
                p.drawLine(pt(ch1c,cd0c,0),    pt(ch1c,cd0c,ch_h))
                p.drawLine(pt(ch0c,cd0c,ch_h), pt(ch1c,cd0c,ch_h))

                # "⛔ CHAMBER X" label at top-front edge
                lbl_pt = QPointF(
                    (pt(ch0c,cd0c,ch_h).x() + pt(ch1c,cd0c,ch_h).x()) / 2 - 28,
                    min(pt(ch0c,cd0c,ch_h).y(), pt(ch1c,cd0c,ch_h).y()) - 5
                )
                if rect.y() + MT < lbl_pt.y() < rect.y() + rh - MB:
                    p.setPen(QPen(QColor(CHAMBER_GLOW)))
                    p.setFont(QFont(MONO_FONT, 7, QFont.Bold))
                    p.drawText(lbl_pt, f"⛔ {ch_lbl}")

        p.setPen(Qt.NoPen)
        for (h, d, v) in pt_list:
            sp = pt(h, d, max(v, 0.02))
            if rect.contains(sp):
                g = int(80 + min(max(v / v_max, 0), 1) * 160)
                p.setBrush(QBrush(QColor(0, g, 30, 190)))
                p.drawEllipse(sp, 2, 2)

        ZONE_COLORS = [
            ("#39ff14", "#00aa33"),
            ("#ffb300", "#cc7700"),
            ("#ff3030", "#aa1010"),
        ]
        for (h_pos, d_pos, v_pos, ht, zi, tid) in person_list:
            ec, fc = ZONE_COLORS[min(zi, 2)]
            col = QColor(ec)
            ht_clamped = min(max(ht, 0.4), v_max - 0.1)
            bot  = pt(h_pos, d_pos, 0.04)
            top_ = pt(h_pos, d_pos, ht_clamped)
            bw   = max(6, int(sx * 0.32))
            g2 = QColor(ec); g2.setAlpha(40)
            p.setBrush(QBrush(g2)); p.setPen(Qt.NoPen)
            p.drawEllipse(bot, bw+8, 5)
            body = QRectF(bot.x()-bw/2, top_.y(), bw, bot.y()-top_.y())
            fill_c = QColor(fc); fill_c.setAlpha(140)
            p.setBrush(QBrush(fill_c))
            p.setPen(QPen(col, 2))
            p.drawRect(body)
            hr = max(4, bw//2)
            p.setBrush(QBrush(col))
            p.setPen(QPen(QColor("#ffffff"), 1))
            p.drawEllipse(QPointF(top_.x(), top_.y()-hr), hr, hr)
            lbl_txt = ["SAFE","ALRT","RESTR"][zi]
            p.setPen(QPen(QColor(ec)))
            p.setFont(QFont(MONO_FONT, 6, QFont.Bold))
            p.drawText(QPointF(top_.x()-10, top_.y()-hr*2-4), f"#{tid:02d}")
            p.setPen(QPen(QColor("#ffffff")))
            p.setFont(QFont(MONO_FONT, 7, QFont.Bold))
            p.drawText(QPointF(top_.x()+hr+4, top_.y()),
                       f"{ht:.1f}m [{lbl_txt}]")

        org = pt(0, 0, 0)
        glow = QColor(PHOSPHOR); glow.setAlpha(55)
        p.setBrush(QBrush(glow)); p.setPen(Qt.NoPen)
        p.drawEllipse(org, 13, 9)
        p.setBrush(QBrush(QColor(PHOSPHOR)))
        p.setPen(QPen(QColor(PHOSPHOR), 1))
        p.drawEllipse(org, 5, 5)
        p.setPen(QPen(QColor(PHOSPHOR)))
        p.setFont(QFont(MONO_FONT, 7, QFont.Bold))
        p.drawText(QPointF(org.x()+8, org.y()+4), "SENSOR")

        AX = "#4aaa4a"
        p.setPen(QPen(QColor(AX), 2))
        p.drawLine(pt(h_min-0.2,0,0), pt(h_max+0.4,0,0))
        p.drawLine(pt(h_min,0,0), pt(h_min,0,v_max+0.2))
        p.drawLine(pt(h_min,0,0), pt(h_min,d_max+0.3,0))

        p.setFont(QFont(MONO_FONT, 8, QFont.Bold))
        p.setPen(QPen(QColor(AX)))
        he = pt(h_max+0.45, 0, 0)
        p.drawText(QPointF(he.x()+2, he.y()+4), f"{h_label} →")
        ze = pt(h_min, 0, v_max+0.25)
        p.drawText(QPointF(ze.x()-28, ze.y()-4), f"↑ Z (m)")
        de = pt(h_min, d_max+0.35, 0)
        p.drawText(QPointF(de.x()+2, de.y()+4), f"{d_label} →")

        p.setFont(QFont(MONO_FONT, 6))
        p.setPen(QPen(QColor("#5a9a5a")))
        hv = math.ceil(h_min)
        while hv <= h_max + 0.01:
            tp = pt(hv, 0, 0)
            p.drawText(QPointF(tp.x()-8, tp.y()+14), f"{hv:.0f}")
            p.setPen(QPen(QColor("#3a7a3a"), 1))
            p.drawLine(QPointF(tp.x(), tp.y()), QPointF(tp.x(), tp.y()+5))
            p.setPen(QPen(QColor("#5a9a5a")))
            hv = round(hv + 1.0, 6)

        vv = 0.5
        while vv <= v_max + 0.01:
            tp = pt(h_min, 0, vv)
            p.drawText(QPointF(tp.x()-34, tp.y()+4), f"{vv:.1f}m")
            p.setPen(QPen(QColor("#3a7a3a"), 1))
            p.drawLine(QPointF(tp.x()-3, tp.y()), QPointF(tp.x(), tp.y()))
            p.setPen(QPen(QColor("#5a9a5a")))
            vv = round(vv + 0.5, 6)

        dv = 1.0
        while dv <= d_max + 0.01:
            tp = pt(h_min, dv, 0)
            p.drawText(QPointF(tp.x()+3, tp.y()+10), f"{dv:.0f}m")
            p.setPen(QPen(QColor("#3a7a3a"), 1))
            p.drawLine(QPointF(tp.x()-2, tp.y()+2), QPointF(tp.x(), tp.y()))
            p.setPen(QPen(QColor("#5a9a5a")))
            dv = round(dv + 1.0, 6)

        zone_radii_list = [self.safe_r, self.alert_r, self.restricted_r]
        for zi, d_bound in enumerate(zone_radii_list):
            if d_bound > d_max: continue
            lbl, wc, fc = self._ZONE_DEF[zi]
            pen_z = QPen(QColor(wc), 1, Qt.DashLine)
            pen_z.setDashPattern([6,4])
            p.setPen(pen_z)
            p.drawLine(pt(h_min, d_bound, 0), pt(h_max, d_bound, 0))
            p.drawLine(pt(h_min, d_bound, v_max), pt(h_max, d_bound, v_max))
            p.drawLine(pt(h_min, d_bound, 0), pt(h_min, d_bound, v_max))
            p.drawLine(pt(h_max, d_bound, 0), pt(h_max, d_bound, v_max))
            mid_lp = QPointF(
                (pt(h_min,d_bound,v_max).x()+pt(h_max,d_bound,v_max).x())/2 - 20,
                min(pt(h_min,d_bound,v_max).y(), pt(h_max,d_bound,v_max).y()) - 5
            )
            if mid_lp.y() > rect.y() + MT - 5:
                p.setPen(QPen(QColor(wc)))
                p.setFont(QFont(MONO_FONT, 7, QFont.Bold))
                p.drawText(mid_lp, f"{lbl}  {d_bound:.0f}m")

        p.setPen(QPen(QColor(PHOSPHOR), 1))
        p.setBrush(Qt.NoBrush)
        p.drawRect(rect)
        p.setClipping(False)

    def _zone_limits(self):
        return [
            (0.0,           self.safe_r),
            (self.safe_r,   self.alert_r),
            (self.alert_r,  self.restricted_r),
        ]

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        W = self.width(); H = self.height()
        p.fillRect(0, 0, W, H, QColor("#040904"))
        GAP = 6; MX = 4; MY = 4
        pw  = (W - 2*MX - GAP) // 2
        ph  = H - 2*MY
        left_rect  = QRectF(MX,           MY, pw, ph)
        right_rect = QRectF(MX+pw+GAP,    MY, pw, ph)
        xh  = self.restricted_r * 0.72
        d_m = self.restricted_r
        z_m = 3.0
        zl = self._zone_limits()
        hz = None
        if self.zone is not None:
            hz = (self.zone.x0, self.zone.x1,
                  self.zone.y0, self.zone.y1,
                  self.zone.z0, self.zone.z1)
        pts_front = [(float(pp[0]), float(pp[1]), float(pp[2]))
                     for pp in self._points]
        pts_side  = [(float(pp[1]), abs(float(pp[0])), float(pp[2]))
                     for pp in self._points]
        def build_persons_front():
            out = []
            for t in self._targets:
                per  = self._persons.get(t["id"])
                ht   = max(per.height if per else 1.7, 0.4)
                dist = math.sqrt(t["x"]**2 + t["y"]**2)
                zi   = self._zone_idx(dist)
                out.append((t["x"], t["y"], t["z"], ht, zi, t["id"]))
            return out
        def build_persons_side():
            out = []
            for t in self._targets:
                per  = self._persons.get(t["id"])
                ht   = max(per.height if per else 1.7, 0.4)
                dist = math.sqrt(t["x"]**2 + t["y"]**2)
                zi   = self._zone_idx(dist)
                out.append((t["y"], abs(t["x"]), t["z"], ht, zi, t["id"]))
            return out
        hz_side = None
        if hz is not None:
            hz_side = (hz[2], hz[3], abs(hz[0]), abs(hz[1]), hz[4], hz[5])
        # Chamber bands for FRONT pane: h=X, d=Y
        # CHAMBER_A: x0=-4.5,x1=-2.0, y0=3.5,y1=7.0
        # CHAMBER_B: x0=+2.0,x1=+4.5, y0=3.5,y1=7.0
        ch_bands_front = [
            (self.CHAMBER_A[0], self.CHAMBER_A[1],
             self.CHAMBER_A[2], self.CHAMBER_A[3], "UNIT A"),
            (self.CHAMBER_B[0], self.CHAMBER_B[1],
             self.CHAMBER_B[2], self.CHAMBER_B[3], "UNIT B"),
        ]
        # Chamber bands for SIDE pane: h=Y (depth), d=|X| (width)
        # Y range stays same; |X| range is the X-extent of each chamber
        ch_bands_side = [
            (self.CHAMBER_A[2], self.CHAMBER_A[3],   # Y: 3.5 → 7.0  (h-axis)
             abs(self.CHAMBER_A[0]), abs(self.CHAMBER_A[1]),  # |X|: 2.0 → 4.5 (d-axis)
             "UNIT A"),
            (self.CHAMBER_B[2], self.CHAMBER_B[3],   # Y: 3.5 → 7.0  (h-axis)
             self.CHAMBER_B[0], self.CHAMBER_B[1],   # X:  2.0 → 4.5 (d-axis)
             "UNIT B"),
        ]
        self._draw_pane(
            p, left_rect,
            title="  FRONT VIEW  ·  X / Z  (Y depth)",
            h_min=-xh, h_max=xh, d_max=d_m, v_max=z_m,
            h_label="X (m)", d_label="Y (m)",
            zone_d_limits=zl, hz_box=hz,
            pt_list=pts_front, person_list=build_persons_front(),
            chamber_bands=ch_bands_front,
        )
        self._draw_pane(
            p, right_rect,
            title="  SIDE VIEW  ·  Y / Z  (X width)",
            h_min=0.0, h_max=d_m, d_max=xh, v_max=z_m,
            h_label="Y (m)", d_label="X (m)",
            zone_d_limits=zl, hz_box=hz_side,
            pt_list=pts_side, person_list=build_persons_side(),
            chamber_bands=ch_bands_side,
        )
        mid_x = MX + pw + GAP/2
        p.setPen(QPen(QColor(PHOSPHOR), 1, Qt.DotLine))
        p.drawLine(QPointF(mid_x, MY), QPointF(mid_x, H-MY))
        p.setPen(QPen(QColor(PHOSPHOR), 1))
        p.setBrush(Qt.NoBrush)
        p.drawRect(QRectF(MX, MY, W-2*MX, H-2*MY))

        # ── Rotation HUD (top-right) ───────────────────────────────
        ang_deg   = math.degrees(self._view_ang)
        depth_val = self._view_depth
        zoom_val  = self._zoom
        p.setFont(QFont(MONO_FONT, 7))
        hud_lines = [
            f"ANG   {ang_deg:+.0f}°",
            f"DEPTH {depth_val:.2f}",
            f"ZOOM  {zoom_val:.2f}×",
            "",
            "LMB drag  =  rotate",
            "LMB ↕     =  depth",
            "RMB drag  =  zoom",
            "scroll    =  zoom",
            "dbl-click =  reset",
        ]
        hud_x = W - MX - 8
        hud_y = MY + 18
        for i, hl in enumerate(hud_lines):
            col = PHOSPHOR if i < 3 else SUBTEXT
            p.setPen(QPen(QColor(col)))
            fw = p.fontMetrics().horizontalAdvance(hl)
            p.drawText(QPointF(hud_x - fw, hud_y + i * 12), hl)

        p.end()


class MiniRadarWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setStyleSheet(f"background:{BG};border-radius:4px;")
        self.safe_r=3.0; self.alert_r=5.0; self.restricted_r=7.5
        self.view_range=9.0
        self._points=np.empty((0,4),dtype=np.float32)
        self._targets=[]; self._persons={}

    def set_scene(self, points, targets, persons, safe_r, alert_r, restricted_r):
        self._points=points; self._targets=targets; self._persons=persons
        self.safe_r=safe_r; self.alert_r=alert_r; self.restricted_r=restricted_r
        self.view_range=restricted_r+1.5; self.update()

    def _wp(self, x_m, y_m, cx, cy, sc):
        return QPointF(cx + x_m*sc, cy - y_m*sc)

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        W=self.width(); H=self.height()
        cx=W/2; cy=H*0.90; sc=(H*0.88)/self.view_range
        p.fillRect(0,0,W,H,QColor(BG))
        pen_g=QPen(QColor(BORDER),1,Qt.DotLine)
        for r_in, r_out, col_hex, alph in [
            (0, self.safe_r, "#00ff80", 18),
            (self.safe_r, self.alert_r, "#ffee44", 14),
            (self.alert_r, self.restricted_r, "#ff4444", 14),
        ]:
            ri=r_in*sc; ro=r_out*sc
            path=QPainterPath()
            if ri > 0:
                path.arcMoveTo(QRectF(cx-ro,cy-ro,2*ro,2*ro),0)
                path.arcTo(QRectF(cx-ro,cy-ro,2*ro,2*ro),0,180)
                path.arcTo(QRectF(cx-ri,cy-ri,2*ri,2*ri),180,-180)
            else:
                path.arcMoveTo(QRectF(cx-ro,cy-ro,2*ro,2*ro),0)
                path.arcTo(QRectF(cx-ro,cy-ro,2*ro,2*ro),0,180)
                path.lineTo(cx,cy)
            path.closeSubpath()
            fc=QColor(col_hex); fc.setAlpha(alph)
            p.setBrush(QBrush(fc)); p.setPen(Qt.NoPen); p.drawPath(path)
        p.setFont(QFont(MONO_FONT,6))
        for d in range(1,int(self.view_range)+1):
            r=d*sc; p.setPen(pen_g)
            p.drawArc(QRectF(cx-r,cy-r,2*r,2*r),0,180*16)
            p.setPen(QPen(QColor(SUBTEXT)))
            p.drawText(QPointF(cx+3,cy-r+4),f"{d}m")
        for deg in range(0,181,30):
            a=math.radians(deg)
            p.setPen(pen_g)
            p.drawLine(QPointF(cx,cy),
                QPointF(cx+self.view_range*sc*math.cos(math.pi-a),
                        cy-self.view_range*sc*math.sin(a)))
        for radius, col_h, lbl in [
            (self.safe_r,       "#39ff14", f"SAFE {self.safe_r:.0f}m"),
            (self.alert_r,      AMBER,     f"ALERT {self.alert_r:.0f}m"),
            (self.restricted_r, RED_ALERT, f"RESTR {self.restricted_r:.0f}m"),
        ]:
            r=radius*sc
            p.setPen(QPen(QColor(col_h),2,Qt.DashLine))
            p.drawArc(QRectF(cx-r,cy-r,2*r,2*r),0,180*16)
            p.setPen(QPen(QColor(col_h)))
            p.setFont(QFont(MONO_FONT,6,QFont.Bold))
            p.drawText(QPointF(cx-20,cy-r-3),lbl)
        if len(self._points)>0:
            p.setPen(Qt.NoPen)
            for pt_ in self._points:
                pp=self._wp(pt_[0],pt_[1],cx,cy,sc)
                z_n=min(max(float(pt_[2])/3.0,0.0),1.0)
                p.setBrush(QBrush(QColor(0,int(77+z_n*178),0,140)))
                p.drawEllipse(pp,2,2)
        for t in self._targets:
            pp=self._wp(t["x"],t["y"],cx,cy,sc)
            per=self._persons.get(t["id"])
            dist=math.sqrt(t["x"]**2+t["y"]**2)
            col=(QColor(RED_ALERT) if dist>self.restricted_r
                 else QColor(AMBER) if dist>self.alert_r
                 else QColor("#39ff14"))
            sz=7
            tri=QPolygonF([QPointF(pp.x(),pp.y()-sz),
                           QPointF(pp.x()-sz*0.7,pp.y()+sz*0.5),
                           QPointF(pp.x()+sz*0.7,pp.y()+sz*0.5)])
            p.setPen(QPen(col,1)); p.setBrush(QBrush(col)); p.drawPolygon(tri)
            p.setPen(QPen(QColor(WHITE_TEXT)))
            p.setFont(QFont(MONO_FONT,7,QFont.Bold))
            p.drawText(QPointF(pp.x()+8,pp.y()+3),f"#{t['id']}")
        p.setPen(QPen(QColor(PHOSPHOR),2)); p.setBrush(QBrush(QColor(PHOSPHOR)))
        p.drawEllipse(QPointF(cx,cy),4,4)
        p.setPen(QPen(QColor(PHOSPHOR)))
        p.setFont(QFont(MONO_FONT,6,QFont.Bold))
        p.drawText(QPointF(cx+6,cy+3),"SENSOR")
        p.setPen(QPen(QColor(PHOSPHOR)))
        p.setFont(QFont(MONO_FONT,8,QFont.Bold))
        p.drawText(6,14,"RADAR FIELD  —  Top View")
        p.end()


# ══════════════════════════════════════════════════════════════════
#  UI HELPERS  
# ══════════════════════════════════════════════════════════════════
def mk_label(text, size=11, bold=False, color=WHITE_TEXT, mono=False):
    l = QLabel(text)
    ff = f"{MONO_FONT}, monospace" if mono else ("Helvetica Neue" if __import__("platform").system()=="Darwin" else "DejaVu Sans" if __import__("platform").system()=="Linux" else "Arial")
    w  = "700" if bold else "400"
    l.setStyleSheet(f"color:{color};font-size:{size}px;font-weight:{w};font-family:{ff};")
    return l

def mk_divider():
    d = QFrame(); d.setFrameShape(QFrame.HLine)
    d.setStyleSheet(f"color:{BORDER}; background:{BORDER};"); d.setFixedHeight(1)
    return d

class StatCard(QFrame):
    def __init__(self, title, value="—", unit="", accent=PHOSPHOR):
        super().__init__()
        self.accent = accent
        self.setStyleSheet(f"""
            QFrame {{
                background:{PANEL};
                border:1px solid {BORDER};
                border-left:3px solid {accent};
                border-radius:4px;
            }}
        """)
        v = QVBoxLayout(self); v.setContentsMargins(10,8,10,8); v.setSpacing(1)
        self._title = QLabel(title.upper())
        self._title.setStyleSheet(f"color:{SUBTEXT};font-size:9px;letter-spacing:2px;font-family:{MONO_FONT};")
        self._val   = QLabel(value)
        self._val.setStyleSheet(f"color:{accent};font-size:20px;font-weight:700;font-family:{MONO_FONT};")
        self._unit  = QLabel(unit)
        self._unit.setStyleSheet(f"color:{SUBTEXT};font-size:9px;font-family:{MONO_FONT};")
        v.addWidget(self._title)
        v.addWidget(self._val)
        v.addWidget(self._unit)

    def set_value(self, v):
        self._val.setText(str(v))

class AlertBanner(QFrame):
    def __init__(self, icon, text, color=RED_ALERT):
        super().__init__()
        self.color  = color
        self.active = False
        self.setFixedHeight(38)
        lay = QHBoxLayout(self); lay.setContentsMargins(12,4,12,4); lay.setSpacing(8)
        self._icon = QLabel(icon)
        self._icon.setStyleSheet(f"font-size:16px;")
        self._lbl  = QLabel(text)
        self._lbl.setStyleSheet(f"font-size:11px;font-weight:700;font-family:{MONO_FONT};letter-spacing:1px;")
        self._dot  = QLabel("●")
        self._dot.setStyleSheet(f"font-size:10px;")
        lay.addWidget(self._dot); lay.addWidget(self._icon); lay.addWidget(self._lbl)
        lay.addStretch()
        self._off()

    def _off(self):
        self.setStyleSheet(f"QFrame{{background:{PANEL};border:1px solid {BORDER};border-radius:4px;}}")
        self._lbl.setStyleSheet(f"color:{SUBTEXT};font-size:11px;font-weight:700;font-family:{MONO_FONT};letter-spacing:1px;")
        self._dot.setStyleSheet(f"color:{SUBTEXT};font-size:10px;")
        self._icon.setStyleSheet(f"color:{SUBTEXT};font-size:16px;")

    def _on(self):
        self.setStyleSheet(f"QFrame{{background:{self.color}18;border:1px solid {self.color};border-radius:4px;}}")
        self._lbl.setStyleSheet(f"color:{self.color};font-size:11px;font-weight:700;font-family:{MONO_FONT};letter-spacing:1px;")
        self._dot.setStyleSheet(f"color:{self.color};font-size:10px;")
        self._icon.setStyleSheet(f"color:{self.color};font-size:16px;")

    def set_active(self, state: bool):
        if state != self.active:
            self.active = state
            self._on() if state else self._off()


class PersonRow(QFrame):
    def __init__(self, person: PersonState):
        super().__init__()
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self._build(person)

    def _build(self, p: PersonState):
        bc = RED_ALERT if p.in_hazard else BORDER
        self.setStyleSheet(f"""
            QFrame {{
                background:{PANEL};
                border:1px solid {bc};
                border-left:3px solid {bc};
                border-radius:3px;
            }}
        """)
        row = QHBoxLayout(self); row.setContentsMargins(8,5,8,5); row.setSpacing(14)

        def field(label, val, col=WHITE_TEXT):
            w = QWidget(); lay = QVBoxLayout(w); lay.setContentsMargins(0,0,0,0); lay.setSpacing(0)
            lbl = QLabel(label); lbl.setStyleSheet(f"color:{SUBTEXT};font-size:8px;font-family:{MONO_FONT};letter-spacing:1px;")
            vl  = QLabel(val);   vl.setStyleSheet(f"color:{col};font-size:12px;font-weight:700;font-family:{MONO_FONT};")
            lay.addWidget(lbl); lay.addWidget(vl)
            return w

        row.addWidget(field("ID", f"#{p.tid:02d}", PHOSPHOR))
        row.addWidget(field("HEIGHT", f"{p.height:.2f} m", WHITE_TEXT))
        row.addWidget(field("X", f"{p.x:+.2f}"))
        row.addWidget(field("Y", f"{p.y:.2f}"))
        row.addWidget(field("Z", f"{p.z:.2f}"))

        flags = []
        if p.in_hazard: flags.append(("⚠ HAZARD", RED_ALERT))

        for txt, col in flags:
            fl = QLabel(txt)
            fl.setStyleSheet(f"color:{col};font-size:9px;font-weight:700;font-family:{MONO_FONT};"
                             f"background:{col}18;border:1px solid {col};border-radius:3px;padding:1px 5px;")
            row.addWidget(fl)
        row.addStretch()


# ══════════════════════════════════════════════════════════════════
#  CHAMBER STATUS WIDGET  
# ══════════════════════════════════════════════════════════════════
class ChamberStatusWidget(QFrame):
    """Compact status panel showing both chamber breach states."""
    def __init__(self):
        super().__init__()
        self.setStyleSheet(f"QFrame{{background:{PANEL};border:1px solid {BORDER};border-radius:4px;}}")
        layout = QHBoxLayout(self)
        layout.setContentsMargins(8, 6, 8, 6)
        layout.setSpacing(12)

        self._banners = {}
        for name, col in [("UNIT A", RED_ALERT), ("UNIT B", RED_ALERT)]:
            frame = QFrame()
            frame.setFixedHeight(44)
            fl = QVBoxLayout(frame); fl.setContentsMargins(8, 4, 8, 4); fl.setSpacing(1)
            title = QLabel(f"🏭  {name}")
            title.setStyleSheet(f"color:{SUBTEXT};font-size:8px;font-family:{MONO_FONT};letter-spacing:2px;font-weight:700;")
            status = QLabel("● SECURE")
            status.setStyleSheet(f"color:{PHOSPHOR};font-size:11px;font-weight:700;font-family:{MONO_FONT};")
            fl.addWidget(title); fl.addWidget(status)
            self._banners[name] = (frame, status)
            layout.addWidget(frame)

        layout.addStretch()

        # Legend
        leg = QLabel("⛔ = Restricted Chamber  ·  Red wall pulsing = Active Breach")
        leg.setStyleSheet(f"color:{SUBTEXT};font-size:8px;font-family:{MONO_FONT};")
        layout.addWidget(leg)

    def set_breach(self, chamber_name: str, breach: bool):
        if chamber_name not in self._banners:
            return
        frame, status = self._banners[chamber_name]
        if breach:
            frame.setStyleSheet(
                f"QFrame{{background:{RED_ALERT}15;border:1px solid {RED_ALERT};border-radius:4px;}}")
            status.setText("⚠ BREACH DETECTED")
            status.setStyleSheet(f"color:{RED_ALERT};font-size:11px;font-weight:700;font-family:{MONO_FONT};")
        else:
            frame.setStyleSheet(
                f"QFrame{{background:{PANEL};border:1px solid {BORDER};border-radius:4px;}}")
            status.setText("● SECURE")
            status.setStyleSheet(f"color:{PHOSPHOR};font-size:11px;font-weight:700;font-family:{MONO_FONT};")


# ══════════════════════════════════════════════════════════════════
#  GLOBAL STYLESHEET
# ══════════════════════════════════════════════════════════════════
GLOBAL_SS = f"""
QMainWindow, QWidget {{
    background: {BG};
    color: {WHITE_TEXT};
    font-family: Arial, sans-serif;
}}
QGroupBox {{
    border: 1px solid {BORDER};
    border-radius: 5px;
    margin-top: 16px;
    padding: 10px 8px 8px 8px;
    color: {SUBTEXT};
    font-size: 9px;
    letter-spacing: 2px;
    font-family: {MONO_FONT};
}}
QGroupBox::title {{
    subcontrol-origin: margin;
    subcontrol-position: top left;
    left: 10px;
    top: 1px;
    padding: 0 4px;
    background: {BG};
}}
QComboBox, QDoubleSpinBox {{
    background: {PANEL};
    border: 1px solid {BORDER};
    color: {PHOSPHOR};
    padding: 4px 8px;
    border-radius: 3px;
    font-size: 11px;
    font-family: {MONO_FONT};
    min-height: 22px;
}}
QComboBox::drop-down {{ border: none; width: 20px; }}
QComboBox QAbstractItemView {{
    background: {PANEL};
    color: {PHOSPHOR};
    border: 1px solid {BORDER};
    selection-background-color: {BORDER};
}}
QPushButton {{
    background: {BORDER};
    border: none;
    color: {WHITE_TEXT};
    padding: 7px 16px;
    border-radius: 3px;
    font-size: 11px;
    font-weight: 700;
    font-family: {MONO_FONT};
    letter-spacing: 1px;
}}
QPushButton:hover {{ background: #253525; }}
QPushButton:disabled {{ color: {SUBTEXT}; }}
QPushButton#btn_connect {{
    background: {PHOSPHOR};
    color: #000;
    border: none;
}}
QPushButton#btn_connect:hover {{ background: #55ff33; }}
QPushButton#btn_connected {{
    background: {BORDER};
    color: {PHOSPHOR};
    border: 1px solid {PHOSPHOR};
    letter-spacing: 2px;
}}
QPushButton#btn_connected:disabled {{
    background: {BORDER};
    color: {PHOSPHOR};
    border: 1px solid {PHOSPHOR};
}}
QPushButton#btn_stop {{
    background: #b06800;
    color: #fff;
}}
QPushButton#btn_stop:hover {{ background: #8a5000; }}
QTextEdit {{
    background: #050805;
    border: 1px solid {BORDER};
    color: {DIM_GREEN};
    font-family: {MONO_FONT};
    font-size: 10px;
    border-radius: 3px;
}}
QScrollBar:vertical {{
    background: {PANEL}; width: 7px; border-radius: 3px;
}}
QScrollBar::handle:vertical {{
    background: {BORDER}; border-radius: 3px;
}}
QTabWidget::pane {{
    border: 1px solid {BORDER}; border-radius: 4px;
}}
QTabBar::tab {{
    background: {PANEL}; color: {SUBTEXT};
    padding: 6px 18px;
    border: 1px solid {BORDER};
    border-bottom: none;
    border-top-left-radius: 4px;
    border-top-right-radius: 4px;
    font-family: {MONO_FONT};
    font-size: 10px;
    letter-spacing: 1px;
    margin-right: 2px;
}}
QTabBar::tab:selected {{
    background: {BORDER};
    color: {PHOSPHOR};
}}
QSplitter::handle {{
    background: {BORDER};
}}
"""


# ══════════════════════════════════════════════════════════════════
#  MAIN WINDOW
# ══════════════════════════════════════════════════════════════════
class MainWindow(QMainWindow):

    def __init__(self):
        super().__init__()
        self.setWindowTitle("RADAR IMS  v2  ·  IWR6843AOP EVM  ·  Industrial Facility Monitor")
        self.setMinimumSize(1480, 860)
        self.setStyleSheet(GLOBAL_SS)

        self.zone    = HazardZone()
        self.persons = {}
        self.worker  = None
        self._frame_ts   = deque(maxlen=60)
        self._track_seen = {}

        self._chamber_a = (-4.5, -2.0, 3.5, 7.0)
        self._chamber_b = ( 2.0,  4.5, 3.5, 7.0)

        # Machine proximity: track rising edge to avoid repeated log spam
        self._mach_a_prox_prev = False
        self._mach_b_prox_prev = False

        # ── v2 companion modules ──────────────────────────────────
        self._settings      = Settings()
        self._settings.load()
        _log.info("Vigil Sense v2 starting up")

        self._alert_mgr     = AlertManager()
        self._data_logger   = DataLogger()
        self._data_logger.start()
        self._stats         = StatsTracker()

        self._build_ui()
        QTimer.singleShot(100, self._on_refresh_ports)

    def _build_ui(self):
        root_w = QWidget(); self.setCentralWidget(root_w)
        root   = QHBoxLayout(root_w)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        # ══════════════════════════════════════════════════════════
        # LEFT SIDE PANEL
        # ══════════════════════════════════════════════════════════
        left_panel = QWidget(); left_panel.setFixedWidth(580)
        left_root  = QHBoxLayout(left_panel)
        left_root.setContentsMargins(0, 0, 0, 0)
        left_root.setSpacing(8)

        # ── COLUMN A ─────────────────────────────────────────────
        col_a = QWidget(); col_a.setFixedWidth(270)
        la    = QVBoxLayout(col_a); la.setSpacing(8); la.setContentsMargins(0,0,0,0)

        hdr  = QLabel("◈  VIGIL SENSE")
        hdr.setStyleSheet(f"color:{PHOSPHOR};font-size:17px;font-weight:700;"
                          f"font-family:{MONO_FONT};letter-spacing:3px;")
        sub  = QLabel("INDUSTRIAL MONITORING SYSTEM")
        sub.setStyleSheet(f"color:{SUBTEXT};font-size:12px;font-family:{MONO_FONT};letter-spacing:2px;")
        sub2 = QLabel("IWR6843AOP EVM  ·  60 GHz mmWave")
        sub2.setStyleSheet(f"color:{DIM_GREEN};font-size:10px;font-family:{MONO_FONT};")
        la.addWidget(hdr); la.addWidget(sub); la.addWidget(sub2)
        la.addWidget(mk_divider())

        conn = QGroupBox("CONNECTION"); cg = QGridLayout(conn); cg.setSpacing(5)
        cg.addWidget(mk_label("CLI Port",  10, color=SUBTEXT, mono=True), 0, 0)
        self.cmb_cli = QComboBox(); self.cmb_cli.setMinimumWidth(118)
        cg.addWidget(self.cmb_cli, 0, 1)
        cg.addWidget(mk_label("Data Port", 10, color=SUBTEXT, mono=True), 1, 0)
        self.cmb_data = QComboBox(); self.cmb_data.setMinimumWidth(118)
        cg.addWidget(self.cmb_data, 1, 1)
        btn_refresh = QPushButton("⟳ REFRESH PORTS")
        btn_refresh.clicked.connect(self._on_refresh_ports)
        cg.addWidget(btn_refresh, 2, 0, 1, 2)
        self.btn_connect = QPushButton("▶  CONNECT")
        self.btn_connect.setObjectName("btn_connect")
        self.btn_connect.clicked.connect(self._on_connect)
        cg.addWidget(self.btn_connect, 3, 0, 1, 2)
        self.btn_stop = QPushButton("■  STOP")
        self.btn_stop.setObjectName("btn_stop")
        self.btn_stop.clicked.connect(self._on_stop)
        self.btn_stop.setEnabled(False)
        cg.addWidget(self.btn_stop, 4, 0, 1, 2)
        la.addWidget(conn)

        self.lbl_status = QLabel("⬤  OFFLINE")
        self.lbl_status.setStyleSheet(
            f"color:{SUBTEXT};font-size:12px;font-weight:700;font-family:{MONO_FONT};")
        la.addWidget(self.lbl_status)
        la.addWidget(mk_divider())

        # ── System Alerts ─────────────────────────────────────────
        alrt_grp = QGroupBox("SYSTEM ALERTS")
        ag = QVBoxLayout(alrt_grp); ag.setSpacing(6); ag.setContentsMargins(6,6,6,6)
        self.alert_hazard        = AlertBanner("⚠",  "RESTRICTED ZONE",    RED_ALERT)
        self.alert_chamber_a     = AlertBanner("🏭", "UNIT A  BREACH",  RED_ALERT)
        self.alert_chamber_b     = AlertBanner("🏭", "UNIT B  BREACH",  RED_ALERT)
        self.alert_mach_a        = AlertBanner("⚡", "UNIT A  MACHINE  CRITICAL", "#ff4400")
        self.alert_mach_b        = AlertBanner("⚡", "UNIT B  MACHINE  CRITICAL", "#ff4400")
        ag.addWidget(self.alert_hazard)
        ag.addWidget(self.alert_chamber_a)
        ag.addWidget(self.alert_chamber_b)
        ag.addWidget(self.alert_mach_a)
        ag.addWidget(self.alert_mach_b)
        la.addWidget(alrt_grp)

        la.addWidget(mk_divider())

        hz_grp = QGroupBox("RANGING ZONES  (metres)")
        hzg    = QGridLayout(hz_grp); hzg.setSpacing(5); hzg.setContentsMargins(6,8,6,8)
        hzg.addWidget(mk_label("ZONE",    9, color=SUBTEXT, mono=True), 0, 0)
        hzg.addWidget(mk_label("MIN (m)", 9, color=SUBTEXT, mono=True), 0, 1)
        hzg.addWidget(mk_label("MAX (m)", 9, color=SUBTEXT, mono=True), 0, 2)

        self._zone_spins = {}

        def zone_row(row, zone_name, color, min_val, max_val, key):
            dot = QLabel("●"); dot.setStyleSheet(f"color:{color};font-size:10px;")
            lbl = QLabel(zone_name)
            lbl.setStyleSheet(
                f"color:{color};font-size:9px;font-family:{MONO_FONT};font-weight:700;")
            sp_min = QDoubleSpinBox()
            sp_min.setRange(0, 50); sp_min.setValue(min_val)
            sp_min.setSingleStep(0.5); sp_min.setDecimals(1)
            sp_max = QDoubleSpinBox()
            sp_max.setRange(0, 50); sp_max.setValue(max_val)
            sp_max.setSingleStep(0.5); sp_max.setDecimals(1)
            row_w = QWidget(); rlay = QHBoxLayout(row_w)
            rlay.setContentsMargins(0,0,0,0); rlay.setSpacing(3)
            rlay.addWidget(dot); rlay.addWidget(lbl)
            hzg.addWidget(row_w,  row, 0)
            hzg.addWidget(sp_min, row, 1)
            hzg.addWidget(sp_max, row, 2)
            self._zone_spins[key + "_min"] = sp_min
            self._zone_spins[key + "_max"] = sp_max

        zone_row(1, "SAFE",       PHOSPHOR,  0.0, 3.0, "safe")
        zone_row(2, "ALERT",      AMBER,     3.0, 5.0, "alert")
        zone_row(3, "RESTRICTED", RED_ALERT, 5.0, 6.0, "restricted")

        btn_hz = QPushButton("APPLY ZONES")
        btn_hz.clicked.connect(self._apply_zone)
        hzg.addWidget(btn_hz, 4, 0, 1, 3)
        la.addWidget(hz_grp)

        # ── Chamber size editor ────────────────────────────────────
        ch_grp = QGroupBox("RESTRICTED UNITS  (editable)")
        cg_lay = QVBoxLayout(ch_grp)
        cg_lay.setSpacing(6); cg_lay.setContentsMargins(6, 8, 6, 8)

        self._ch_spins = {}   # key: "A_x0", "A_x1", "A_y0", "A_y1", same for B

        def _chamber_editor(label, col_hex, key_prefix,
                             x0_def, x1_def, y0_def, y1_def):
            """Build one chamber's 4-spinbox editor row."""
            hdr = QWidget(); hl = QHBoxLayout(hdr)
            hl.setContentsMargins(0,0,0,0); hl.setSpacing(4)
            dot = QLabel("⛔"); dot.setStyleSheet(f"color:{col_hex};font-size:11px;")
            nm  = QLabel(label)
            nm.setStyleSheet(f"color:{col_hex};font-size:9px;font-family:{MONO_FONT};"
                             f"font-weight:700;letter-spacing:1px;")
            hl.addWidget(dot); hl.addWidget(nm); hl.addStretch()
            cg_lay.addWidget(hdr)

            grid = QWidget(); gl = QGridLayout(grid)
            gl.setSpacing(3); gl.setContentsMargins(0,0,0,0)

            def lbl(t):
                w = QLabel(t)
                w.setStyleSheet(f"color:{SUBTEXT};font-size:8px;font-family:{MONO_FONT};")
                return w

            def sp(val, lo, hi):
                s = QDoubleSpinBox()
                s.setRange(lo, hi); s.setValue(val)
                s.setSingleStep(0.5); s.setDecimals(1)
                s.setStyleSheet(f"color:{col_hex};background:{PANEL};"
                                f"border:1px solid {BORDER};font-size:10px;"
                                f"font-family:{MONO_FONT};border-radius:2px;")
                return s

            sx0 = sp(x0_def, -10.0, 10.0)
            sx1 = sp(x1_def, -10.0, 10.0)
            sy0 = sp(y0_def,   0.0, 15.0)
            sy1 = sp(y1_def,   0.0, 15.0)

            gl.addWidget(lbl("X min"), 0, 0); gl.addWidget(sx0, 0, 1)
            gl.addWidget(lbl("X max"), 0, 2); gl.addWidget(sx1, 0, 3)
            gl.addWidget(lbl("Y min"), 1, 0); gl.addWidget(sy0, 1, 1)
            gl.addWidget(lbl("Y max"), 1, 2); gl.addWidget(sy1, 1, 3)
            cg_lay.addWidget(grid)

            self._ch_spins[f"{key_prefix}_x0"] = sx0
            self._ch_spins[f"{key_prefix}_x1"] = sx1
            self._ch_spins[f"{key_prefix}_y0"] = sy0
            self._ch_spins[f"{key_prefix}_y1"] = sy1

        _chamber_editor("UNIT A", CHAMBER_GLOW, "A",
                        -4.5, -2.0, 2.5, 5.0)
        cg_lay.addWidget(mk_divider())
        _chamber_editor("UNIT B", "#ff6060",    "B",
                         2.0,  4.5, 2.5, 5.0)

        btn_apply_ch = QPushButton("⛔  APPLY UNITS")
        btn_apply_ch.setStyleSheet(
            f"background:{CHAMBER_WALL};color:#fff;border:none;"
            f"padding:6px 12px;border-radius:3px;font-size:10px;"
            f"font-weight:700;font-family:{MONO_FONT};letter-spacing:1px;")
        btn_apply_ch.clicked.connect(self._apply_chambers)
        cg_lay.addWidget(btn_apply_ch)

        # Live current bounds display
        self._ch_bounds_lbl = QLabel()
        self._ch_bounds_lbl.setStyleSheet(
            f"color:{SUBTEXT};font-size:7px;font-family:{MONO_FONT};")
        self._ch_bounds_lbl.setWordWrap(True)
        self._ch_bounds_lbl.setText(
            "A: X[-4.5, -2.0]  Y[3.5, 7.0]\nB: X[+2.0, +4.5]  Y[3.5, 7.0]")
        cg_lay.addWidget(self._ch_bounds_lbl)

        la.addWidget(ch_grp)

        la.addStretch()

        # ── COLUMN B ─────────────────────────────────────────────
        col_b = QWidget()
        lb    = QVBoxLayout(col_b); lb.setSpacing(8); lb.setContentsMargins(0,0,0,0)

        lb.addSpacing(72)

        # Live Statistics
        stats = QGroupBox("LIVE STATISTICS")
        sg = QGridLayout(stats); sg.setSpacing(6); sg.setContentsMargins(6,8,6,8)
        self.card_count = StatCard("PEOPLE",        "0", "currently detected",   PHOSPHOR)
        self.card_total = StatCard("TOTAL (2 MIN)", "0", "persons in 2 minutes", CYAN_INFO)
        self.card_fps   = StatCard("FPS",           "0", "frames / second",       CYAN_INFO)
        self.card_pts   = StatCard("POINTS",        "0", "point cloud size",      DIM_GREEN)
        self.card_trk   = StatCard("TRACKS",        "0", "confirmed targets",     AMBER)
        sg.addWidget(self.card_count, 0, 0)
        sg.addWidget(self.card_total, 0, 1)
        sg.addWidget(self.card_fps,   1, 0)
        sg.addWidget(self.card_pts,   1, 1)
        sg.addWidget(self.card_trk,   2, 0)
        lb.addWidget(stats)

        self._total_seen_ids   = set()
        self._window_start_ts  = time.time()
        self._zone_history = []

        # ── Mini radar ─────────────────────────────────────────────
        radar_grp = QGroupBox("RADAR FIELD  (Top View)")
        rg = QVBoxLayout(radar_grp); rg.setContentsMargins(4, 4, 4, 4)
        self._mini_radar = MiniRadarWidget()
        self._mini_radar.setFixedHeight(200)
        rg.addWidget(self._mini_radar)
        lb.addWidget(radar_grp)

        # Live People Count Per Zone
        lzc_grp = QGroupBox("LIVE PEOPLE COUNT PER ZONE")
        lzc_lay = QVBoxLayout(lzc_grp)
        lzc_lay.setSpacing(6); lzc_lay.setContentsMargins(8, 8, 8, 8)

        def zone_count_row(label, color):
            row  = QWidget(); rlay = QHBoxLayout(row)
            rlay.setContentsMargins(0, 0, 0, 0); rlay.setSpacing(8)
            dot  = QLabel("●")
            dot.setStyleSheet(f"color:{color};font-size:12px;")
            lbl  = QLabel(label)
            lbl.setStyleSheet(
                f"color:{WHITE_TEXT};font-size:10px;font-family:{MONO_FONT};font-weight:700;")
            val  = QLabel("0")
            val.setStyleSheet(
                f"color:{color};font-size:16px;font-family:{MONO_FONT};font-weight:700;")
            val.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            rlay.addWidget(dot); rlay.addWidget(lbl)
            rlay.addStretch(); rlay.addWidget(val)
            return row, val

        self._row_safe,       self._val_safe       = zone_count_row("Zone 1 — SAFE",                 PHOSPHOR)
        self._row_alert,      self._val_alert      = zone_count_row("Zone 2 — ALERT",                AMBER)
        self._row_restricted, self._val_restricted = zone_count_row("Zone 3 — RESTRICTED",           RED_ALERT)
        self._row_ch_a,  self._val_ch_a            = zone_count_row("Unit A — BREACH",               CHAMBER_GLOW)
        self._row_ch_b,  self._val_ch_b            = zone_count_row("Unit B — BREACH",               CHAMBER_GLOW)
        self._row_mach_a, self._val_mach_a         = zone_count_row("Unit A — MACHINE PROXIMITY ⚡", "#ff0000")
        self._row_mach_b, self._val_mach_b         = zone_count_row("Unit B — MACHINE PROXIMITY ⚡", "#ff0000")

        lzc_lay.addWidget(mk_divider())
        lzc_lay.addWidget(self._row_safe)
        lzc_lay.addWidget(mk_divider())
        lzc_lay.addWidget(self._row_alert)
        lzc_lay.addWidget(mk_divider())
        lzc_lay.addWidget(self._row_restricted)
        lzc_lay.addWidget(mk_divider())
        lzc_lay.addWidget(self._row_ch_a)
        lzc_lay.addWidget(mk_divider())
        lzc_lay.addWidget(self._row_ch_b)
        lzc_lay.addWidget(mk_divider())
        lzc_lay.addWidget(self._row_mach_a)
        lzc_lay.addWidget(mk_divider())
        lzc_lay.addWidget(self._row_mach_b)
        lb.addWidget(lzc_grp)

        # ── Machine proximity radius tuner ────────────────────────
        mpr_grp = QGroupBox("MACHINE PROXIMITY RADIUS")
        mpr_lay = QHBoxLayout(mpr_grp)
        mpr_lay.setContentsMargins(8, 6, 8, 6); mpr_lay.setSpacing(8)
        mpr_lay.addWidget(mk_label("Radius (m)", 9, color=SUBTEXT, mono=True))
        self._mach_prox_spin = QDoubleSpinBox()
        self._mach_prox_spin.setRange(0.3, 5.0)
        self._mach_prox_spin.setValue(MACHINE_PROX_RADIUS)
        self._mach_prox_spin.setSingleStep(0.1)
        self._mach_prox_spin.setDecimals(1)
        self._mach_prox_spin.setStyleSheet(
            f"color:#ff4400;background:{PANEL};border:1px solid {BORDER};"
            f"font-size:11px;font-family:{MONO_FONT};border-radius:2px;")
        mpr_lay.addWidget(self._mach_prox_spin)
        btn_mpr = QPushButton("APPLY")
        btn_mpr.setStyleSheet(
            f"background:#5a1a00;color:#ff6600;border:1px solid #ff4400;"
            f"padding:4px 10px;border-radius:3px;font-size:10px;"
            f"font-weight:700;font-family:{MONO_FONT};")
        btn_mpr.clicked.connect(self._apply_mach_prox_radius)
        mpr_lay.addWidget(btn_mpr)
        lb.addWidget(mpr_grp)

        lb.addStretch()

        self._safe_min       = 0.0;  self._safe_max       = 3.0
        self._alert_min      = 3.0;  self._alert_max      = 5.0
        self._restricted_min = 5.0;  self._restricted_max = 7.5
        self._hz_spins = []

        sep = QFrame(); sep.setFrameShape(QFrame.VLine)
        sep.setStyleSheet(f"color:{BORDER};")

        left_root.addWidget(col_a)
        left_root.addWidget(sep)
        left_root.addWidget(col_b, stretch=1)

        left_scroll = QScrollArea()
        left_scroll.setWidget(left_panel)
        left_scroll.setWidgetResizable(True)
        left_scroll.setFixedWidth(596)
        left_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        left_scroll.setStyleSheet(
            f"QScrollArea {{ border: none; background: {BG}; }}")

        # ══════════════════════════════════════════════════════════
        # RIGHT PANEL  — tabs
        # ══════════════════════════════════════════════════════════
        right = QWidget()
        rl    = QVBoxLayout(right); rl.setContentsMargins(0,0,0,0); rl.setSpacing(6)

        tabs = QTabWidget()

        # ── TAB 1: INDUSTRIAL FLOOR PLAN  (primary new view) ─────
        tab_floor = QWidget()
        tfl_lay   = QVBoxLayout(tab_floor); tfl_lay.setContentsMargins(4, 4, 4, 4); tfl_lay.setSpacing(4)

        # Chamber status bar at top
        self._chamber_status = ChamberStatusWidget()
        self._chamber_status.setFixedHeight(56)
        tfl_lay.addWidget(self._chamber_status)

        self.floor_plan = IndustrialFloorPlanWidget()
        tfl_lay.addWidget(self.floor_plan, stretch=1)
        tabs.addTab(tab_floor, "  🏭  FACILITY FLOOR PLAN  ")

        # ── TAB 2: 3D FLOOR PLAN  +  2D RADAR TOP VIEW ──────────
        tab_dual = QWidget()
        t2l = QHBoxLayout(tab_dual)
        t2l.setContentsMargins(4, 4, 4, 4)
        t2l.setSpacing(4)

        # Left half — 3D Floor plan (live; fed by _on_frame same as Tab 1)
        self.canvas3d = IndustrialFloorPlanWidget()
        t2l.addWidget(self.canvas3d, stretch=1)

        sep2 = QFrame(); sep2.setFrameShape(QFrame.VLine)
        sep2.setStyleSheet(f"color:{BORDER};")
        t2l.addWidget(sep2)

        # Right half — full-size 2D Radar Top-View (live; updated in _on_frame)
        self.radar_full = RadarViewWidget()
        t2l.addWidget(self.radar_full, stretch=1)

        tabs.addTab(tab_dual, "  3D PLAN  ·  RADAR VIEW  ")

        # ── Demo data timer (runs even when radar is disconnected) ──
        self._demo_tick = 0
        self._demo_timer = QTimer(self)
        self._demo_timer.timeout.connect(self._push_demo_frame)
        self._demo_timer.start(80)   # ~12.5 fps demo

        # ── TAB 3: CLI CONSOLE ───────────────────────────────────
        tab_cli = QWidget(); tcl = QVBoxLayout(tab_cli); tcl.setContentsMargins(6,6,6,6)
        tcl.addWidget(mk_label("CLI  /  SERIAL CONSOLE", 10, bold=True, color=PHOSPHOR, mono=True))
        tcl.addWidget(mk_label(
            f"AOP_6m_default (People Tracking)  ·  CLI @ {CLI_BAUD}  ·  Data @ {DATA_BAUD} baud"
            f"  ·  Lower COM# = CLI,  Higher COM# = Data",
            9, color=SUBTEXT, mono=True))
        self.cli_log = QTextEdit(); self.cli_log.setReadOnly(True)
        tcl.addWidget(self.cli_log)
        tabs.addTab(tab_cli, "  CLI CONSOLE  ")

        # ── TAB 4: CONFIG SCRIPT ─────────────────────────────────
        tab_cfg = QWidget(); tfl2 = QVBoxLayout(tab_cfg); tfl2.setContentsMargins(6,6,6,6)
        tfl2.addWidget(mk_label(
            "CONFIG SCRIPT  —  AOP_6m_default  (People Tracking)",
            10, bold=True, color=PHOSPHOR, mono=True))
        self.cfg_editor = QTextEdit()
        self.cfg_editor.setPlainText(AOP_6M_DEFAULT_CFG.strip())
        tfl2.addWidget(self.cfg_editor)
        tabs.addTab(tab_cfg, "  CONFIG SCRIPT  ")

        rl.addWidget(tabs)

        root.addWidget(left_scroll)
        root.addWidget(right, stretch=1)

    # ══════════════════════════════════════════════════════════════
    #  PORT HELPERS  (v2: uses port_utils module)
    # ══════════════════════════════════════════════════════════════
    def _on_refresh_ports(self):
        ports = list_ports()
        fill_combo(self.cmb_cli,  ports)
        fill_combo(self.cmb_data, ports)
        cli_i, data_i = auto_assign(ports)
        if cli_i is not None and data_i is not None:
            self.cmb_cli.setCurrentIndex(cli_i)
            self.cmb_data.setCurrentIndex(data_i)
            cli_dev  = ports[cli_i].device
            data_dev = ports[data_i].device
            self._log(f"Auto-assigned → CLI: {cli_dev}   Data: {data_dev}", "ok")
            self._log("  (Lower COM# = CLI,  Higher COM# = Data — for IWR6843AOP EVM)", "dim")
            # Restore last-used ports from settings if they match
            saved_cli  = self._settings.get("cli_port",  "")
            saved_data = self._settings.get("data_port", "")
            for i, p in enumerate(ports):
                if p.device == saved_cli:
                    self.cmb_cli.setCurrentIndex(i)
                if p.device == saved_data:
                    self.cmb_data.setCurrentIndex(i)
        elif len(ports) == 1:
            self._log("⚠  Only 1 COM port found — EVM may not be connected or drivers missing", "warn")
        else:
            self._log("✘  No COM ports found — check USB cable and XDS110 drivers", "error")

    def _on_connect(self):
        cp  = self.cmb_cli.currentData()  or self.cmb_cli.currentText().split()[0]
        dp  = self.cmb_data.currentData() or self.cmb_data.currentText().split()[0]
        cfg = self.cfg_editor.toPlainText()

        if not cp or "none" in cp.lower():
            self._log("No CLI port selected — click ⟳ REFRESH PORTS first.", "error"); return
        if not dp or "none" in dp.lower():
            self._log("No Data port selected — click ⟳ REFRESH PORTS first.", "error"); return
        if cp == dp:
            self._log("⚠  CLI and Data ports are the SAME PORT — they must be different!", "error")
            return

        self.btn_connect.setEnabled(False)
        self.btn_connect.setText("◌  CONNECTING…")
        self.btn_connect.setStyleSheet(
            f"background:#1a2a1a;color:{AMBER};border:1px solid {AMBER};"
            f"padding:7px 16px;border-radius:3px;font-size:11px;"
            f"font-weight:700;font-family:{MONO_FONT};letter-spacing:1px;")
        self.btn_stop.setEnabled(True)
        self._set_status("CONNECTING…", AMBER)
        self._log(f"━━━ Connecting:  CLI={cp}   Data={dp} ━━━", "info")

        # Save port choice to settings
        self._settings.set("cli_port",  cp)
        self._settings.set("data_port", dp)
        self._settings.save()
        _log.info("Connecting: CLI=%s  Data=%s", cp, dp)

        self.worker = SerialWorker(cp, dp, cfg)
        self.worker.sig.log.connect(self._log)
        self.worker.sig.config_ok.connect(self._on_config_ok)
        self.worker.sig.data_started.connect(self._on_data_started)
        self.worker.sig.frame.connect(self._on_frame)
        self.worker.start()

    def _on_stop(self):
        if self.worker:
            self.worker.stop(); self.worker = None
        self.btn_connect.setText("▶  CONNECT")
        self.btn_connect.setEnabled(True)
        self.btn_connect.setObjectName("btn_connect")
        self.btn_connect.setStyleSheet(
            f"background:{PHOSPHOR};color:#000;border:none;"
            f"padding:7px 16px;border-radius:3px;font-size:11px;"
            f"font-weight:700;font-family:{MONO_FONT};letter-spacing:1px;")
        self.btn_stop.setEnabled(False)
        self._set_status("OFFLINE", SUBTEXT)
        self._log("Sensor stopped.", "info")
        self._alert_mgr.reset()
        self._stats.reset()
        _log.info("Sensor stopped")

    def _on_config_ok(self, ok):
        if not ok:
            self._set_status("CONFIG ERR", RED_ALERT)
            self._log("━━━ Connection failed. Check ports and retry. ━━━", "error")
            self.btn_connect.setText("▶  CONNECT")
            self.btn_connect.setEnabled(True)
            self.btn_connect.setStyleSheet(
                f"background:{PHOSPHOR};color:#000;border:none;"
                f"padding:7px 16px;border-radius:3px;font-size:11px;"
                f"font-weight:700;font-family:{MONO_FONT};letter-spacing:1px;")
            self.btn_stop.setEnabled(False)

    def _on_data_started(self):
        self._set_status("LIVE", PHOSPHOR)
        self.btn_connect.setText("✔  CONNECTED")
        self.btn_connect.setEnabled(False)
        self.btn_connect.setStyleSheet(
            f"background:{DIM_GREEN};color:{PHOSPHOR};border:1px solid {PHOSPHOR};"
            f"padding:7px 16px;border-radius:3px;font-size:11px;"
            f"font-weight:700;font-family:{MONO_FONT};letter-spacing:1px;")

    def _style_zone_ax(self): pass
    def _update_zone_chart(self): pass

    def _classify_zone(self, person: PersonState) -> str:
        dist = float(np.sqrt(person.x**2 + person.y**2))
        if dist > self._restricted_min:
            return "restricted"
        elif dist > self._alert_min:
            return "alert"
        else:
            return "safe"

    def _in_chamber(self, x, y, chamber):
        x0, x1, y0, y1 = chamber
        return x0 <= x <= x1 and y0 <= y <= y1

    def _machine_pos(self, chamber):
        """Return (mx, my) world coords of the machine inside a chamber.
        Mirrors _draw_chamber_3d exactly: mx=(x0+x1)/2, my=(y0+y1)/2+0.5"""
        x0, x1, y0, y1 = chamber
        return (x0 + x1) / 2.0, (y0 + y1) / 2.0 + 0.5

    def _near_machine(self, tx, ty, chamber, radius=None):
        """True if tracker (tx, ty) is within radius metres of machine centre."""
        if radius is None:
            radius = MACHINE_PROX_RADIUS
        mx, my = self._machine_pos(chamber)
        return math.sqrt((tx - mx) ** 2 + (ty - my) ** 2) <= radius
    # ══════════════════════════════════════════════════════════════
    def _on_frame(self, frame: RadarFrame):
        try:
            self._on_frame_inner(frame)
        except Exception as e:
            import traceback
            self._log(f"⚠ Frame processing error (skipped): {e}", "warn")
            self._log(traceback.format_exc(), "warn")

    def _on_frame_inner(self, frame: RadarFrame):
        now = time.time()
        self._frame_ts.append(now)

        pts = frame.points

        raw_ids = {t["id"] for t in frame.targets}
        for tid in list(self._track_seen.keys()):
            if tid not in raw_ids:
                del self._track_seen[tid]
        for t in frame.targets:
            tid = t["id"]
            self._track_seen[tid] = self._track_seen.get(tid, 0) + 1

        CONFIRM_FRAMES = 3
        confirmed = [t for t in frame.targets
                     if self._track_seen.get(t["id"], 0) >= CONFIRM_FRAMES]

        if len(pts) > 0 and confirmed:
            track_xy = np.array([[t["x"], t["y"]] for t in confirmed], dtype=np.float64)
            pts_xy   = pts[:, :2].astype(np.float64)
            diffs    = pts_xy[np.newaxis, :, :] - track_xy[:, np.newaxis, :]
            sq       = np.clip((diffs ** 2).sum(axis=2), 0.0, 1e6)
            dists    = np.sqrt(sq)
            min_d    = dists.min(axis=0)
            pts      = pts[min_d <= 0.8]
        elif len(pts) > 0 and not confirmed:
            pts = np.empty((0, 4), dtype=np.float32)

        active_ids = set()
        for t in confirmed:
            tid = t["id"]; active_ids.add(tid)
            if tid not in self.persons:
                self.persons[tid] = PersonState(tid)
            # Extract radar-native Doppler velocities (default 0.0 if absent)
            vx = t.get("vx", 0.0)
            vy = t.get("vy", 0.0)
            vz = t.get("vz", 0.0)
            self.persons[tid].update(
                t["x"], t["y"], t["z"],
                vx, vy, vz,
                pts, self.zone,
                fw_height=t.get("fw_height"),
            )
        for gone in set(self.persons) - active_ids:
            del self.persons[gone]

        # ── Zone classification ────────────────────────────────────
        zone_live = {"safe": 0, "alert": 0, "restricted": 0}
        for p in self.persons.values():
            z = self._classify_zone(p)
            zone_live[z] += 1

        self._val_safe.setText(str(zone_live["safe"]))
        self._val_alert.setText(str(zone_live["alert"]))
        self._val_restricted.setText(str(zone_live["restricted"]))

        # ── Chamber breach detection (uses live bounds from _apply_chambers) ──
        ch_a_count = sum(1 for t in confirmed
                         if self._in_chamber(t["x"], t["y"], self._chamber_a))
        ch_b_count = sum(1 for t in confirmed
                         if self._in_chamber(t["x"], t["y"], self._chamber_b))

        self._val_ch_a.setText(str(ch_a_count))
        self._val_ch_b.setText(str(ch_b_count))

        ch_a_breach = ch_a_count > 0
        ch_b_breach = ch_b_count > 0

        # Update chamber status bar
        self._chamber_status.set_breach("UNIT A", ch_a_breach)
        self._chamber_status.set_breach("UNIT B", ch_b_breach)

        # Update chamber alert banners
        self.alert_chamber_a.set_active(ch_a_breach)
        self.alert_chamber_b.set_active(ch_b_breach)

        # ── Machine proximity — CRITICAL hazard ────────────────────
        # A person is "near the machine" when they are inside the unit
        # AND within MACHINE_PROX_RADIUS metres of the machine centre.
        mach_a_prox = any(
            self._near_machine(t["x"], t["y"], self._chamber_a)
            for t in confirmed
            if self._in_chamber(t["x"], t["y"], self._chamber_a)
        )
        mach_b_prox = any(
            self._near_machine(t["x"], t["y"], self._chamber_b)
            for t in confirmed
            if self._in_chamber(t["x"], t["y"], self._chamber_b)
        )

        # Fire critical log only on rising edge (False → True) to avoid spam
        if mach_a_prox and not self._mach_a_prox_prev:
            self._log("🔴 CRITICAL — Person within machine proximity: UNIT A", "error")
        if mach_b_prox and not self._mach_b_prox_prev:
            self._log("🔴 CRITICAL — Person within machine proximity: UNIT B", "error")
        if not mach_a_prox and self._mach_a_prox_prev:
            self._log("✔  Machine proximity cleared: UNIT A", "ok")
        if not mach_b_prox and self._mach_b_prox_prev:
            self._log("✔  Machine proximity cleared: UNIT B", "ok")

        self._mach_a_prox_prev = mach_a_prox
        self._mach_b_prox_prev = mach_b_prox

        self.alert_mach_a.set_active(mach_a_prox)
        self.alert_mach_b.set_active(mach_b_prox)

        # ── v2: Audio alerts (rising edge only via AlertManager) ──
        self._alert_mgr.on_frame(
            ch_a_breach    = ch_a_breach,
            ch_b_breach    = ch_b_breach,
            mach_a_prox    = mach_a_prox,
            mach_b_prox    = mach_b_prox,
            any_restricted = zone_live["restricted"] > 0,
        )

        # Count persons near each machine for live display
        mach_a_persons = sum(
            1 for t in confirmed
            if self._in_chamber(t["x"], t["y"], self._chamber_a)
            and self._near_machine(t["x"], t["y"], self._chamber_a)
        )
        mach_b_persons = sum(
            1 for t in confirmed
            if self._in_chamber(t["x"], t["y"], self._chamber_b)
            and self._near_machine(t["x"], t["y"], self._chamber_b)
        )
        self._val_mach_a.setText(str(mach_a_persons))
        self._val_mach_b.setText(str(mach_b_persons))

        # Pass proximity flags to 3D/floor widgets so they can highlight machine
        self.floor_plan.set_machine_prox(mach_a_prox, mach_b_prox)
        self.canvas3d.set_machine_prox(mach_a_prox, mach_b_prox)

        # ── v2: Stats tracker ─────────────────────────────────────
        self._stats.update(point_count=len(pts), track_count=len(confirmed))
        self.card_fps.set_value(self._stats.fps_str)
        self.card_pts.set_value(self._stats.points_str)
        self.card_trk.set_value(self._stats.tracks_str)

        # ── v2: CSV data logging ──────────────────────────────────
        self._data_logger.log_frame(
            frame_num   = frame.frame_num,
            targets     = confirmed,
            persons     = self.persons,
            zone_counts = zone_live,
            ch_a_breach = ch_a_breach,
            ch_b_breach = ch_b_breach,
            mach_a_prox = mach_a_prox,
            mach_b_prox = mach_b_prox,
        )

        self._zone_history.append((
            zone_live["safe"],
            zone_live["alert"],
            zone_live["restricted"],
        ))
        self._update_zone_chart()

        any_restricted = zone_live["restricted"] > 0
        self.alert_hazard.set_active(any_restricted)

        self.card_count.set_value(len(self.persons))
        if now - self._window_start_ts > 120.0:
            self._total_seen_ids  = set()
            self._window_start_ts = now
        self._total_seen_ids.update(active_ids)
        self.card_total.set_value(len(self._total_seen_ids))

        # ── Update all views ───────────────────────────────────────
        self.canvas3d.update_scene(pts, confirmed, self.persons)
        self.floor_plan.update_scene(pts, confirmed, self.persons)
        self.radar_full.update_scene(pts, confirmed, self.persons)
        self._mini_radar.set_scene(
            pts, confirmed, self.persons,
            self._safe_max, self._alert_max, self._restricted_max
        )

    # ── Demo data: animated dummy scene shown even without radar ──────
    def _push_demo_frame(self):
        """Generate synthetic persons + point cloud and push to the
        Tab-2 widgets so the operator can see expected output at all
        times, even before the radar is connected.
        When the radar IS live the demo timer keeps running but its
        updates are immediately overwritten by _on_frame 18+ fps so
        they are invisible in practice."""
        t = self._demo_tick * 0.06
        self._demo_tick += 1

        # ── Synthetic targets (3 persons walking scripted paths) ──
        targets = [
            {   # Person 0 — slow loop in the open area
                "id": 0,
                "x": float(2.8 * math.sin(t * 0.25)),
                "y": float(2.5 + 1.5 * math.cos(t * 0.18)),
                "z": float(1.6 + 0.05 * math.sin(t)),
            },
            {   # Person 1 — approaches Unit A chamber
                "id": 1,
                "x": float(-3.2 + 0.6 * math.sin(t * 0.35)),
                "y": float(3.0 + 2.5 * abs(math.sin(t * 0.15))),
                "z": float(1.65),
            },
            {   # Person 2 — paces near Unit B, occasionally enters
                "id": 2,
                "x": float(3.1 + 0.5 * math.cos(t * 0.28)),
                "y": float(3.8 + 2.0 * abs(math.sin(t * 0.20))),
                "z": float(1.7 + 0.04 * math.cos(t * 1.3)),
            },
        ]

        # ── Synthetic PersonState objects ──────────────────────────
        persons = {}
        for td in targets:
            ps = PersonState(td["id"])
            ps.x = td["x"]; ps.y = td["y"]; ps.z = td["z"]
            ps.height = td["z"] + 0.05 * math.sin(t + td["id"])
            ps.in_hazard = False
            persons[td["id"]] = ps

        # ── Synthetic point cloud (~40 pts scattered around targets) ─
        rng = np.random.default_rng(self._demo_tick % 1000)
        pt_list = []
        for td in targets:
            n = 12
            ox = rng.normal(td["x"], 0.25, n).astype(np.float32)
            oy = rng.normal(td["y"], 0.25, n).astype(np.float32)
            oz = rng.uniform(0.0, td["z"], n).astype(np.float32)
            od = np.ones(n, dtype=np.float32)
            pt_list.append(np.column_stack([ox, oy, oz, od]))
        # A handful of scattered background points
        bx = rng.uniform(-4, 4, 8).astype(np.float32)
        by = rng.uniform(0.5, 7, 8).astype(np.float32)
        bz = rng.uniform(0, 0.3, 8).astype(np.float32)
        bd = np.ones(8, dtype=np.float32)
        pt_list.append(np.column_stack([bx, by, bz, bd]))
        pts = np.vstack(pt_list)

        # Only update Tab-2 widgets when the radar worker is NOT live
        # (if worker is running, _on_frame handles it at higher fps)
        if self.worker is None or not self.worker.isRunning():
            self.canvas3d.update_scene(pts, targets, persons)
            self.radar_full.update_scene(pts, targets, persons)

    def _apply_chambers(self):
        """Read spinboxes, validate, push new chamber bounds to all widgets."""
        ax0 = self._ch_spins["A_x0"].value()
        ax1 = self._ch_spins["A_x1"].value()
        ay0 = self._ch_spins["A_y0"].value()
        ay1 = self._ch_spins["A_y1"].value()
        bx0 = self._ch_spins["B_x0"].value()
        bx1 = self._ch_spins["B_x1"].value()
        by0 = self._ch_spins["B_y0"].value()
        by1 = self._ch_spins["B_y1"].value()

        # Validate: min < max for each axis
        errs = []
        if ax0 >= ax1: errs.append("Unit A: X min must be < X max")
        if ay0 >= ay1: errs.append("Unit A: Y min must be < Y max")
        if bx0 >= bx1: errs.append("Unit B: X min must be < X max")
        if by0 >= by1: errs.append("Unit B: Y min must be < Y max")
        if ax1 > 0 and bx0 < 0:
            errs.append("Chambers A and B overlap in X — check bounds")

        if errs:
            for e in errs:
                self._log(f"⚠  {e}", "warn")
            return

        # Store on MainWindow (used by _on_frame breach detection)
        self._chamber_a = (ax0, ax1, ay0, ay1)
        self._chamber_b = (bx0, bx1, by0, by1)

        # Push to IndustrialFloorPlanWidget (Tab 1)
        self.floor_plan.CHAMBER_A = self._chamber_a
        self.floor_plan.CHAMBER_B = self._chamber_b
        self.floor_plan.update()

        # Push to IndustrialFloorPlanWidget (Tab 2 left side)
        self.canvas3d.CHAMBER_A = self._chamber_a
        self.canvas3d.CHAMBER_B = self._chamber_b
        self.canvas3d.update()

        # Update live bounds label
        self._ch_bounds_lbl.setText(
            f"A: X[{ax0:+.1f}, {ax1:+.1f}]  Y[{ay0:.1f}, {ay1:.1f}]\n"
            f"B: X[{bx0:+.1f}, {bx1:+.1f}]  Y[{by0:.1f}, {by1:.1f}]")

        self._log(
            f"Chambers updated → "
            f"A: X[{ax0:+.1f}→{ax1:+.1f}] Y[{ay0:.1f}→{ay1:.1f}]  "
            f"B: X[{bx0:+.1f}→{bx1:+.1f}] Y[{by0:.1f}→{by1:.1f}]",
            "ok")

    def _apply_mach_prox_radius(self):
        """Update the live machine proximity radius from the spinbox."""
        global MACHINE_PROX_RADIUS
        MACHINE_PROX_RADIUS = self._mach_prox_spin.value()
        self._log(
            f"Machine proximity radius → {MACHINE_PROX_RADIUS:.1f} m  "
            f"(persons within this distance of machine = CRITICAL)", "warn")

    def _apply_zone(self):
        self._safe_min       = self._zone_spins["safe_min"].value()
        self._safe_max       = self._zone_spins["safe_max"].value()
        self._alert_min      = self._zone_spins["alert_min"].value()
        self._alert_max      = self._zone_spins["alert_max"].value()
        self._restricted_min = self._zone_spins["restricted_min"].value()
        self._restricted_max = self._zone_spins["restricted_max"].value()

        self.zone.update(
            -self._restricted_min, self._restricted_min,
            self._restricted_min,  self._restricted_max,
            0.0, 3.0
        )
        self.canvas3d.safe_r       = self._safe_max
        self.canvas3d.alert_r      = self._alert_max
        self.canvas3d.restricted_r = self._restricted_max
        self.canvas3d.view_range   = self._restricted_max + 1.5
        self.canvas3d.refresh_zones()

        self.floor_plan.safe_r       = self._safe_max
        self.floor_plan.alert_r      = self._alert_max
        self.floor_plan.restricted_r = self._restricted_max
        self.floor_plan.refresh_zones()

        self.radar_full.safe_r       = self._safe_max
        self.radar_full.alert_r      = self._alert_max
        self.radar_full.restricted_r = self._restricted_max
        self.radar_full.refresh_zones()

        self._mini_radar.set_scene(
            np.empty((0, 4), dtype=np.float32), [], {},
            self._safe_max, self._alert_max, self._restricted_max
        )

        self._log(
            f"Zones updated → Safe[{self._safe_min:.1f}-{self._safe_max:.1f}m]  "
            f"Alert[{self._alert_min:.1f}-{self._alert_max:.1f}m]  "
            f"Restricted[{self._restricted_min:.1f}-{self._restricted_max:.1f}m]", "info")

    def _log(self, msg, level="info"):
        col_map = {"info": CYAN_INFO, "ok": PHOSPHOR, "error": RED_ALERT,
                   "tx": AMBER, "dim": SUBTEXT, "warn": AMBER}
        col = col_map.get(level, WHITE_TEXT)
        ts  = time.strftime("%H:%M:%S")
        self.cli_log.append(
            f'<span style="color:{SUBTEXT}">[{ts}]</span>'
            f' <span style="color:{col}">{msg}</span>'
        )
        sb = self.cli_log.verticalScrollBar()
        sb.setValue(sb.maximum())
        # ── v2: mirror to rotating log file ──────────────────────
        log_fn = {"error": _log.error, "warn": _log.warning}.get(level, _log.info)
        log_fn(msg)

    def _set_status(self, text, color):
        self.lbl_status.setText(f"⬤  {text}")
        self.lbl_status.setStyleSheet(
            f"color:{color};font-size:12px;font-weight:700;font-family:{MONO_FONT};")

    def closeEvent(self, ev):
        self._on_stop()
        # ── v2: save settings and flush CSV on close ──────────────
        self._data_logger.stop()
        self._settings.save()
        _log.info("Application closed — settings saved, CSV flushed")
        super().closeEvent(ev)

def main():
    _setup_hidpi()

    # QApplication.setAttribute must be called BEFORE QApplication()
    from PyQt5.QtWidgets import QApplication as _QApp
    _QApp.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    _QApp.setAttribute(Qt.AA_UseHighDpiPixmaps, True)

    app = QApplication(sys.argv)

    # Cross-platform default font
    _sys = platform.system()
    if _sys == "Darwin":
        app.setFont(QFont("Helvetica Neue", 10))
    elif _sys == "Linux":
        app.setFont(QFont("DejaVu Sans", 10))
    else:
        app.setFont(QFont("Arial", 10))

    # Fusion style gives consistent look across Windows, macOS, Linux
    app.setStyle("Fusion")

    win = MainWindow()
    win.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
