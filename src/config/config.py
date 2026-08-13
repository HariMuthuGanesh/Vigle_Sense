"""
config.py — Centralised configuration for Vigil_Sense.

All tunable constants live here so they can be found and adjusted
without scrolling through the main application file.
"""

# ── Serial / Radar protocol ──────────────────────────────────────
MAGIC_WORD  = bytes([0x02, 0x01, 0x04, 0x03, 0x06, 0x05, 0x08, 0x07])
HEADER_SIZE = 40
CLI_BAUD    = 115200
DATA_BAUD   = 921600

# ── TLV type identifiers ─────────────────────────────────────────
TLV_POINT_CLOUD  = 1020
TLV_TARGET_LIST  = 1010
TLV_TARGET_IDX   = 1011
TLV_TARGET_HEIGHT = 1012
TLV_PRESENCE     = 1021

# ── Person / height tracking ─────────────────────────────────────
HEIGHT_BIAS  = 0.15   # metres — algorithmic bias correction (+15 cm) for missing head
HEIGHT_ALPHA = 0.2    # IIR filter weight for height smoothing (replaces mean window)

# ── Safety zones ─────────────────────────────────────────────────
MACHINE_PROX_RADIUS = 1.0   # metres — distance from machine centre = critical hazard

# ── Colour palette (industrial dark theme) ───────────────────────
BG          = "#0a0d0a"
PANEL       = "#0d1210"
BORDER      = "#1a2a1a"
PHOSPHOR    = "#39ff14"
DIM_GREEN   = "#1f6b10"
AMBER       = "#ffb300"
RED_ALERT   = "#ff2020"
CYAN_INFO   = "#00e5cc"
WHITE_TEXT  = "#d4e8d4"
SUBTEXT     = "#4a6b4a"
GRID_COL    = "#14201a"

# Industrial floor plan colours
FLOOR_BG        = "#0c1a0c"
FLOOR_GRID      = "#132213"
FLOOR_WALL      = "#1e3c1e"
FLOOR_WALL_LIT  = "#2a5a2a"
CHAMBER_FILL    = "#1a0505"
CHAMBER_WALL    = "#8b0000"
CHAMBER_GLOW    = "#ff1a1a"
CHAMBER_STRIPE  = "#3a0000"
SAFE_FILL       = "#001a00"
ALERT_FILL      = "#1a1000"
RESTRICTED_FILL = "#1a0000"
MACHINE_COL     = "#161628"
MACHINE_BORDER  = "#2a4a2a"

# ── Default radar configuration (AOP 6 m, People Tracking) ───────
AOP_6M_DEFAULT_CFG = """\
sensorStop
flushCfg
dfeDataOutputMode 1
channelCfg 15 7 0
adcCfg 2 1
adcbufCfg -1 0 1 1 1
lowPower 0 0
profileCfg 0 60.75 30.00 25.00 59.10 394758 0 54.71 1 96 2950.00 2 1 36 
chirpCfg 0 0 0 0 0 0 0 1
chirpCfg 1 1 0 0 0 0 0 2
chirpCfg 2 2 0 0 0 0 0 4
frameCfg 0 2 96 0 55.00 1 0
dynamicRACfarCfg -1 4 4 2 2 8 12 4 12 5.00 8.00 0.40 1 1
staticRACfarCfg -1 6 2 2 2 8 8 6 4 8.00 15.00 0.30 0 0
dynamicRangeAngleCfg -1 0.75 0.0010 1 0
dynamic2DAngleCfg -1 3.0 0.0300 1 0 1 0.30 0.85 8.00
staticRangeAngleCfg -1 0 8 8
antGeometry0 -1 -1 0 0 -3 -3 -2 -2 -1 -1 0 0
antGeometry1 -1 0 -1 0 -3 -2 -3 -2 -3 -2 -3 -2
antPhaseRot 1 -1 1 -1 1 -1 1 -1 1 -1 1 -1
fovCfg -1 70.0 70.0
compRangeBiasAndRxChanPhase 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0
staticBoundaryBox -3 3 0.5 7.5 0 3
boundaryBox -4 4 0 8 0 3
sensorPosition 2 0 15
gatingParam 3 2 2 3 4
stateParam 3 3 12 500 5 6000
allocationParam 20 100 0.1 20 0.5 20
maxAcceleration 0.1 0.1 0.1
trackingCfg 1 2 800 30 46 96 55
presenceBoundaryBox -3 3 0.5 7.5 0 3
sensorStart
"""

# 9 m config is identical to 6 m for now
AOP_9M_DEFAULT_CFG = AOP_6M_DEFAULT_CFG 
