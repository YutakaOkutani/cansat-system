import os
from enum import IntEnum

from lib.cone_diagnostics import CONE_DIAGNOSTIC_LOG_COLUMNS
from mission.paths import DEFAULT_RUNS_ROOT, ROI_CAPTURE_DIR as RUNTIME_ROI_CAPTURE_DIR
from mission.paths import ROI_PRIMARY_REFERENCE, ROI_REFERENCE_DIR as VERSIONED_ROI_REFERENCE_DIR


# 定数定義
# フェーズ定義
class Phase(IntEnum):
    PHASE0 = 0
    PHASE1 = 1
    PHASE2 = 2
    PHASE3 = 3
    PHASE4 = 4
    PHASE5 = 5
    PHASE6 = 6
    PHASE7 = 7

# ログ関連定数。新規実行は run bundle を作成するため、LOG_DIR は互換用の基底値。
LOG_DIR = str(DEFAULT_RUNS_ROOT)
LOG_PREFIX = "robust_log_"
LOG_FILE_DATETIME_FORMAT = "%Y-%m%d-%H%M%S"
MISSION_LOG_SCHEMA_VERSION = 1

# 大会／ミッション固有の制御契約。
# 現在値はNSE2026で検証された現行ミッションを表し、由来は
# docs/competitions/nse2026.md に記録する。大会間で暗黙に共通扱いしない。
# タイムアウトと動作関連定数
TIMEOUT_PHASE_0 = 2 * 60
TIMEOUT_PHASE_1 = 10
# Offset成立を優先する。総合13分のうち、Phase3に最低3分を残せる上限。
TIMEOUT_PHASE_2 = 90
TIMEOUT_PHASE_3 = 5 * 60
TIMEOUT_PHASE_4 = 60
TIMEOUT_PHASE_5 = 60
DATA_SAMPLING_RATE = 0.20
GRASS_MIN_MOTOR_SPEED = 45
# 現機体の寄り不足への初期調整。中央合わせ後、右輪ゲイン込みでも65%以上で押す。
PHASE6_RAM_SPEED = 75
PHASE6_RAM_DURATION_SEC = 5.0
PHASE6_RAM_RAMP_TIME = 0.05

# ミッション全体のフェーズ累積予算
# Phase3-5 は再入を考慮して、個別タイムアウトより大きい累積値を持たせる。
# 投下から15分以内の大会制限に対し、systemd 起動遅延を見込んで
# プログラム開始から13分で give up する。
MISSION_TIMEOUT_TARGET_TOTAL = 13 * 60
MISSION_PHASE3_MIN_RESERVE_SEC = 3 * 60
MISSION_TIMEOUT_TRANSITION_GRACE_SEC = 5.0
MISSION_PHASE4_CUMULATIVE_BUDGET = 90
MISSION_PHASE5_CUMULATIVE_BUDGET = 90
MISSION_PHASE3_CUMULATIVE_BUDGET = (
    MISSION_TIMEOUT_TARGET_TOTAL
    - TIMEOUT_PHASE_0
    - TIMEOUT_PHASE_1
    - TIMEOUT_PHASE_2
    - MISSION_PHASE4_CUMULATIVE_BUDGET
    - MISSION_PHASE5_CUMULATIVE_BUDGET
    - PHASE6_RAM_DURATION_SEC
    - MISSION_TIMEOUT_TRANSITION_GRACE_SEC
)
if MISSION_PHASE3_CUMULATIVE_BUDGET < MISSION_PHASE3_MIN_RESERVE_SEC:
    raise ValueError("Phase3 cumulative budget is below the required navigation reserve")
MISSION_PHASE_TIME_BUDGETS = {
    Phase.PHASE0: TIMEOUT_PHASE_0,  # 降下待機
    Phase.PHASE1: TIMEOUT_PHASE_1,  # パラ分離
    Phase.PHASE2: TIMEOUT_PHASE_2,  # キャリブレーション
    Phase.PHASE3: MISSION_PHASE3_CUMULATIVE_BUDGET,  # GPS航行(再入込み)
    Phase.PHASE4: MISSION_PHASE4_CUMULATIVE_BUDGET,  # カメラ探索(複数回)
    Phase.PHASE5: MISSION_PHASE5_CUMULATIVE_BUDGET,  # 接近(複数回)
}
MISSION_PHASE_BUDGET_TOTAL = sum(MISSION_PHASE_TIME_BUDGETS.values())
# 全体タイムアウトは「フェーズ累積予算合計 + 最終突入 + 遷移吸収マージン」で導出する。
MISSION_TIMEOUT_TOTAL = (
    MISSION_PHASE_BUDGET_TOTAL
    + PHASE6_RAM_DURATION_SEC
    + MISSION_TIMEOUT_TRANSITION_GRACE_SEC
)

MISSION_PHASE_TIMEOUT_TRANSITIONS = {
    Phase.PHASE0: Phase.PHASE1,
    Phase.PHASE1: Phase.PHASE2,
    Phase.PHASE2: Phase.PHASE3,
    Phase.PHASE3: Phase.PHASE4,
    Phase.PHASE4: Phase.PHASE7,
    Phase.PHASE5: Phase.PHASE7,
    Phase.PHASE6: Phase.PHASE7,
}

# センサーと動作の閾値

# Phase0: carrier release / descent detection tuning
# 現場調整はまずこのブロックだけを見る。
#
# 高度検知:
#   Phase0突入後、最初の有効なBMP高度を基準高度として保存する。
#   `基準高度 - 現在高度` が DROP_ALTITUDE_DIFF_THRESHOLD [m] を超えると
#   落下候補としてラッチし、PHASE0_DROP_TO_PHASE1_DELAY_SEC 秒後にPhase1へ進む。
#   誤検知が多い場合は上げる。落としても高度で拾えない場合は下げる。
#
# 衝撃検知:
#   `Fall` ログ列はBNOの3軸加速度ノルムで、静止時も重力込みで約9.8m/s^2になる。
#   通常は PHASE0_IMPACT_DELTA_THRESHOLD で「静止baselineからのズレ」を見る。
#   誤検知が多い場合は上げる。開傘/着地衝撃を拾えない場合は下げる。
#   IMPACT_FALL_THRESHOLD は非常に大きい衝撃用の絶対値ガードとして残す。
#   PHASE0_IMPACT_CONFIRM_SAMPLES は衝撃候補の連続サンプル数。1にすると敏感、
#   3以上にするとノイズに強いが短い衝撃を逃しやすい。
#   PHASE0_ACCEL_BASELINE_ALPHA はbaseline追従速度。通常は触らない。
#   大きくすると姿勢変化への追従が速いが、ゆっくりした衝撃を吸収しやすい。
#   小さくするとbaselineが安定するが、長い姿勢変化には鈍くなる。
#   PHASE0_DROP_TO_PHASE1_DELAY_SEC は検知後に開傘/着地を待つ時間。
IMPACT_FALL_THRESHOLD = 30.0
PHASE0_IMPACT_DELTA_THRESHOLD = 6.0
PHASE0_IMPACT_CONFIRM_SAMPLES = 2
PHASE0_ACCEL_BASELINE_ALPHA = 0.08
DROP_ALTITUDE_DIFF_THRESHOLD = 20.0
PHASE0_DROP_TO_PHASE1_DELAY_SEC = 20.0

# コーン検出関連定数
CONE_PROBABILITY_THRESHOLD = 0.10
# Field runs show the cone is consistently boxed around prob~=0.23 in Phase4.
# Phase4 should accept that signal instead of holding forever on "low-confidence".
CONE_PROBABILITY_THRESHOLD_PHASE4 = 0.20
CONE_PROBABILITY_THRESHOLD_PHASE5 = 0.18
CONE_PHASE4_CONFIRM_FRAMES = 3
CONE_PHASE4_CENTER_TOLERANCE = 0.42
CONE_PHASE4_DIRECTION_CONSISTENCY_TOLERANCE = 0.22
CONE_PHASE4_BEARING_CONSISTENCY_TOLERANCE_DEG = 25.0
CONE_PHASE4_BBOX_CENTER_Y_TOLERANCE = 0.35
CONE_PHASE4_BBOX_SIZE_RATIO_MAX = 3.50
CONE_PHASE4_BBOX_VERTICAL_OVERLAP_MIN = 0.10
# 互換性のため残す旧値。到達判定には使用せず、close_reached_ok を正とする。
CONE_PHASE4_REACHED_PROBABILITY_THRESHOLD = 0.28
CONE_PHASE5_REACHED_PROBABILITY_THRESHOLD = 0.30
CONE_PHASE5_REACH_CONFIRM_FRAMES = 2
CONE_PHASE4_STRONG_PROBABILITY = 0.30
CONE_LOST_COUNT_LIMIT = 10
CONE_CENTER_POSITION = 0.5

# パラシュート展開関連定数
PARACHUTE_DIRECTION = -400.0
PARACHUTE_SEPARATION_SPEED = 100
PARACHUTE_MOTOR_PULSE = 0.05
# フェーズ1はサブキャリア内で初動トルクが大きくなりやすいため、
# 他フェーズより長いランプ時間と細かいステップで突入電流を抑える。
PHASE1_SOFTSTART_RAMP_TIME = 1.2
PHASE1_SOFTSTART_STEP = 0.01

# キャリブレーション関連定数
CALIBRATION_TURN_SPEED = 60
CALIBRATION_MAG_THRESHOLD = 2

# 障害物回避関連定数
OBSTACLE_AVOID_DIST = 30.0
OBSTACLE_CONFIRM_COUNT = 3
OBSTACLE_SPEED = 60
OBSTACLE_BACKUP_TIME = 1.0
OBSTACLE_TURN_TIME = 0.5
OBSTACLE_PAUSE_TIME = 0.2

# カメラ関連定数
CAMERA_ACTIVE_SLEEP = 0.02
CAMERA_IDLE_SLEEP = 0.5
CAMERA_REINIT_INTERVAL = 5.0
CAMERA_FAIL_LIMIT = 5
CAMERA_DEAD_TIMEOUT = 30.0
CAMERA_REINIT_MAX_ATTEMPTS = 3
# Motor control must not depend on the camera thread returning from a blocked
# capture call.  The ROI-enabled field rate was 1.8 fps (p95 interval 0.72 s),
# so allow normal processing jitter while still bounding a frozen camera.
CAMERA_FRAME_STALE_STOP_SEC = 1.20
# Phase4では再初期化3回に加え、最後の復帰確認のため最低15秒の猶予を持たせる。
CAMERA_RECOVERY_GRACE_SEC = 15.0
CAMERA_CONTROL_INVERT_X = False
CAMERA_PHASE5_MAX_ATTEMPTS = 3
CAMERA_TEMPORAL_MIN_PROBABILITY = 0.12
CAMERA_TEMPORAL_STRONG_PROBABILITY = 0.42
CAMERA_TEMPORAL_DIR_JUMP_MAX = 0.18
CAMERA_TEMPORAL_CONFIRM_FRAMES = 2
CAMERA_TEMPORAL_HOLD_SEC = 0.35
CAMERA_TEMPORAL_DIR_FILTER_ALPHA = 0.45
CAMERA_HORIZONTAL_FOV_DEG = 62.0
CAMERA_WEAK_MIN_CANDIDATE_PROBABILITY = 0.18
CAMERA_WEAK_MIN_SHAPE_SCORE = 0.32
CAMERA_WEAK_MIN_HUE_SCORE = 0.38
CAMERA_WEAK_MIN_SV_SCORE = 0.28
CAMERA_WEAK_STRONG_HUE_SCORE = 0.48
CAMERA_WEAK_RELAXED_SV_SCORE = 0.20
CAMERA_WEAK_MIN_ROI_SUPPORT = 0.10
CAMERA_WEAK_MIN_ROI_ABSOLUTE_SUPPORT = 0.08
# A single component below this occupancy is too small to steer the rover.
# Field evidence separated real cones (>=0.0051) from grass false positives
# (<=0.00134), leaving a deliberate margin at 0.003.
CAMERA_TINY_OCCUPANCY_THRESHOLD = 0.003
CAMERA_TINY_MIN_CONSISTENT_FRAMES = 3
# Phase transition requires three physical frames for both mixed/strict and
# weak tracks. Motor recentering also waits for the three-frame observation
# window, with at least two credible consistent observations inside it.
CAMERA_MIXED_CONFIRM_FRAMES = 3
CAMERA_WEAK_CONFIRM_FRAMES = 3
CAMERA_WEAK_MAX_MISSED_FRAMES = 2

# ROI画像パス。参照画像はバージョン管理対象の制御入力、capturesは生成物。
# 検知用の参照画像と roi_capture.py のタイムスタンプ付き撮影画像は
# 別ディレクトリに保存し、試し撮りが検知入力に混ざらないようにする。
ROI_REFERENCE_DIR = str(VERSIONED_ROI_REFERENCE_DIR)
ROI_CAPTURE_DIR = str(RUNTIME_ROI_CAPTURE_DIR)
ROI_PATH_1 = str(ROI_PRIMARY_REFERENCE)
# 後方互換のため残すが、実体は同一ファイルを指す
ROI_PATH_2 = ROI_PATH_1
ROI_GLOB_PATTERNS = (
    "roi_cone*.png",
    "roi_cone*.jpg",
    "roi_cone*.jpeg",
    "false_*.png",
    "false_*.jpg",
    "false_*.jpeg",
    "fake_*.png",
    "fake_*.jpg",
    "fake_*.jpeg",
)

# モーター制御関連定数
PHASE4_SEARCH_OUTER_SPEED = 60
# 芝生で低速輪がストールしないよう、Phase4 の全動作で最低45%を保つ。
PHASE4_SEARCH_INNER_SPEED = GRASS_MIN_MOTOR_SPEED
# Candidate observation is governed by fresh-frame count.  The time limit is
# only a safety bound when detector throughput drops.
PHASE4_CANDIDATE_CAPTURE_SEC = 2.50
PHASE4_CANDIDATE_CAPTURE_MIN_FRAMES = 3
PHASE4_CANDIDATE_CAPTURE_MAX_FRAMES = 5
PHASE4_CANDIDATE_TRACK_CONFIRM_FRAMES = 2
PHASE4_CANDIDATE_OUTER_SPEED = 100
PHASE4_CANDIDATE_INNER_SPEED = GRASS_MIN_MOTOR_SPEED
PHASE4_REACQUIRE_TURN_SEC = 0.80
PHASE4_TRACK_ROTATE_GAIN = 240
PHASE4_TRACK_ROTATE_CLAMP = 55
PHASE4_TRACK_MIN_ROTATE_SPEED = 5
PHASE4_ALIGN_STOP_DEADBAND = 0.04
PHASE4_ALIGN_ARC_DEADBAND = 0.18
PHASE4_ALIGN_FORWARD_SPEED = 80
PHASE4_ALIGN_PIVOT_SPEED = 85
PHASE4_ALIGN_INNER_SPEED = GRASS_MIN_MOTOR_SPEED
PHASE4_ALIGN_ARC_BASE_SPEED = 80
PHASE4_ALIGN_ARC_TURN_GAIN = 30
APPROACH_TURN_GAIN = 200
# 画面端への逸脱時は外輪100%・内輪停止で向きを戻す。
# 65%下限の前進弧では旋回力が飽和するため、画像誤差で切り替える。
CAMERA_EDGE_TURN_ERROR = 0.20
CAMERA_EDGE_TURN_RELEASE_ERROR = 0.12
CAMERA_EDGE_TURN_SPEED = 100
# Phase5は画面内の赤占有率に応じて段階的に減速する。近距離検知と
# 中央合わせで最終突撃の確認待ちに入り、通常の接近中は
# 右輪ゲイン補正後も65%以上を維持する。実走に合わせた初期調整値。
PHASE5_BASE_SPEED = 85
PHASE5_MID_SPEED = 80
PHASE5_NEAR_SPEED = 75
PHASE5_MID_OCCUPANCY_THRESHOLD = 0.02
PHASE5_NEAR_OCCUPANCY_THRESHOLD = 0.08
PHASE5_TURN_CLAMP = 40
PHASE5_STEER_DEADBAND = 0.03
# 近距離検知に加え、画像中心からのずれが画面幅の8%以内で最終突撃へ進む。
PHASE5_RAM_CENTER_TOLERANCE = 0.08
# Keep steering briefly toward the last reliable image position when one or
# two frames are lost.  Stopping immediately leaves the camera looking away
# from a cone that crossed the image edge during the P4 -> P5 handoff.
PHASE5_REACQUIRE_GRACE_SEC = 0.75
PHASE5_REACQUIRE_OUTER_SPEED = 85
PHASE5_REACQUIRE_INNER_SPEED = GRASS_MIN_MOTOR_SPEED
PHASE45_CONE_DIR_FILTER_ALPHA = 0.72
PHASE4_MOTOR_RAMP_TIME = 0.06
PHASE5_MOTOR_RAMP_TIME = 0.05
PHASE45_MOTOR_LOOP_INTERVAL = 0.02
BASE_SPEED = 100
PHASE3_NO_HEADING_SPEED = 70
PHASE3_NO_HEADING_TURN_BIAS = 0
PHASE3_NO_HEADING_TURN_INTERVAL = 2.5
PHASE3_FORWARD_SPEED = 70
PHASE3_TURN_OUTER_SPEED = 75
PHASE3_TURN_INNER_SPEED = 50
PHASE3_HEADING_DEADBAND_DEG = 14.0
PHASE3_GPS_FALLBACK_DEADBAND_DEG = 20.0
PHASE3_GPS_FALLBACK_TURN_SCALE = 0.55
PHASE3_GPS_ARC_BASE_SPEED = 70
PHASE3_GPS_ARC_MIN_DELTA = 6
PHASE3_GPS_ARC_MAX_DELTA = 20
PHASE3_GPS_ARC_KP = 0.14
PHASE3_GPS_ARC_MAX_HOLD_SEC = 1.6
PHASE3_MOTOR_LOOP_INTERVAL = 0.40
PHASE3_LARGE_ERROR_DEG = 75.0
PHASE3_LARGE_ERROR_OUTER_SPEED = 85
PHASE3_LARGE_ERROR_INNER_SPEED = 45
PHASE3_MAG_STUCK_TIMEOUT_SEC = 1.5
PHASE3_MAG_STUCK_MIN_DELTA_DEG = 3.0
PHASE3_TURN_FAST_SPEED = 100
PHASE3_TURN_SLOW_SPEED = 75
PHASE3_TURN_FULL_ERROR_DEG = 45.0
# Legacy values retained only for old log-analysis imports. Phase3 no longer
# commands a stopped inner wheel on large heading errors.
PHASE3_PIVOT_THRESHOLD_DEG = PHASE3_LARGE_ERROR_DEG
PHASE3_PIVOT_SPEED = PHASE3_LARGE_ERROR_OUTER_SPEED
PHASE3_PIVOT_SLOW_MIN_SPEED = PHASE3_LARGE_ERROR_INNER_SPEED
PHASE3_BNO_GPS_OFFSET_ALPHA = 0.2
PHASE3_BNO_GPS_OFFSET_MAX_STEP_DEG = 45.0
PHASE3_BNO_TRUST_MAX_STALE_SEC = 0.6
PHASE3_BNO_TRUST_MAX_OFFSET_DEG = 180.0
PHASE3_BNO_TRUST_MAX_JUMP_DEG = 75.0
PHASE3_NO_HEADING_TIMEOUT_SEC = 3.0
PHASE3_ARRIVAL_CONFIRM_COUNT = 3
PHASE3_ARRIVAL_CONFIRM_SEC = 1.0
PHASE3_FORWARD_RAMP_TIME = 0.2
PHASE3_TURN_RAMP_TIME = 0.1
MOTOR_LOOP_INTERVAL = 0.05
MOTOR_IDLE_SLEEP = 0.1
MOTOR_RAMP_TIME = 0.6
MOTOR_RAMP_STEP = 0.05
# 機体固有値: 現行単一機体のモーター配線・個体差に合わせた固定値。
MOTOR_DIR_INVERT_1 = True
MOTOR_DIR_INVERT_2 = False
# 走行会前のWASD実機確認に基づく配線: 物理MTR1が左輪、物理MTR2が右輪。
# 操舵ロジックは常に「左輪、右輪」の順で指令し、この対応で物理chへ変換する。
MOTOR_LEFT_MTR_INDEX = 1
MOTOR_RIGHT_MTR_INDEX = 2
MANUAL_TURN_SPEED_RATIO = 3.0 / 5.0
# 旋回時の定常PWM下限。ゲイン補正後に同比率で引き上げ、100%で制限する。
MOTOR_TURN_MIN_SPEED = 65.0
# モーター個体差補正 (PWM指令値に乗算)
MOTOR_SPEED_SCALE_1 = 1.00
MOTOR_SPEED_SCALE_2 = 0.90  # 速い右輪を10%減速する初期調整値。実走で再調整する。
# モーター個体差補正 (PWM指令値に加算, scale適用後)
# 追加調整はまず各輪のMOTOR_SPEED_SCALEで行う。
MOTOR_SPEED_OFFSET_1 = 0.0
MOTOR_SPEED_OFFSET_2 = 0.0

# LED関連定数
LED_INTERVAL_PHASE0 = 5
LED_INTERVAL_PHASE2 = 3
LED_INTERVAL_PHASE3 = 10
LED_INTERVAL_PHASE3_NEAR = 2
LED_INTERVAL_PHASE5 = 2
LED_SIGNAL_SLEEP = 0.2
LED_SIGNAL_COUNT = 3
LED_TIMEOUT_ALERT_FLASH_COUNT = 24
LED_TIMEOUT_ALERT_FAST_SLEEP = 0.08

RADIO_COMMAND_TIMEOUT_SEC = 10.0

# 機体固有値: モーター・センサーのGPIO関連定数
# Physical motor channels (fixed wiring): MTR1=LEFT, MTR2=RIGHT
PIN_EN1 = 12
PIN_PH1 = 13
PIN_EN2 = 19
PIN_PH2 = 17
PWM_FREQ = 1000
PIN_LED_RED = 6
PIN_LED_GREEN = 5
PIN_TRIG = 23
PIN_ECHO = 24
SONAR_MAX_DISTANCE = 4.0
# 5 sampling periods (DATA_SAMPLING_RATE=0.2s). Older readings are not used for avoidance.
SONAR_STALE_TIMEOUT_SEC = 1.0

# GPS関連定数
GPS_SERIAL_PORT = "/dev/serial0"
GPS_SERIAL_PORT_CANDIDATES = ["/dev/serial0", "/dev/ttyAMA0", "/dev/ttyS0"]
GPS_BAUDRATE = 9600
# Common GPS module UART rates (u-blox often ships at 9600/38400, some setups use 115200).
GPS_BAUDRATE_CANDIDATES = [9600, 38400, 115200, 57600, 19200, 4800]
GPS_SERIAL_TIMEOUT = 1
GPS_SERIAL_DISCOVERY_TIMEOUT = 0.2
GPS_HEADING_OFFSET = 5.43
GPS_TURN_GAIN = 0.3
GPS_TURN_CLAMP = 30.0
GPS_CLOSE_DISTANCE = 3.0
GPS_PHASE45_MAX_DISTANCE = 15.0
GPS_BUFFER_CLEAR_THRESHOLD = 2048
GPS_BUFFER_CLEAR_INTERVAL = 5.0
GPS_MIN_FIX_QUAL = 1
GPS_MIN_SATELLITES = 4
GPS_MAX_HDOP = 5.0
GPS_MAX_SPEED_MPS = 10.0
GPS_STABLE_FIX_COUNT = 3
GPS_FIX_LOSS_TIMEOUT = 8.0
GPS_HEADING_MIN_DIST = 1.2
GPS_HEADING_BASELINE_MIN_DIST = 1.5
GPS_HEADING_WINDOW_SEC = 8.0
GPS_HEADING_HOLD_SEC = 2.5
HEADING_OFFSET_LEARN_MIN_SPEED_MPS = 0.5
HEADING_OFFSET_LEARN_BOOTSTRAP_SAMPLES = 6
HEADING_OFFSET_LEARN_BOOTSTRAP_MAX_RESIDUAL_DEG = 30.0
HEADING_OFFSET_LEARN_MAX_RESIDUAL_DEG = 180.0
HEADING_OFFSET_LEARN_MAX_ABS_OFFSET_DEG = 180.0
GPS_COORD_LAT_OFFSET_DEG = 0.0
GPS_COORD_LNG_OFFSET_DEG = 0.0
GPS_PROBE_SECONDS = 2.0
GPS_STARTUP_SYNC_SECONDS = 8.0
GPS_STARTUP_READ_SIZE = 256
GPS_RECONNECT_SLEEP = 1.0
GPS_NO_DATA_REOPEN_TIMEOUT = 12.0
GPS_NON_GGA_REOPEN_TIMEOUT = 20.0
GPS_ACTIVE_DETECT = 1
GPS_INACTIVE_DETECT = 0
GPS_DIAGNOSTIC_LOG_INTERVAL = 5.0

# BNO055関連定数
BNO_SETUP_RETRY_COUNT = 3
BNO_SETUP_RETRY_INTERVAL = 0.5
BNO_INIT_READY_TIMEOUT = 2.5
BNO_INIT_SAMPLE_INTERVAL = 0.1
BNO_FAIL_LIMIT = 10
BNO_REINIT_COOLDOWN = 3.0
BNO_ACC_MAX = 200.0
BNO_GYRO_MAX = 2000.0
BNO_MAG_MAX = 2000.0
BNO_ANGLE_JUMP_MAX = 60.0
BNO_HEADING_RECOVERY_STALE_SEC = 0.8
BNO_HEADING_RECOVERY_SAMPLES = 3
BNO_HEADING_RECOVERY_MAX_STEP_DEG = 35.0
BNO_CALIB_MAG_MIN = 2
BNO_STALE_TIMEOUT = 2.0
BNO_FREEZE_EPS = 0.001
BNO_FUSION_OK_STATES = (5, 6)
PHASE0_SENSOR_STALE_TIMEOUT = 1.5

# BMP180関連定数
BMP_FAIL_LIMIT = 3
BMP_REINIT_COOLDOWN = 3.0
BMP_SAMPLING_RATE = 0.1
BMP_SEA_LEVEL_PRESSURE_PA = 101325.0
BMP_PRESSURE_MIN_VALID = 30000.0
BMP_PRESSURE_MAX_VALID = 120000.0
BMP_ALTITUDE_MIN_VALID = -500.0
BMP_ALTITUDE_MAX_VALID = 10000.0

# PHASE2関連定数
# パラシュート離脱、磁気校正、GPS基準のBNO方位整列を順番に行う。
PHASE2_ESCAPE_TIME = 6.0
PHASE2_CALIB_MIN_TIME = 6.0
PHASE2_CALIB_MAX_TIME = 15.0
PHASE2_CALIB_STABLE_SEC = 1.5
# 芝生で低速輪がストールしないよう、Phase3 の大旋回と同じ最低45%を保つ。
# 旋回差を残すため外輪も70%へ引き上げる。
PHASE2_CALIB_ARC_OUTER_SPEED = 70
PHASE2_CALIB_ARC_INNER_SPEED = 45
PHASE2_CALIB_ARC_INTERVAL_SEC = 2.5
PHASE2_OFFSET_LEG_MAX_TIME = 25.0
PHASE2_OFFSET_CONTROL_SETTLE_TIME = 2.0
PHASE2_OFFSET_MIN_DISTANCE_M = 3.0
PHASE2_OFFSET_MAX_DISTANCE_M = 6.5
PHASE2_OFFSET_PROGRESS_WINDOW_SEC = 5.0
PHASE2_OFFSET_MIN_PROGRESS_M = 0.5
PHASE2_OFFSET_NEAR_GOAL_DISTANCE_M = 4.0
PHASE2_OFFSET_NEAR_GOAL_GRACE_SEC = 10.0
PHASE2_OFFSET_NEAR_GOAL_MIN_PROGRESS_M = 0.1
PHASE2_OFFSET_MIN_SAMPLES = 6
PHASE2_OFFSET_MIN_PATH_EFFICIENCY = 0.50
PHASE2_OFFSET_MAX_BNO_SPREAD_DEG = 20.0
PHASE2_OFFSET_MAX_SUBSEGMENT_DIFF_DEG = 25.0
PHASE2_TURN_INTERVAL = 3.0
PHASE2_SPEED = 100
PHASE2_OFFSET_SPEED = 100
PHASE2_OFFSET_CORRECTION_BASE_SPEED = 85
PHASE2_OFFSET_HOLD_KP = 0.5
PHASE2_OFFSET_HOLD_DEADBAND_DEG = 2.5
PHASE2_OFFSET_MAX_SAMPLE_HEADING_ERROR_DEG = 20.0
PHASE2_OFFSET_HOLD_MAX_DELTA = 15.0
PHASE2_FORWARD_REORIENT_OUTER_SPEED = 70
PHASE2_FORWARD_REORIENT_INNER_SPEED = 45
PHASE2_FORWARD_REORIENT_TIME = 2.5
PHASE2_OFFSET_MAX_RETRIES = 3
PHASE2_BNO_WAIT_TIMEOUT = 3.0
PHASE2_ENTRY_READY_CONFIRM_COUNT = 10
PHASE2_ENTRY_RECOVERY_QUIET_SEC = 2.0
PHASE2_OFFSET_MODE_COLLECT = "collect"
PHASE2_OFFSET_MODE_REORIENT = "reorient"
PHASE2_OFFSET_MODE_BNO_WAIT = "bno_wait"
PHASE2_OFFSET_MODE_READINESS = "readiness"
# Legacy alias retained for old log readers. No counter-rotation remains.
PHASE2_OFFSET_MODE_TURNAROUND = PHASE2_OFFSET_MODE_REORIENT
PHASE2_RAMP_TIME = 0.2
PHASE2_STAGE_ESCAPE = "escape"
PHASE2_STAGE_CALIBRATION = "calibration"
PHASE2_STAGE_OFFSET = "offset"

# その他定数
EARTH_RADIUS_METERS = 6378137.0
MILLISECOND_SCALE = 1000
DEGREE_FULL_CIRCLE = 360.0
DEGREE_HALF_CIRCLE = 180.0
HEADING_SOURCE_INVALID = "INVALID"
HEADING_SOURCE_GPS = "GPS"
HEADING_SOURCE_BNO = "BNO"
HEADING_SOURCE_JOINER = "+"
HEADING_WEIGHT_GPS = 0.65
HEADING_WEIGHT_BNO_BASE = 0.35
HEADING_WEIGHT_BNO_STEP = 0.1
HEADING_WEIGHT_BNO_MIN = 0.25
HEADING_WEIGHT_BNO_MAX = 0.7
HEADING_MAG_CALIB_MAX = 3

# ログとループ関連定数
PHASE_LOG_INTERVAL = 10
GPS_LOST_LOG_INTERVAL = 20
MAIN_LOOP_INTERVAL = 0.1
SHORT_SLEEP = MAIN_LOOP_INTERVAL

# モーター制御のランプアップ・ダウン関連定数
RAMP_HALF_DIVISOR = 2.0
PWM_PERCENT_MIN = 0.0
PWM_PERCENT_MAX = 100.0
PWM_DUTY_MIN = 0.0
PWM_DUTY_MAX = 1.0
TURN_GAIN_SCALE_MIN = 0.5
TURN_GAIN_SCALE_MAX = 1.0

# デフォルト値
DEFAULT_VECTOR3 = (0.0, 0.0, 0.0)
DEFAULT_BNO_CALIB = {"valid": False, "value": (0, 0, 0, 0)}
DEFAULT_OBSTACLE_DIST_CM = 999.0
DEFAULT_FLOAT_VALUE = 0.0
DEFAULT_PHASE = int(Phase.PHASE0)

# NMEA関連定数
NMEA_GGA_PREFIXES = ("$GPGGA", "$GNGGA")
NMEA_SENTENCE_GGA = "GGA"

# デバイスキー
DEVICE_BNO = "bno"
DEVICE_BMP = "bmp"
DEVICE_DETECTOR = "detector"
DEVICE_LED_RED = "led_red"
DEVICE_LED_GREEN = "led_green"
DEVICE_MOTOR_1_PWM = "motor_1_pwm"
DEVICE_MOTOR_1_DIR = "motor_1_dir"
DEVICE_MOTOR_2_PWM = "motor_2_pwm"
DEVICE_MOTOR_2_DIR = "motor_2_dir"
DEVICE_SONAR = "sonar"
DEVICE_KEYS = (
    DEVICE_BNO,
    DEVICE_BMP,
    DEVICE_DETECTOR,
    DEVICE_LED_RED,
    DEVICE_LED_GREEN,
    DEVICE_MOTOR_1_PWM,
    DEVICE_MOTOR_1_DIR,
    DEVICE_MOTOR_2_PWM,
    DEVICE_MOTOR_2_DIR,
    DEVICE_SONAR,
)

# フェーズごとの動作制御関連定数
PHASES_STOP_MOTORS = (Phase.PHASE0, Phase.PHASE7)
PHASES_SKIP_OBSTACLE = (Phase.PHASE0, Phase.PHASE1, Phase.PHASE5, Phase.PHASE6, Phase.PHASE7)
PHASES_CAMERA_ACTIVE = (Phase.PHASE4, Phase.PHASE5)

# ログのヘッダー
LOG_HEADER = [
    "LogSchemaVersion",
    "RunId",
    "ElapsedSec",
    "Phase",
    "Phase0ExitReason",
    "Phase0ExitDetail",
    "AccX",
    "AccY",
    "AccZ",
    "GyroX",
    "GyroY",
    "GyroZ",
    "MagX",
    "MagY",
    "MagZ",
    "LAT",
    "LNG",
    "GpsSpeedMps",
    "GPSFixQual",
    "GPSSats",
    "GPSHdop",
    "GpsHeading",
    "GpsHeadingValid",
    "GPSFixSeq",
    "NavHeading",
    "NavHeadingSource",
    "HeadingDiff",
    "HeadingTrust",
    "BNOTrusted",
    "BNOOffsetDeg",
    "BNOOffsetValid",
    "BNOOffsetCandidateDeg",
    "BNOOffsetCandidateCount",
    "GPSHeadingBaselineM",
    "Phase2Stage",
    "BNOCalibSys",
    "BNOCalibGyro",
    "BNOCalibAcc",
    "BNOCalibMag",
    "Phase2OffsetRefDeg",
    "Phase2OffsetHeadingErrorDeg",
    "Phase2OffsetDistanceM",
    "Phase2OffsetPathEfficiency",
    "Phase2OffsetCourseDeg",
    "Phase2OffsetBNOMeanDeg",
    "Phase2OffsetBNOSpreadDeg",
    "Phase2OffsetSubsegmentDiffDeg",
    "Phase2OffsetAttemptCount",
    "Phase2OffsetMode",
    "Phase2OffsetTurnTargetDeg",
    "Phase2OffsetStageRetryCount",
    "Phase2OffsetNearGoalActive",
    "Phase2OffsetLegTimeLimitSec",
    "Phase2OffsetRejectReason",
    "Phase1OffsetCandidateValid",
    "Phase1OffsetCandidateDeg",
    "Phase1OffsetDistanceM",
    "Phase1OffsetPathEfficiency",
    "Phase1OffsetBNOSpreadDeg",
    "Phase1OffsetSubsegmentDiffDeg",
    "Phase1OffsetRejectReason",
    "ArrivalInside",
    "ArrivalConfirmCount",
    "Phase3ArrivedLatched",
    "ALT",
    "Pres",
    "Distance",
    "Azimuth",
    "TargetLat",
    "TargetLng",
    "Angle",
    "Direction",
    "Fall",
    *CONE_DIAGNOSTIC_LOG_COLUMNS,
    "ConeStaleSec",
    "ConeUpdatedElapsedSec",
    "ConePhaseDecision",
    "ConePhaseThreshold",
    "ConePhaseReachedProbabilityThreshold",
    "ConePhaseCenterTolerance",
    "ConePhaseDirectionTolerance",
    "ConePhaseRequiredConfirmFrames",
    "ConePhaseDetected",
    "ConePhaseReachedEffective",
    "ConePhaseCentered",
    "ConePhaseDirectionConsistent",
    "ConePhaseConfirmCount",
    "Phase4ConeConfirmCount",
    "Phase4ConeConfirmMarker",
    "Phase5ConeLostCount",
    "Phase5ReachConfirmCount",
    "Phase5EntryReason",
    "CameraFailCount",
    "CameraReinitAttemptCount",
    "CameraRecoveryElapsedSec",
    "CameraRecoveryExhausted",
    "ObstacleDist",
    "SonarValid",
    "SonarStaleSec",
    "AngleValid",
    "BNOStaleSec",
    "BNORecoveryActive",
    "BNORecoveryCount",
    "BNORecoverySeq",
    "BNOAccValid",
    "BNOAccStaleSec",
    "BNOAccUpdatedElapsedSec",
    "BMPValid",
    "BMPStaleSec",
    "BMPUpdatedElapsedSec",
    "MotorCmdType",
    "MotorCmdUpdatedElapsedSec",
    "Motor1CmdSpeed",
    "Motor1CmdForward",
    "Motor2CmdSpeed",
    "Motor2CmdForward",
    "Phase7ArrivalReason",
    "MissionEndReason",
    "MissionTotalTimeout",
    "MissionElapsedSec",
    "RadioDisabled",
    "RadioControlMode",
    "RadioLastEvent",
    "RadioConfigSource",
    "RadioRestoreDeadlineElapsedSec",
]
