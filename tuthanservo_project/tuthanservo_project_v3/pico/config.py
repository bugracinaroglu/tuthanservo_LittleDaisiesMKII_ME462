# =====================================================
# Hardware
# =====================================================

LEFT_SERVO_PIN = 16
RIGHT_SERVO_PIN = 17

LEFT_FSR_PIN = 26
RIGHT_FSR_PIN = 27


# =====================================================
# Servo / gripper geometry
# All angles are absolute servo angles.
# =====================================================

LEFT_MEAN_ANGLE = 85
RIGHT_MEAN_ANGLE = 100

OPEN_OFFSET = 30
CLOSE_OFFSET = 10

SERVO_FREQUENCY = 50
SERVO_MIN_US = 500
SERVO_MAX_US = 2500
SERVO_MAX_ANGLE = 180

STEP_DEG = 1
MOVE_INTERVAL_MS = 20

# When both fingers have stopped at contact/angle limits, keep sampling
# briefly before declaring partial_contact or close_failed.
CLOSE_SETTLE_MS = 250

# "open", "neutral", or "close".
# "close" is an unverified immediate startup position, so "open" is safer.
INITIAL_POSITION = "open"


# =====================================================
# FSR
# =====================================================

LEFT_FSR_THRESHOLD_VOLTAGE = 2.0
RIGHT_FSR_THRESHOLD_VOLTAGE = 2.0

FSR_VREF = 3.3
FSR_WINDOW_SIZE = 5
FSR_SAMPLE_INTERVAL_MS = 5
FSR_ACTIVE_HIGH = True


# =====================================================
# Wi-Fi / MQTT timing
# =====================================================

WIFI_RETRY_INTERVAL_MS = 5000
WIFI_CONNECT_TIMEOUT_MS = 15000

MQTT_BASE_TOPIC = "robot/gripper"
MQTT_KEEPALIVE_SECONDS = 30
MQTT_RECONNECT_INTERVAL_MS = 3000
MQTT_PING_INTERVAL_MS = 15000

TELEMETRY_INTERVAL_MS = 500
MAIN_LOOP_SLEEP_MS = 2
