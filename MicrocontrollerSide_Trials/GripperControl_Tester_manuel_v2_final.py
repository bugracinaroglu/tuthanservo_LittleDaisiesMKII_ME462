from machine import Pin, PWM
from time import ticks_ms, ticks_diff, sleep_ms


# =====================================================
# Servo Controller
# =====================================================

class ServoController:
    def __init__(
        self,
        pin,
        initial_angle,
        freq=50,
        min_us=500,
        max_us=2500,
        max_angle=180
    ):
        self.pin = pin
        self.freq = freq
        self.min_us = min_us
        self.max_us = max_us
        self.max_angle = max_angle

        self.pwm = PWM(Pin(pin))
        self.pwm.freq(freq)

        self.current_angle = self._clamp_angle(initial_angle)
        self.go_to_angle(self.current_angle)

    def _clamp_angle(self, angle):
        return max(0, min(float(angle), self.max_angle))

    def _angle_to_duty_ns(self, angle):
        angle = self._clamp_angle(angle)

        pulse_us = (
            self.min_us
            + (angle / self.max_angle)
            * (self.max_us - self.min_us)
        )

        return int(pulse_us * 1000)

    def go_to_angle(self, angle):
        """
        Servoyu verilen mutlak açıya gönderir.
        """

        angle = self._clamp_angle(angle)

        self.current_angle = angle
        self.pwm.duty_ns(self._angle_to_duty_ns(angle))

    def move_by(self, delta):
        self.go_to_angle(self.current_angle + delta)

    def deinit(self):
        self.pwm.deinit()


# =====================================================
# Gripper Controller
# =====================================================

class GripperController:
    def __init__(
        self,
        left_servo,
        right_servo,
        left_mean_angle,
        right_mean_angle,
        open_offset,
        close_offset,
        initial_position="open",
        step_deg=1,
        move_interval_ms=20,
        open_wait_ms=1000,
        close_wait_ms=1000
    ):
        self.left_servo = left_servo
        self.right_servo = right_servo

        self.left_mean_angle = left_mean_angle
        self.right_mean_angle = right_mean_angle

        self.open_offset = open_offset
        self.close_offset = close_offset

        # Mutlak açık konumlar
        self.left_open_angle = (
            self.left_mean_angle - self.open_offset
        )

        self.right_open_angle = (
            self.right_mean_angle + self.open_offset
        )

        # Mutlak kapalı konumlar
        self.left_close_angle = (
            self.left_mean_angle + self.close_offset
        )

        self.right_close_angle = (
            self.right_mean_angle - self.close_offset
        )

        self.step_deg = step_deg
        self.move_interval_ms = move_interval_ms

        self.open_wait_ms = open_wait_ms
        self.close_wait_ms = close_wait_ms

        self.last_move_time = ticks_ms()
        self.wait_start_time = ticks_ms()

        self.mode = "stop"

        self._check_angles()
        self.go_to_initial_position(initial_position)

    def _check_angles(self):
        angles = {
            "left_open_angle": self.left_open_angle,
            "right_open_angle": self.right_open_angle,
            "left_close_angle": self.left_close_angle,
            "right_close_angle": self.right_close_angle
        }

        for name, angle in angles.items():
            if angle < 0 or angle > 180:
                raise ValueError(
                    name + " servo aralığı dışında: " + str(angle)
                )

    def go_to_initial_position(self, initial_position):
        """
        initial_position seçenekleri:
        "open"
        "close"
        "mean"
        """

        initial_position = initial_position.lower()

        if initial_position == "open":
            self.open_immediate()

            # Bir süre açık bekledikten sonra kapanacak.
            self.mode = "wait_open"
            self.wait_start_time = ticks_ms()

        elif initial_position == "close":
            self.close_immediate()

            # Bir süre kapalı bekledikten sonra açılacak.
            self.mode = "wait_close"
            self.wait_start_time = ticks_ms()

        elif initial_position == "mean":
            self.mean_immediate()

            # Mean pozisyondan ilk olarak açılmaya başlar.
            self.mode = "open"

        else:
            raise ValueError(
                "initial_position open, close veya mean olmalıdır."
            )

    def open_immediate(self):
        self.left_servo.go_to_angle(self.left_open_angle)
        self.right_servo.go_to_angle(self.right_open_angle)

        print(
            "Initial open position | Left:",
            self.left_open_angle,
            "| Right:",
            self.right_open_angle
        )

    def close_immediate(self):
        self.left_servo.go_to_angle(self.left_close_angle)
        self.right_servo.go_to_angle(self.right_close_angle)

        print(
            "Initial closed position | Left:",
            self.left_close_angle,
            "| Right:",
            self.right_close_angle
        )

    def mean_immediate(self):
        self.left_servo.go_to_angle(self.left_mean_angle)
        self.right_servo.go_to_angle(self.right_mean_angle)

        print(
            "Initial mean position | Left:",
            self.left_mean_angle,
            "| Right:",
            self.right_mean_angle
        )

    def start_opening(self):
        self.mode = "open"
        print("Opening gripper...")

    def start_closing(self):
        self.mode = "close"
        print("Closing gripper...")

    def stop(self):
        self.mode = "stop"
        print("Gripper stopped. Servos holding position.")

    def _move_servo_towards(self, servo, target_angle):
        current_angle = servo.current_angle
        difference = target_angle - current_angle

        if abs(difference) <= self.step_deg:
            servo.go_to_angle(target_angle)
            return True

        if difference > 0:
            servo.move_by(self.step_deg)
        else:
            servo.move_by(-self.step_deg)

        return False

    def update(self):
        now = ticks_ms()

        # Açık konumda bekleme
        if self.mode == "wait_open":
            if ticks_diff(now, self.wait_start_time) >= self.open_wait_ms:
                self.start_closing()

            return

        # Kapalı konumda bekleme
        if self.mode == "wait_close":
            if ticks_diff(now, self.wait_start_time) >= self.close_wait_ms:
                self.start_opening()

            return

        # Servo hareket zamanlaması
        if ticks_diff(now, self.last_move_time) < self.move_interval_ms:
            return

        self.last_move_time = now

        if self.mode == "open":
            left_done = self._move_servo_towards(
                self.left_servo,
                self.left_open_angle
            )

            right_done = self._move_servo_towards(
                self.right_servo,
                self.right_open_angle
            )

            if left_done and right_done:
                print(
                    "Gripper opened | Left:",
                    self.left_open_angle,
                    "| Right:",
                    self.right_open_angle
                )

                self.mode = "wait_open"
                self.wait_start_time = now

        elif self.mode == "close":
            left_done = self._move_servo_towards(
                self.left_servo,
                self.left_close_angle
            )

            right_done = self._move_servo_towards(
                self.right_servo,
                self.right_close_angle
            )

            if left_done and right_done:
                print(
                    "Gripper closed | Left:",
                    self.left_close_angle,
                    "| Right:",
                    self.right_close_angle
                )

                self.mode = "wait_close"
                self.wait_start_time = now

    def deinit(self):
        self.left_servo.deinit()
        self.right_servo.deinit()


# =====================================================
# Gripper Settings
# =====================================================

LEFT_SERVO_PIN = 16
RIGHT_SERVO_PIN = 17

LEFT_MEAN_ANGLE = 85
RIGHT_MEAN_ANGLE = 100

OPEN_OFFSET = 30
CLOSE_OFFSET = 6

# "open", "close" veya "mean"
INITIAL_POSITION = "open"

STEP_DEG = 1
MOVE_INTERVAL_MS = 20

OPEN_WAIT_MS = 1000
CLOSE_WAIT_MS = 1000


# =====================================================
# Object Definitions
# =====================================================

left_servo = ServoController(
    pin=LEFT_SERVO_PIN,
    initial_angle=LEFT_MEAN_ANGLE,
    max_angle=180
)

right_servo = ServoController(
    pin=RIGHT_SERVO_PIN,
    initial_angle=RIGHT_MEAN_ANGLE,
    max_angle=180
)

gripper = GripperController(
    left_servo=left_servo,
    right_servo=right_servo,

    left_mean_angle=LEFT_MEAN_ANGLE,
    right_mean_angle=RIGHT_MEAN_ANGLE,

    open_offset=OPEN_OFFSET,
    close_offset=CLOSE_OFFSET,

    initial_position=INITIAL_POSITION,

    step_deg=STEP_DEG,
    move_interval_ms=MOVE_INTERVAL_MS,

    open_wait_ms=OPEN_WAIT_MS,
    close_wait_ms=CLOSE_WAIT_MS
)


# =====================================================
# Main
# =====================================================

print("Gripper continuous open-close test started.")
print("Stop için Ctrl+C kullanabilirsiniz.")

try:
    while True:
        gripper.update()

        # İşlemciyi gereksiz yere tamamen meşgul etmemesi için
        sleep_ms(1)

except KeyboardInterrupt:
    print("Gripper test stopped.")

finally:
    gripper.deinit()
    print("PWM signals disabled.")