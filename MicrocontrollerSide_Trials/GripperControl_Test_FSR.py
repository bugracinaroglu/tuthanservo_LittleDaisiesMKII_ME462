from machine import Pin, PWM, ADC
from time import ticks_ms, ticks_diff
import sys

try:
    import uselect as select
except ImportError:
    import select


# =====================================================
# Servo Controller
# =====================================================

class ServoController:
    def __init__(self, pin, freq=50, min_us=500, max_us=2500,
                 max_angle=270, neutral_angle=None):

        self.pin = pin
        self.freq = freq
        self.min_us = min_us
        self.max_us = max_us
        self.max_angle = max_angle

        if neutral_angle is None:
            self.neutral_angle = max_angle / 2
        else:
            self.neutral_angle = neutral_angle

        self.current_angle = self.neutral_angle

        self.pwm = PWM(Pin(pin))
        self.pwm.freq(freq)

        self.go_to_angle(self.current_angle)

    def _clamp_angle(self, angle):
        return max(0, min(angle, self.max_angle))

    def _angle_to_duty_ns(self, angle):
        angle = self._clamp_angle(angle)
        pulse_us = self.min_us + (angle / self.max_angle) * (self.max_us - self.min_us)
        return int(pulse_us * 1000)

    def go_to_angle(self, angle):
        angle = self._clamp_angle(angle)
        self.current_angle = angle
        self.pwm.duty_ns(self._angle_to_duty_ns(angle))

    def go_to_neutral(self):
        self.go_to_angle(self.neutral_angle)

    def move_by(self, delta):
        self.go_to_angle(self.current_angle + delta)

    def deinit(self):
        self.pwm.deinit()


# =====================================================
# FSR Controller
# =====================================================

class FSR:
    def __init__(self, pin, threshold, window_size=10,
                 sample_interval_ms=10, vref=3.3, active_high=True):

        self.adc = ADC(Pin(pin))
        self.pin = pin
        self.threshold = threshold
        self.window_size = window_size
        self.sample_interval_ms = sample_interval_ms
        self.vref = vref
        self.active_high = active_high

        self.raw_values = []
        self.last_raw = 0
        self.last_voltage = 0.0
        self.pressed_state = False

        self.last_sample_time = ticks_ms()

    def read_raw_data(self):
        self.last_raw = self.adc.read_u16()
        return self.last_raw

    def read_voltage_data(self):
        raw = self.read_raw_data()
        self.last_voltage = raw * self.vref / 65535
        return self.last_voltage

    def _update_window(self, raw):
        self.raw_values.append(raw)

        if len(self.raw_values) > self.window_size:
            self.raw_values.pop(0)

    def get_average_raw(self):
        if len(self.raw_values) == 0:
            return 0

        return sum(self.raw_values) / len(self.raw_values)

    def update(self):
        now = ticks_ms()

        if ticks_diff(now, self.last_sample_time) < self.sample_interval_ms:
            return self.pressed_state

        self.last_sample_time = now

        raw = self.read_raw_data()
        self._update_window(raw)

        if len(self.raw_values) < self.window_size:
            self.pressed_state = False
            return self.pressed_state

        avg_raw = self.get_average_raw()

        if self.active_high:
            self.pressed_state = avg_raw >= self.threshold
        else:
            self.pressed_state = avg_raw <= self.threshold

        return self.pressed_state

    def is_pressed(self):
        return self.pressed_state


# =====================================================
# Gripper Controller
# =====================================================

class GripperController:
    def __init__(self, left_servo, right_servo,
                 fsr_left=None, fsr_right=None,
                 neutral_angle=135,
                 left_negative_limit=75,
                 left_positive_limit=20,
                 right_negative_limit=20,
                 right_positive_limit=75,
                 step_deg=1,
                 move_interval_ms=20):

        self.left_servo = left_servo
        self.right_servo = right_servo

        self.fsr_left = fsr_left
        self.fsr_right = fsr_right

        self.neutral_angle = neutral_angle

        self.left_min = neutral_angle - left_negative_limit
        self.left_max = neutral_angle + left_positive_limit

        self.right_min = neutral_angle - right_negative_limit
        self.right_max = neutral_angle + right_positive_limit

        self.step_deg = step_deg
        self.move_interval_ms = move_interval_ms
        self.last_move_time = ticks_ms()

        self.mode = "stop"

        self.left_contact = False
        self.right_contact = False

        self.open_immediate()

    def open_immediate(self):
        self.left_servo.go_to_angle(self.left_min)
        self.right_servo.go_to_angle(self.right_max)
        self.mode = "stop"
        self.left_contact = False
        self.right_contact = False

    def close_immediate(self):
        self.left_servo.go_to_angle(self.left_max)
        self.right_servo.go_to_angle(self.right_min)
        self.mode = "stop"

    def start_opening(self):
        self.mode = "open"
        self.left_contact = False
        self.right_contact = False

    def start_release(self):
        self.start_opening()

    def start_closing_with_fsr(self):
        self.mode = "close_fsr"
        self.left_contact = False
        self.right_contact = False

    def start_closing_without_fsr(self):
        self.mode = "close_no_fsr"
        self.left_contact = False
        self.right_contact = False

    def start_neutral(self):
        self.mode = "neutral"
        self.left_contact = False
        self.right_contact = False

    def stop(self):
        self.mode = "stop"

    def _move_servo_towards(self, servo, target_angle):
        current = servo.current_angle

        if abs(target_angle - current) <= self.step_deg:
            servo.go_to_angle(target_angle)
            return True

        if target_angle > current:
            servo.move_by(self.step_deg)
        else:
            servo.move_by(-self.step_deg)

        return False

    def _update_fsr_states(self):
        if self.fsr_left is not None:
            self.left_contact = self.fsr_left.update()

        if self.fsr_right is not None:
            self.right_contact = self.fsr_right.update()

    def update(self):
        self._update_fsr_states()

        now = ticks_ms()

        if ticks_diff(now, self.last_move_time) < self.move_interval_ms:
            return None

        self.last_move_time = now

        if self.mode == "open":
            left_done = self._move_servo_towards(self.left_servo, self.left_min)
            right_done = self._move_servo_towards(self.right_servo, self.right_max)

            if left_done and right_done:
                self.mode = "stop"
                return "open_done"

        elif self.mode == "close_no_fsr":
            left_done = self._move_servo_towards(self.left_servo, self.left_max)
            right_done = self._move_servo_towards(self.right_servo, self.right_min)

            if left_done and right_done:
                self.mode = "stop"
                return "close_no_fsr_done"

        elif self.mode == "close_fsr":
            if self.left_contact:
                left_done = True
            else:
                left_done = self._move_servo_towards(self.left_servo, self.left_max)

            if self.right_contact:
                right_done = True
            else:
                right_done = self._move_servo_towards(self.right_servo, self.right_min)

            if left_done and right_done:
                self.mode = "stop"
                return "close_fsr_done"

        elif self.mode == "neutral":
            left_done = self._move_servo_towards(self.left_servo, self.neutral_angle)
            right_done = self._move_servo_towards(self.right_servo, self.neutral_angle)

            if left_done and right_done:
                self.mode = "stop"
                return "neutral_done"

        return None

    def deinit(self):
        self.left_servo.deinit()
        self.right_servo.deinit()


# =====================================================
# Helper Functions
# =====================================================

def read_command():
    if select.select([sys.stdin], [], [], 0)[0]:
        cmd = sys.stdin.readline().strip().lower()
        return cmd
    return None


def print_menu():
    print("-----------------------")
    print("Gripper control")
    print("c + Enter : close with FSR")
    print("x + Enter : close without FSR")
    print("o + Enter : open without FSR")
    print("r + Enter : birak / release")
    print("n + Enter : neutral")
    print("s + Enter : stop / hold")
    print("q + Enter : quit")
    print("-----------------------")
    print("Enter : ", end="")


# =====================================================
# Object Definitions
# =====================================================

left_servo = ServoController(
    pin=13,
    max_angle=270,
    neutral_angle=135
)

right_servo = ServoController(
    pin=12,
    max_angle=270,
    neutral_angle=135
)

fsr_left = FSR(
    pin=26,
    threshold=12000,
    window_size=10,
    sample_interval_ms=10,
    active_high=True
)

fsr_right = FSR(
    pin=27,
    threshold=12000,
    window_size=10,
    sample_interval_ms=10,
    active_high=True
)

gripper = GripperController(
    left_servo=left_servo,
    right_servo=right_servo,
    fsr_left=fsr_left,
    fsr_right=fsr_right,
    neutral_angle=135,
    left_negative_limit=75,
    left_positive_limit=20,
    right_negative_limit=20,
    right_positive_limit=75,
    step_deg=1,
    move_interval_ms=20
)


# =====================================================
# Main
# =====================================================

print_menu()

try:
    while True:
        cmd = read_command()

        if cmd is not None:
            if cmd == "c":
                gripper.start_closing_with_fsr()

            elif cmd == "x":
                gripper.start_closing_without_fsr()

            elif cmd == "o":
                gripper.start_opening()

            elif cmd == "r":
                gripper.start_release()

            elif cmd == "n":
                gripper.start_neutral()

            elif cmd == "s":
                gripper.stop()
                print_menu()

            elif cmd == "q":
                break

            else:
                print_menu()

        event = gripper.update()

        if event is not None:
            print_menu()

finally:
    gripper.deinit()