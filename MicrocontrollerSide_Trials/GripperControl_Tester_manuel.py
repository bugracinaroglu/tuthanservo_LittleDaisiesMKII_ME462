from machine import Pin, PWM
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

        duty_ns = self._angle_to_duty_ns(angle)
        self.pwm.duty_ns(duty_ns)

    def go_to_neutral(self):
        self.go_to_angle(self.neutral_angle)

    def move_by(self, delta):
        self.go_to_angle(self.current_angle + delta)

    def deinit(self):
        self.pwm.deinit()


# =====================================================
# Gripper Controller
# =====================================================

class GripperController:
    def __init__(self, left_servo, right_servo,
                 neutral_angle=135,
                 left_negative_limit=75,
                 left_positive_limit=20,
                 right_negative_limit=20,
                 right_positive_limit=75,
                 step_deg=1,
                 move_interval_ms=20):

        self.left_servo = left_servo
        self.right_servo = right_servo

        self.neutral_angle = neutral_angle

        self.left_min = neutral_angle - left_negative_limit
        self.left_max = neutral_angle + left_positive_limit

        self.right_min = neutral_angle - right_negative_limit
        self.right_max = neutral_angle + right_positive_limit

        self.step_deg = step_deg
        self.move_interval_ms = move_interval_ms
        self.last_move_time = ticks_ms()

        self.mode = "stop"

        # Başlangıçta gripper açık pozisyona gitsin
        self.open_immediate()

    def open_immediate(self):
        self.left_servo.go_to_angle(self.left_min)
        self.right_servo.go_to_angle(self.right_max)
        self.mode = "stop"

    def close_immediate(self):
        self.left_servo.go_to_angle(self.left_max)
        self.right_servo.go_to_angle(self.right_min)
        self.mode = "stop"

    def start_opening(self):
        self.mode = "open"
        print("Opening gripper...")

    def start_closing(self):
        self.mode = "close"
        print("Closing gripper...")

    def start_neutral(self):
        self.mode = "neutral"
        print("Moving to neutral...")

    def stop(self):
        self.mode = "stop"
        print("Stopped. Holding current position.")

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

    def update(self):
        now = ticks_ms()

        if ticks_diff(now, self.last_move_time) < self.move_interval_ms:
            return

        self.last_move_time = now

        if self.mode == "open":
            left_done = self._move_servo_towards(self.left_servo, self.left_min)
            right_done = self._move_servo_towards(self.right_servo, self.right_max)

            if left_done and right_done:
                self.mode = "stop"
                print("Gripper opened.")

        elif self.mode == "close":
            left_done = self._move_servo_towards(self.left_servo, self.left_max)
            right_done = self._move_servo_towards(self.right_servo, self.right_min)

            if left_done and right_done:
                self.mode = "stop"
                print("Gripper closed.")

        elif self.mode == "neutral":
            left_done = self._move_servo_towards(self.left_servo, self.neutral_angle)
            right_done = self._move_servo_towards(self.right_servo, self.neutral_angle)

            if left_done and right_done:
                self.mode = "stop"
                print("Moved to neutral.")

        # stop modunda hiçbir şey yapmıyoruz.
        # PWM aktif kaldığı için servolar mevcut pozisyonlarını tutar.

    def deinit(self):
        self.left_servo.deinit()
        self.right_servo.deinit()


# =====================================================
# Helper Function
# =====================================================

def read_command():
    if select.select([sys.stdin], [], [], 0)[0]:
        cmd = sys.stdin.readline().strip().lower()
        return cmd
    return None


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

gripper = GripperController(
    left_servo=left_servo,
    right_servo=right_servo,
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

def print_menu():
    print("-----------------------")
    print("Gripper control started.")
    print("o + Enter : open")
    print("c + Enter : close")
    print("s + Enter : stop / hold")
    print("n + Enter : neutral")
    print("q + Enter : quit")
    print("-----------------------")
    print("Enter : ", end="")


print_menu()

try:
    while True:
        cmd = read_command()

        if cmd is not None:
            if cmd == "o":
                gripper.start_opening()

            elif cmd == "c":
                gripper.start_closing()

            elif cmd == "s":
                gripper.stop()

            elif cmd == "n":
                gripper.start_neutral()

            elif cmd == "q":
                print("\nExiting...")
                break

            else:
                print("Invalid command.")

            print_menu()

        gripper.update()

finally:
    gripper.deinit()
    print("PWM signals disabled.")