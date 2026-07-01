from machine import Pin, PWM
from time import sleep


LEFT_SERVO_PIN = 16
RIGHT_SERVO_PIN = 17

LEFT_MEAN_ANGLE = 85
RIGHT_MEAN_ANGLE = 100

LEFT_INITIAL_ANGLE = 85
RIGHT_INITIAL_ANGLE = 100

# Gripper hareket miktarları
OPEN_OFFSET = 30
CLOSE_OFFSET = 6

SERVO_FREQUENCY = 50
SERVO_MIN_US = 500
SERVO_MAX_US = 2500
SERVO_MAX_ANGLE = 180

MOVE_STEP_DEG = 1
MOVE_STEP_DELAY = 0.03

OPEN_WAIT_TIME = 1.0
CLOSE_WAIT_TIME = 1.0


class ServoController:
    def __init__(
        self,
        pin,
        initial_angle,
        max_angle=180,
        frequency=50,
        min_us=500,
        max_us=2500
    ):
        self.max_angle = max_angle
        self.min_us = min_us
        self.max_us = max_us

        self.pwm = PWM(Pin(pin))
        self.pwm.freq(frequency)

        self.current_angle = initial_angle
        self.set_angle(initial_angle)

    def angle_to_duty_ns(self, angle):
        angle = max(0, min(float(angle), self.max_angle))

        pulse_us = (
            self.min_us
            + (angle / self.max_angle)
            * (self.max_us - self.min_us)
        )

        return int(pulse_us * 1000)

    def set_angle(self, angle):
        angle = max(0, min(float(angle), self.max_angle))

        self.current_angle = angle
        self.pwm.duty_ns(self.angle_to_duty_ns(angle))

    def deinit(self):
        self.pwm.deinit()


class GripperController:
    def __init__(
        self,
        left_pin,
        right_pin,
        left_mean_angle,
        right_mean_angle,
        left_initial_angle,
        right_initial_angle,
        open_offset,
        close_offset
    ):
        self.left_mean_angle = left_mean_angle
        self.right_mean_angle = right_mean_angle

        self.open_offset = open_offset
        self.close_offset = close_offset

        self.left_servo = ServoController(
            pin=left_pin,
            initial_angle=left_initial_angle,
            max_angle=SERVO_MAX_ANGLE,
            frequency=SERVO_FREQUENCY,
            min_us=SERVO_MIN_US,
            max_us=SERVO_MAX_US
        )

        self.right_servo = ServoController(
            pin=right_pin,
            initial_angle=right_initial_angle,
            max_angle=SERVO_MAX_ANGLE,
            frequency=SERVO_FREQUENCY,
            min_us=SERVO_MIN_US,
            max_us=SERVO_MAX_US
        )

    def move_to(self, left_target_angle, right_target_angle):
        left_start = self.left_servo.current_angle
        right_start = self.right_servo.current_angle

        left_difference = left_target_angle - left_start
        right_difference = right_target_angle - right_start

        largest_difference = max(
            abs(left_difference),
            abs(right_difference)
        )

        number_of_steps = max(
            1,
            int(largest_difference / MOVE_STEP_DEG)
        )

        for step in range(1, number_of_steps + 1):
            ratio = step / number_of_steps

            left_angle = left_start + left_difference * ratio
            right_angle = right_start + right_difference * ratio

            self.left_servo.set_angle(left_angle)
            self.right_servo.set_angle(right_angle)

            sleep(MOVE_STEP_DELAY)

        self.left_servo.set_angle(left_target_angle)
        self.right_servo.set_angle(right_target_angle)

        print(
            "Left:",
            round(left_target_angle, 1),
            "| Right:",
            round(right_target_angle, 1)
        )

    def open(self):
        # Sol: mean - open_offset
        # Sağ: mean + open_offset

        left_target = self.left_mean_angle - self.open_offset
        right_target = self.right_mean_angle + self.open_offset

        print("Gripper opening...")
        self.move_to(left_target, right_target)

    def close(self):
        # Sol: mean + close_offset
        # Sağ: mean - close_offset

        left_target = self.left_mean_angle + self.close_offset
        right_target = self.right_mean_angle - self.close_offset

        print("Gripper closing...")
        self.move_to(left_target, right_target)

    def go_to_mean_position(self):
        self.move_to(
            self.left_mean_angle,
            self.right_mean_angle
        )

    def deinit(self):
        self.left_servo.deinit()
        self.right_servo.deinit()


gripper = GripperController(
    left_pin=LEFT_SERVO_PIN,
    right_pin=RIGHT_SERVO_PIN,

    left_mean_angle=LEFT_MEAN_ANGLE,
    right_mean_angle=RIGHT_MEAN_ANGLE,

    left_initial_angle=LEFT_INITIAL_ANGLE,
    right_initial_angle=RIGHT_INITIAL_ANGLE,

    open_offset=OPEN_OFFSET,
    close_offset=CLOSE_OFFSET
)


try:
    print("Gripper test started.")

    while True:
        gripper.open()
        sleep(OPEN_WAIT_TIME)

        gripper.close()
        sleep(CLOSE_WAIT_TIME)

except KeyboardInterrupt:
    print("Gripper test stopped.")

finally:
    gripper.deinit()