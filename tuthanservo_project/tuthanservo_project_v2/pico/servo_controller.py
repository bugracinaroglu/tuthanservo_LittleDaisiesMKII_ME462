from machine import Pin, PWM


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
        """Move directly to an absolute servo angle."""
        angle = self._clamp_angle(angle)
        self.current_angle = angle
        self.pwm.duty_ns(self._angle_to_duty_ns(angle))

    def move_by(self, delta):
        self.go_to_angle(self.current_angle + delta)

    def deinit(self):
        self.pwm.deinit()
