from machine import Pin, ADC
from time import ticks_ms, ticks_diff


class FSR:
    def __init__(
        self,
        pin,
        threshold_voltage=2.0,
        window_size=5,
        sample_interval_ms=5,
        vref=3.3,
        active_high=True
    ):
        self.pin = pin
        self.threshold_voltage = threshold_voltage
        self.window_size = window_size
        self.sample_interval_ms = sample_interval_ms
        self.vref = vref
        self.active_high = active_high

        self.threshold_raw = int(
            self.threshold_voltage * 65535 / self.vref
        )

        self.adc = ADC(Pin(pin))

        self.raw_values = []
        self.last_raw = 0
        self.average_raw = 0.0
        self.last_voltage = 0.0
        self.average_voltage = 0.0
        self.pressed_state = False

        self.last_sample_time = ticks_ms()

    def reset_filter(self):
        """
        Clear previous moving-average/contact data.

        This prevents an old pressed value from being reused by a new
        close command.
        """
        self.raw_values = []
        self.last_raw = 0
        self.average_raw = 0.0
        self.last_voltage = 0.0
        self.average_voltage = 0.0
        self.pressed_state = False
        self.last_sample_time = ticks_ms()

    def read_raw_data(self):
        self.last_raw = self.adc.read_u16()
        return self.last_raw

    def _update_window(self, raw):
        self.raw_values.append(raw)

        if len(self.raw_values) > self.window_size:
            self.raw_values.pop(0)

    def _calculate_average_raw(self):
        if not self.raw_values:
            return 0.0

        return sum(self.raw_values) / len(self.raw_values)

    def update(self):
        now = ticks_ms()

        if ticks_diff(
            now,
            self.last_sample_time
        ) < self.sample_interval_ms:
            return self.pressed_state

        self.last_sample_time = now

        raw = self.read_raw_data()
        self._update_window(raw)

        self.last_voltage = (
            self.last_raw * self.vref / 65535
        )

        self.average_raw = self._calculate_average_raw()

        self.average_voltage = (
            self.average_raw * self.vref / 65535
        )

        # Do not make a pressed decision before the filter is full.
        if len(self.raw_values) < self.window_size:
            self.pressed_state = False
            return self.pressed_state

        if self.active_high:
            self.pressed_state = (
                self.average_voltage >= self.threshold_voltage
            )
        else:
            self.pressed_state = (
                self.average_voltage <= self.threshold_voltage
            )

        return self.pressed_state

    def is_pressed(self):
        return self.pressed_state

    def get_status(self):
        return {
            "raw": self.last_raw,
            "average_raw": round(self.average_raw, 1),
            "voltage": round(self.last_voltage, 3),
            "average_voltage": round(
                self.average_voltage,
                3
            ),
            "threshold_voltage": self.threshold_voltage,
            "pressed": self.pressed_state
        }
