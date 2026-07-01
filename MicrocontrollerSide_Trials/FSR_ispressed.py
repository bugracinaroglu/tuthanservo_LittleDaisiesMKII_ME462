from machine import Pin, ADC
from time import ticks_ms, ticks_diff


class FSR:
    def __init__(self, pin, threshold, window_size=10, vref=3.3, active_high=True):
        self.adc = ADC(Pin(pin))
        self.threshold = threshold
        self.window_size = window_size
        self.vref = vref
        self.active_high = active_high

        self.raw_values = []
        self.last_raw = 0
        self.last_voltage = 0.0
        self.pressed_state = False

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

    def _average_raw(self):
        if len(self.raw_values) == 0:
            return 0

        return sum(self.raw_values) / len(self.raw_values)

    def is_pressed(self):
        raw = self.read_raw_data()
        self._update_window(raw)

        # İlk 10 değer dolmadan kesin karar verme
        if len(self.raw_values) < self.window_size:
            return False

        avg_raw = self._average_raw()

        if self.active_high:
            self.pressed_state = avg_raw >= self.threshold
        else:
            self.pressed_state = avg_raw <= self.threshold

        return self.pressed_state

    def get_average_raw(self):
        return self._average_raw()


# =====================================================
# Object Definition
# =====================================================

fsr = FSR(
    pin=27,
    threshold=15000,
    window_size=10,
    active_high=True
)


# =====================================================
# Main
# =====================================================

PRINT_INTERVAL_MS = 100
last_print_time = ticks_ms()

while True:
    now = ticks_ms()

    if ticks_diff(now, last_print_time) >= PRINT_INTERVAL_MS:
        last_print_time = now

        raw = fsr.read_raw_data()
        voltage = raw * 3.3 / 65535
        pressed = fsr.is_pressed()
        avg_raw = fsr.get_average_raw()

        print(
            "Raw:", raw,
            "| Avg:", round(avg_raw, 1),
            "| Voltage:", round(voltage, 3),
            "V | Pressed:", pressed
        )