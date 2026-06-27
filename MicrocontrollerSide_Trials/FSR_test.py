from machine import Pin, ADC
from time import ticks_ms, ticks_diff

# GP28 = ADC2
fsr = ADC(Pin(26))

PRINT_INTERVAL_MS = 200
last_print_time = ticks_ms()

while True:
    now = ticks_ms()

    if ticks_diff(now, last_print_time) >= PRINT_INTERVAL_MS:
        last_print_time = now

        raw = fsr.read_u16()          # 0 - 65535
        voltage = raw * 3.3 / 65535   # ADC voltage

        print("FSR raw:", raw, "Voltage:", round(voltage, 3), "V")