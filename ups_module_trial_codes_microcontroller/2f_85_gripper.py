from machine import Pin, PWM, I2C
import time
import sys
import select

# ==========================================
# 1. INA219 Power Monitor Class
# ==========================================
class INA219:
    REG_CONFIG = 0x00
    REG_CALIBRATION = 0x05
    REG_BUS_VOLTAGE = 0x02
    REG_CURRENT = 0x04
    REG_POWER = 0x03

    def __init__(self, i2c_bus, address=0x41):
        self.i2c = i2c_bus
        self.address = address
        self.cal_value = 4096 
        self.current_lsb = 0.1 
        self.power_lsb = 0.002 
        self.configure()

    def _write_register(self, reg, value):
        data = bytearray([(value >> 8) & 0xFF, value & 0xFF])
        self.i2c.writeto_mem(self.address, reg, data)

    def _read_register(self, reg):
        data = self.i2c.readfrom_mem(self.address, reg, 2)
        return int.from_bytes(data, 'big')

    def configure(self):
        self._write_register(self.REG_CALIBRATION, self.cal_value)
        self._write_register(self.REG_CONFIG, 0x399F)

    def get_bus_voltage_V(self):
        val = self._read_register(self.REG_BUS_VOLTAGE)
        return ((val >> 3) * 4) / 1000.0

    def get_current_mA(self):
        self._write_register(self.REG_CALIBRATION, self.cal_value)
        val = self._read_register(self.REG_CURRENT)
        if val > 32767: val -= 65536
        return val * self.current_lsb

    def get_power_mW(self):
        self._write_register(self.REG_CALIBRATION, self.cal_value)
        val = self._read_register(self.REG_POWER)
        return val * (self.power_lsb * 1000)

# ==========================================
# 2. Hardware Setup
# ==========================================
print("Initializing I2C (UPS Monitor)...")
i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)
ina = INA219(i2c, address=0x41)

print("Initializing Servos...")
PWM_FREQ = 50
MIN_DUTY = 1638  # ~500us
MAX_DUTY = 8192  # ~2500us

servo_left = PWM(Pin(2))
servo_left.freq(PWM_FREQ)

servo_right = PWM(Pin(3)) # Assumed GP3 for Right Servo
servo_right.freq(PWM_FREQ)

def angle_to_duty(angle, max_angle=180):
    return int(MIN_DUTY + (MAX_DUTY - MIN_DUTY) * (angle / max_angle))

def update_servos(left_ang, right_ang):
    servo_left.duty_u16(angle_to_duty(left_ang))
    servo_right.duty_u16(angle_to_duty(right_ang))

# ==========================================
# 3. Keyboard Input Setup (Non-Blocking)
# ==========================================
poll_obj = select.poll()
poll_obj.register(sys.stdin, select.POLLIN)

def check_keyboard():
    """Reads keyboard input from Thonny without pausing the code."""
    if poll_obj.poll(0):
        ch = sys.stdin.read(1)
        if ch == '\x1b': # Escape character (indicates arrow keys)
            ch2 = sys.stdin.read(2)
            if ch2 == '[C': return 'RIGHT'
            if ch2 == '[D': return 'LEFT'
        return ch.lower()
    return None

# ==========================================
# 4. Main Execution Loop
# ==========================================
# Start both servos at their 90-degree homed position
left_angle = 90
right_angle = 90
step_size = 2 # Degrees to move per key press

update_servos(left_angle, right_angle)

print("\n--- GRIPPER CONTROL ACTIVE ---")
print("Left Servo  (90 to 115): Press 'A' and 'D'")
print("Right Servo (65 to 90) : Press 'Left Arrow' and 'Right Arrow'")
print("Press 'Q' to quit.\n")

last_print_time = time.ticks_ms()

try:
    while True:
        key = check_keyboard()
        moved = False

        if key == 'a':
            left_angle -= step_size
            moved = True
        elif key == 'd':
            left_angle += step_size
            moved = True
        elif key == 'LEFT':
            right_angle -= step_size
            moved = True
        elif key == 'RIGHT':
            right_angle += step_size
            moved = True
        elif key == 'q':
            break

        # Apply strict physical constraints
        left_angle = max(90, min(115, left_angle))
        right_angle = max(65, min(90, right_angle))

        if moved:
            update_servos(left_angle, right_angle)
            print(f"Angles -> Left: {left_angle} | Right: {right_angle}")

        # Print BMS data every 2 seconds to avoid spamming the console
        current_time = time.ticks_ms()
        if time.ticks_diff(current_time, last_print_time) > 2000:
            volts = ina.get_bus_voltage_V()
            amps = ina.get_current_mA()
            print(f"[UPS] {volts:.2f}V | Draw: {amps:.2f}mA")
            last_print_time = current_time

        time.sleep(0.02) # Small delay to keep the loop stable

except KeyboardInterrupt:
    pass
finally:
    print("\nStopping...")
    servo_left.deinit()
    servo_right.deinit()