from machine import Pin, PWM, ADC
from time import ticks_ms, ticks_diff, sleep_ms
import sys

try:
    import uselect as select
except ImportError:
    import select


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
# FSR Controller
# =====================================================

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

    def read_raw_data(self):
        self.last_raw = self.adc.read_u16()
        return self.last_raw

    def _update_window(self, raw):
        self.raw_values.append(raw)

        if len(self.raw_values) > self.window_size:
            self.raw_values.pop(0)

    def _calculate_average_raw(self):
        if len(self.raw_values) == 0:
            return 0.0

        return sum(self.raw_values) / len(self.raw_values)

    def update(self):
        now = ticks_ms()

        if ticks_diff(now, self.last_sample_time) < self.sample_interval_ms:
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

        # Ortalama penceresi dolmadan temas kararı verme.
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

    def get_raw(self):
        return self.last_raw

    def get_average_raw(self):
        return self.average_raw

    def get_voltage(self):
        return self.last_voltage

    def get_average_voltage(self):
        return self.average_voltage


# =====================================================
# Gripper Controller
# =====================================================

class GripperController:
    def __init__(
        self,
        left_servo,
        right_servo,
        left_fsr,
        right_fsr,
        left_mean_angle,
        right_mean_angle,
        open_offset,
        close_offset,
        initial_position="open",
        step_deg=1,
        move_interval_ms=20
    ):
        self.left_servo = left_servo
        self.right_servo = right_servo

        self.left_fsr = left_fsr
        self.right_fsr = right_fsr

        self.left_mean_angle = left_mean_angle
        self.right_mean_angle = right_mean_angle

        self.open_offset = open_offset
        self.close_offset = close_offset

        # Açık konumdaki mutlak açılar
        self.left_open_angle = (
            self.left_mean_angle - self.open_offset
        )

        self.right_open_angle = (
            self.right_mean_angle + self.open_offset
        )

        # FSR algılanmazsa gidilecek maksimum kapanma açıları
        self.left_close_angle = (
            self.left_mean_angle + self.close_offset
        )

        self.right_close_angle = (
            self.right_mean_angle - self.close_offset
        )

        self.step_deg = step_deg
        self.move_interval_ms = move_interval_ms

        self.last_move_time = ticks_ms()
        self.mode = "stop"

        # Kapanma sırasında temas algılandığında kilitlenir.
        self.left_contact = False
        self.right_contact = False

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
        initial_position = initial_position.lower()

        if initial_position == "open":
            self.open_immediate()

        elif initial_position == "close":
            self.close_immediate()

        elif initial_position == "mean":
            self.mean_immediate()

        else:
            raise ValueError(
                "initial_position open, close veya mean olmalıdır."
            )

    def open_immediate(self):
        self.left_servo.go_to_angle(self.left_open_angle)
        self.right_servo.go_to_angle(self.right_open_angle)

        self.left_contact = False
        self.right_contact = False
        self.mode = "stop"

        print(
            "Initial open position | Left:",
            self.left_open_angle,
            "| Right:",
            self.right_open_angle
        )

    def close_immediate(self):
        self.left_servo.go_to_angle(self.left_close_angle)
        self.right_servo.go_to_angle(self.right_close_angle)

        self.left_contact = False
        self.right_contact = False
        self.mode = "stop"

        print(
            "Initial closed position | Left:",
            self.left_close_angle,
            "| Right:",
            self.right_close_angle
        )

    def mean_immediate(self):
        self.left_servo.go_to_angle(self.left_mean_angle)
        self.right_servo.go_to_angle(self.right_mean_angle)

        self.left_contact = False
        self.right_contact = False
        self.mode = "stop"

        print(
            "Initial mean position | Left:",
            self.left_mean_angle,
            "| Right:",
            self.right_mean_angle
        )

    def start_opening(self):
        self.left_contact = False
        self.right_contact = False

        self.mode = "open"
        print("Opening gripper...")

    def start_closing(self):
        self.left_contact = False
        self.right_contact = False

        self.mode = "close_fsr"
        print("Closing gripper with FSR control...")

    def stop(self):
        self.mode = "stop"

        print(
            "Gripper stopped | Left:",
            round(self.left_servo.current_angle, 1),
            "| Right:",
            round(self.right_servo.current_angle, 1)
        )

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

    def _update_fsr_sensors(self):
        self.left_fsr.update()
        self.right_fsr.update()

    def _print_fsr_stop_data(self, name, servo, fsr):
        print()
        print(name, "FSR contact detected.")
        print(
            "Stopped angle:",
            round(servo.current_angle, 1)
        )
        print(
            "Raw:",
            fsr.get_raw(),
            "| Average raw:",
            round(fsr.get_average_raw(), 1)
        )
        print(
            "Voltage:",
            round(fsr.get_voltage(), 3),
            "V | Average voltage:",
            round(fsr.get_average_voltage(), 3),
            "V"
        )
        print(
            "Threshold:",
            round(fsr.threshold_voltage, 3),
            "V"
        )

    def _update_contact_latches(self):
        """
        FSR teması algılandığında ilgili servoyu durdurur.
        Temas bilgisi mevcut kapanma işlemi boyunca kilitlenir.
        """

        if self.mode != "close_fsr":
            return None

        contact_event = False

        if not self.left_contact and self.left_fsr.is_pressed():
            self.left_contact = True
            contact_event = True

            self._print_fsr_stop_data(
                "Left",
                self.left_servo,
                self.left_fsr
            )

        if not self.right_contact and self.right_fsr.is_pressed():
            self.right_contact = True
            contact_event = True

            self._print_fsr_stop_data(
                "Right",
                self.right_servo,
                self.right_fsr
            )

        if contact_event:
            return "contact"

        return None

    def update(self):
        # Sensörler hareket zamanlamasından bağımsız okunur.
        self._update_fsr_sensors()

        contact_event = self._update_contact_latches()

        now = ticks_ms()

        if ticks_diff(now, self.last_move_time) < self.move_interval_ms:
            return contact_event

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
                self.mode = "stop"

                print()
                print(
                    "Gripper opened | Left:",
                    round(self.left_servo.current_angle, 1),
                    "| Right:",
                    round(self.right_servo.current_angle, 1)
                )

                return "open_done"

        elif self.mode == "close_fsr":
            if self.left_contact:
                left_done = True
            else:
                left_done = self._move_servo_towards(
                    self.left_servo,
                    self.left_close_angle
                )

            if self.right_contact:
                right_done = True
            else:
                right_done = self._move_servo_towards(
                    self.right_servo,
                    self.right_close_angle
                )

            if left_done and right_done:
                self.mode = "stop"

                print()
                print("Gripper closing completed.")
                print(
                    "Final angles | Left:",
                    round(self.left_servo.current_angle, 1),
                    "| Right:",
                    round(self.right_servo.current_angle, 1)
                )
                print(
                    "FSR contacts | Left:",
                    self.left_contact,
                    "| Right:",
                    self.right_contact
                )
                print(
                    "Left FSR average:",
                    round(self.left_fsr.get_average_voltage(), 3),
                    "V | Right FSR average:",
                    round(self.right_fsr.get_average_voltage(), 3),
                    "V"
                )

                return "close_done"

        return contact_event

    def print_status(self):
        print()
        print("-----------------------")
        print("Mode:", self.mode)

        print(
            "Angles | Left:",
            round(self.left_servo.current_angle, 1),
            "| Right:",
            round(self.right_servo.current_angle, 1)
        )

        print(
            "Left FSR | Raw:",
            self.left_fsr.get_raw(),
            "| Avg:",
            round(self.left_fsr.get_average_raw(), 1),
            "| Voltage:",
            round(self.left_fsr.get_average_voltage(), 3),
            "V | Pressed:",
            self.left_fsr.is_pressed(),
            "| Latched:",
            self.left_contact
        )

        print(
            "Right FSR | Raw:",
            self.right_fsr.get_raw(),
            "| Avg:",
            round(self.right_fsr.get_average_raw(), 1),
            "| Voltage:",
            round(self.right_fsr.get_average_voltage(), 3),
            "V | Pressed:",
            self.right_fsr.is_pressed(),
            "| Latched:",
            self.right_contact
        )

        print("-----------------------")

    def deinit(self):
        self.left_servo.deinit()
        self.right_servo.deinit()


# =====================================================
# Terminal Helper
# =====================================================

def read_command():
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.readline().strip().lower()

    return None


def print_menu():
    print()
    print("-----------------------")
    print("Gripper FSR test")
    print("c + Enter : FSR kontrollü kapat")
    print("o + Enter : aç")
    print("s + Enter : durdur ve mevcut konumu tut")
    print("p + Enter : açı ve FSR değerlerini göster")
    print("q + Enter : çıkış")
    print("-----------------------")
    print("Enter: ", end="")


# =====================================================
# Hardware Settings
# =====================================================

LEFT_SERVO_PIN = 16
RIGHT_SERVO_PIN = 17

LEFT_FSR_PIN = 26
RIGHT_FSR_PIN = 27

LEFT_MEAN_ANGLE = 85
RIGHT_MEAN_ANGLE = 100

OPEN_OFFSET = 30
CLOSE_OFFSET = 10

LEFT_FSR_THRESHOLD_VOLTAGE = 2.0
RIGHT_FSR_THRESHOLD_VOLTAGE = 2.0

FSR_WINDOW_SIZE = 5
FSR_SAMPLE_INTERVAL_MS = 5

STEP_DEG = 1
MOVE_INTERVAL_MS = 20

# "open", "close" veya "mean"
INITIAL_POSITION = "open"


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

left_fsr = FSR(
    pin=LEFT_FSR_PIN,
    threshold_voltage=LEFT_FSR_THRESHOLD_VOLTAGE,
    window_size=FSR_WINDOW_SIZE,
    sample_interval_ms=FSR_SAMPLE_INTERVAL_MS,
    active_high=True
)

right_fsr = FSR(
    pin=RIGHT_FSR_PIN,
    threshold_voltage=RIGHT_FSR_THRESHOLD_VOLTAGE,
    window_size=FSR_WINDOW_SIZE,
    sample_interval_ms=FSR_SAMPLE_INTERVAL_MS,
    active_high=True
)

gripper = GripperController(
    left_servo=left_servo,
    right_servo=right_servo,

    left_fsr=left_fsr,
    right_fsr=right_fsr,

    left_mean_angle=LEFT_MEAN_ANGLE,
    right_mean_angle=RIGHT_MEAN_ANGLE,

    open_offset=OPEN_OFFSET,
    close_offset=CLOSE_OFFSET,

    initial_position=INITIAL_POSITION,

    step_deg=STEP_DEG,
    move_interval_ms=MOVE_INTERVAL_MS
)


# =====================================================
# Main
# =====================================================

print_menu()

try:
    while True:
        command = read_command()

        if command is not None:
            if command == "c":
                gripper.start_closing()

            elif command == "o":
                gripper.start_opening()

            elif command == "s":
                gripper.stop()

            elif command == "p":
                gripper.print_status()

            elif command == "q":
                print()
                print("Exiting...")
                break

            elif command != "":
                print()
                print("Invalid command.")

            # Her terminal komutundan sonra menüyü yeniden yaz.
            if command != "q":
                print_menu()

        event = gripper.update()

        # Temas veya hareket tamamlanınca menüyü yeniden yaz.
        if event is not None:
            print_menu()

        sleep_ms(1)

except KeyboardInterrupt:
    print()
    print("Gripper test stopped.")

finally:
    gripper.deinit()
    print("PWM signals disabled.")