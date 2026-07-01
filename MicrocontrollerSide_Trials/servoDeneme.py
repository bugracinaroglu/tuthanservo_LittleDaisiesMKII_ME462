from machine import Pin, PWM
from time import sleep, ticks_ms, ticks_diff
import sys

try:
    import uselect as select
except ImportError:
    import select


SERVO_PIN = 16

FREQ = 50
MIN_US = 500
MAX_US = 2500

STEP_DEG = 2
MOVE_DELAY = 0.03

# Tuş bırakılınca servo kaç ms sonra dursun?
# Keyboard auto-repeat için biraz yüksek tutuyoruz.
HOLD_TIMEOUT_MS = 600


class ServoController:
    def __init__(self, pin, max_angle=180, freq=50, min_us=500, max_us=2500):
        self.max_angle = max_angle
        self.freq = freq
        self.min_us = min_us
        self.max_us = max_us

        self.servo = PWM(Pin(pin))
        self.servo.freq(freq)

        self.current_angle = max_angle / 2
        self.set_angle(self.current_angle)

    def angle_to_duty_ns(self, angle):
        angle = max(0, min(angle, self.max_angle))

        pulse_us = self.min_us + (angle / self.max_angle) * (self.max_us - self.min_us)
        return int(pulse_us * 1000)

    def set_angle(self, angle):
        angle = max(0, min(angle, self.max_angle))
        self.current_angle = angle

        self.servo.duty_ns(self.angle_to_duty_ns(angle))
        print("Angle:", round(angle, 1))

    def move_by(self, delta):
        self.set_angle(self.current_angle + delta)

    def center(self):
        self.set_angle(self.max_angle / 2)

    def deinit(self):
        self.servo.deinit()


def key_pressed():
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return None


def manual_control(servo):
    print("\nManual control mode")
    print("Hold r -> CW")
    print("Hold s -> CCW")
    print("Press q -> return menu")
    print("Not: Thonny kullanıyorsan tuşlar Enter bekleyebilir.")

    direction = 0
    last_key_time = ticks_ms()

    while True:
        key = key_pressed()

        if key == "q":
            print("Manual control stopped.")
            return

        elif key == "r":
            direction = 1
            last_key_time = ticks_ms()

        elif key == "s":
            direction = -1
            last_key_time = ticks_ms()

        # Tuş basılı değilse, terminalden yeni karakter gelmez.
        # Bir süre karakter gelmezse motoru durduruyoruz.
        if ticks_diff(ticks_ms(), last_key_time) > HOLD_TIMEOUT_MS:
            direction = 0

        if direction == 1:
            servo.move_by(STEP_DEG)

        elif direction == -1:
            servo.move_by(-STEP_DEG)

        sleep(MOVE_DELAY)


def oscillation(servo):
    print("\nOscillation started.")
    print("Press q to stop.")

    while True:
        angle = 0
        while angle <= servo.max_angle:
            key = key_pressed()
            if key == "q":
                print("Oscillation stopped.")
                return

            servo.set_angle(angle)
            angle += STEP_DEG
            sleep(0.02)

        angle = servo.max_angle
        while angle >= 0:
            key = key_pressed()
            if key == "q":
                print("Oscillation stopped.")
                return

            servo.set_angle(angle)
            angle -= STEP_DEG
            sleep(0.02)


def ask_max_angle():
    print("Servo max angle seçiniz:")
    print("180 -> normal DS3225 180 derece")
    print("270 -> DS3225 270 derece versiyon")
    print("Başka değer de yazabilirsin.")

    value = input("Max angle: ")

    try:
        max_angle = float(value)
    except:
        print("Geçersiz değer. 180 derece kabul edildi.")
        max_angle = 180

    if max_angle <= 0:
        print("Geçersiz değer. 180 derece kabul edildi.")
        max_angle = 180

    return max_angle


max_angle = ask_max_angle()
servo = ServoController(
    pin=SERVO_PIN,
    max_angle=max_angle,
    freq=FREQ,
    min_us=MIN_US,
    max_us=MAX_US
)

while True:
    print("\nServo Control Menu")
    print("1 - Açıya gönder")
    print("2 - Manual control: hold r = CW, hold s = CCW")
    print("3 - Center / Home")
    print("4 - Sürekli oscillation")
    print("q - Çıkış")

    choice = input("Seçim: ")

    if choice == "1":
        try:
            angle = float(input("Açı giriniz 0-" + str(max_angle) + ": "))
            servo.set_angle(angle)
        except:
            print("Geçersiz açı.")

    elif choice == "2":
        manual_control(servo)

    elif choice == "3":
        servo.center()

    elif choice == "4":
        oscillation(servo)

    elif choice == "q":
        print("Program bitti.")
        servo.deinit()
        break

    else:
        print("Geçersiz seçim.")