from machine import Pin, PWM

# Servo signal pin
SERVO_PIN = 16

# Onboard LED
led = Pin("LED", Pin.OUT)
led.on()   # Kod çalışınca LED yansın

# Servo PWM setup
servo = PWM(Pin(SERVO_PIN))
servo.freq(50)   # Standard servo frequency: 50 Hz


def angle_to_duty_ns(angle):
    """
    Converts angle 0-180 degrees to PWM pulse width.
    Typical micro servo pulse range:
    0 deg   -> 500 us
    180 deg -> 2500 us
    """
    angle = max(0, min(180, angle))

    min_pulse_us = 500
    max_pulse_us = 2500

    pulse_us = min_pulse_us + (angle / 180) * (max_pulse_us - min_pulse_us)
    return int(pulse_us * 1000)


def set_servo_angle(angle):
    duty_ns = angle_to_duty_ns(angle)
    servo.duty_ns(duty_ns)


while True:
    try:
        user_input = input("Enter angle between 0 and 180: ")

        if user_input.lower() in ["q", "quit", "exit"]:
            print("Exiting...")
            servo.deinit()
            led.off()
            break

        angle = float(user_input)

        if angle < 0 or angle > 180:
            print("Please enter an angle between 0 and 180.")
            continue

        set_servo_angle(angle)
        print("Servo moved to", angle, "degrees")

    except ValueError:
        print("Invalid input. Please enter a number.")