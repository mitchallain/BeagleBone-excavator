import Adafruit_BBIO.PWM as PWM
import time


class Servo():

    def __init__(self, servo_pin):
        # Define duty cycle parameters for all servos
        self.duty_min = 4.94
        self.duty_max = 9.97
        self.duty_span = self.duty_max - self.duty_min
        self.duty_mid = 7.47
        self.freq = 50.625

        self.servo_pin = servo_pin
        print 'starting servo PWM'
        PWM.start(self.servo_pin, self.duty_mid, self.freq)

    def set_servo_angle(self, angle):
        angle_f = float(angle)
        duty = ((angle_f / 180) * self.duty_span + self.duty_min)
        PWM.set_duty_cycle(self.servo_pin, duty)

    def close_servo(self):
        PWM.stop(self.servo_pin)
        PWM.cleanup()

    def neutral_pwm(self):
        PWM.set_duty_cycle(self.servo_pin, self.duty_mid)


if __name__ == "__main__":

    servo1 = Servo("P9_22")

    while True:
        userCommand = raw_input("Direction 'L/R/exit': ")
        if userCommand == 'exit':
            servo1.close_servo()
            break
        elif userCommand == 'L':
            PWM.set_duty_cycle(servo1.servo_pin, 6.5)
            time.sleep(0.3)
            servo1.neutral_pwm()
        elif userCommand == 'R':
            PWM.set_duty_cycle(servo1.servo_pin, 8.5)
            time.sleep(0.3)
            servo1.neutral_pwm()
        else:
            print("invalid input")
        # servo1.set_servo_angle(angle)
