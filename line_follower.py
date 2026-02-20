import sensor
import pyb
import time
import math

uart = pyb.UART(1, 57600, timeout_char=50)

class PidControl:
    KP = 2.0
    KI = 0.0
    KD = 1.0

    def __init__(self, setpoint, dt):
        self._setpoint = setpoint
        self._dt = dt
        self._pv = 0
        self._error = 0
        self._previous_error = 0
        self._integral = 0
        self._derivative = 0

    def update(self, pv):
        self._error = self._setpoint - pv
        self._integral += self._error * self._dt
        self._derivative = (self._error - self._previous_error) / self._dt
        cv = PidControl.KP * self._error + \
            PidControl.KI * self._integral + \
            PidControl.KD * self._derivative
        return cv


class Motors:
    MIN_SPEED = -100
    MAX_SPEED = 100

    def run(self, left_speed: int, right_speed:int):
        motor_left = int(max(Robot.MIN_SPEED, min(Robot.MAX_SPEED, left_speed)))
        motor_right = int(max(Robot.MIN_SPEED, min(Robot.MAX_SPEED, right_speed)))
        uart.write(""+str(motor_left)+";"+str(motor_right)+"m")

    def stop(self):
        self.drive(0, 0)


class Camera:
    WIDTH = 320
    BORDER_L = 50
    BORDER_R = 40
    HEIGHT = 240
    CENTER = WIDTH // 2
    RANGE_NEAR = HEIGHT // 2
    RANGE_FAR = HEIGHT
    ROI = (BORDER_L, 0, WIDTH - BORDER_L - BORDER_R, HEIGHT)
    ROI_NEAR = (BORDER_L, 0, WIDTH - BORDER_L - BORDER_R, RANGE_NEAR - 1)
    ROI_FAR = (BORDER_L, RANGE_NEAR, WIDTH - BORDER_L - BORDER_R, RANGE_FAR - RANGE_NEAR)
    BLACK_THRESHOLD = [(0, 35, -128, 127, -128, 127)]
    GREEN_THRESHOLD = [(30, 70, -60, -20, -10, 40)]

    def __init__(self):
        sensor.reset()
        sensor.set_pixformat(sensor.RGB565)
        sensor.set_framesize(sensor.QVGA)
        sensor.skip_frames(time=1000)

        sensor.set_auto_gain(False, gain_db=10)
        sensor.set_auto_whitebal(False)
        sensor.set_auto_exposure(False, exposure_us=10000)

        self._img = None

    def find_lines(self):
        pass

    def get_angle(self):
        self._img = sensor.snapshot()
        img_gs = self._img.to_grayscale(copy=True)

        self.draw_debug()

        # and now ...

    def draw_debug(self):
        #roi = Camera.ROI
        ron = Camera.ROI_NEAR
        rof = Camera.ROI_FAR
        #self._img.draw_rectangle(roi[0], roi[1], roi[2], roi[3], color=(255, 0, 0))
        self._img.draw_rectangle(ron[0], ron[1], ron[2], ron[3], color=(0, 255, 0))
        self._img.draw_rectangle(rof[0], rof[1], rof[2], rof[3], color=(0, 0, 255))

class Robot:
    BASE_SPEED = 80
    MAX_CORRECTION = 60
    MIN_SPEED = 20

    def __init__(self):
        self._motors = Motors()
        self._camera = Camera()
        self._pid = PidControl(Camera.CENTER, 0.1)

        ## check the below
        self.ist_near = Camera.CENTER
        self.ist_far = Camera.CENTER
        self.alpha = 0
        self.last_error = 0
        self.integral = 0
        self.green_left = 0
        self.green_right = 0
        self.green_cooldown = 0
        self.line_found = False
        self.lost_counter = 0
        self.calibrated = False

    def init(self):
        pass

    def stop(self):
        pass

    def calibrate(self):
        pass

    def navigate(self):
        angle = self._camera.get_angle()
        # weitermachen

    def __drive(self, left, right):
        self._motors.run(left, right)


robot = Robot()

def init():
    robot.init()

def start():
    clock = time.clock()
    robot.calibrate()

    while True:
        clock.tick() # next cycle
        robot.navigate()
        print(clock.fps())

def stop():
    robot.stop()

if __name__ == "__main__":
    print("INIT")
    init()
    try:
        print("START")
        start()
    except KeyboardInterrupt:
        print("STOP")
        stop()
    except Exception as e:
        print("EXCEPTION: {}".format(e))
        stop()
