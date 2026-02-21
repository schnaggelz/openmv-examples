import sensor
import pyb
import time
import math
import image

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
    MIN_WIDTH = 20
    MIN_HEIGHT = 20
    CENTER = WIDTH // 2
    ROI = [BORDER_L, 0, WIDTH - BORDER_L - BORDER_R, HEIGHT]
    ROI_FAR = [BORDER_L - 5, 0, WIDTH - BORDER_L - BORDER_R + 10, 120]
    ROI_NEAR = [BORDER_L, 80, WIDTH - BORDER_L - BORDER_R, HEIGHT]
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
        self._img_gs = None
        self._blobs_near = []
        self._blobs_far = []
        #self._last_best_line = None

    def update(self):
        self._img = sensor.snapshot()
        self._img_gs = self._img.to_grayscale(copy=True)

        self._blobs_near = self._img_gs.find_blobs([(0, 50)],
            roi=Camera.ROI_NEAR,
            pixels_threshold=150,
            area_threshold=80,
            merge=True)

        self._blobs_far = self._img_gs.find_blobs([(0, 50)],
            roi=Camera.ROI_NEAR,
            pixels_threshold=150,
            area_threshold=80,
            merge=True)

        self.draw_debug()

    def find_lines(self):
        if self._img_gs is None:
            return []

        line_blobs = []
        for blob in self._blobs_near:
            width = blob.w()
            height = blob.h()
            aspect_ratio = width / max(height, 1)
            #print("aspect_ratio: {}".format(aspect_ratio))

            if width > Camera.MIN_WIDTH and \
                height > Camera.MIN_HEIGHT and \
                aspect_ratio < 0.5:
                line_blobs.append(blob)

        return line_blobs

    def get_angle(self):
        if not self._blobs_near:
            return None, False

        line_blobs = self.find_lines()
        if len(line_blobs) == 0:
            print("NO LINES")
            return None, False

        for line_blob in line_blobs:
            pass
            #self._img.draw_rectangle(line_blob, color=(0, 255, 0))
            #(x1, y1, x2, y2) = line.major_axis_line()
            #self._img.draw_line(x1, y1, x2, y2, color=(255, 255, 0), thickness=2)

        return 0, False

    def draw_debug(self):
        self._img.draw_rectangle(Camera.ROI_NEAR, color=(0, 0, 100), thickness=1)
        self._img.draw_rectangle(Camera.ROI_FAR, color=(0, 100, 0), thickness=1)
        for blob in self._blobs_near:
            self._img.draw_rectangle(blob.rect(), color=(100, 0, 0), thickness=3)

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
        self._camera.update() # get all raw data
        angle, state = self._camera.get_angle()

        #if state is False:
            #print("NO ANGLE!")

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
        print("FPS:{}".format(clock.fps()))

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
