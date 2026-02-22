import sensor
import pyb
import time

from collections import deque

uart = pyb.UART(1, 57600, timeout_char=50)

class PidControl:
    DEFAULT_KP = 2.0
    DEFAULT_KI = 0.1
    DEFAULT_KD = 1.0

    def __init__(self, max_corr, kp=DEFAULT_KP, ki=DEFAULT_KI, kd=DEFAULT_KD):
        self._max_corr = max_corr
        self._kp = kp
        self._ki = ki
        self._kd = kd
        self._previous_error = 0
        self._integral = 0

    def update(self, error):
        p = error * self._kp

        self._integral += error
        self._integral = max(-100, min(100, self._integral))
        i = self._integral * self._ki

        d = (error - self._previous_error) * self._kd
        self._previous_error = error

        cv = p + i + d
        cv = max(-self._max_corr, min(self._max_corr, cv))

        return cv


class Motors:
    MIN_SPEED = -100
    MAX_SPEED = 100

    def run(self, left_speed: int, right_speed:int):
        motor_left = int(max(self.MIN_SPEED, min(self.MAX_SPEED, left_speed)))
        motor_right = int(max(self.MIN_SPEED, min(self.MAX_SPEED, right_speed)))
        uart.write(""+str(motor_left)+";"+str(motor_right)+"m")

    def stop(self):
        self.drive(0, 0)


class Camera:
    WIDTH = 320
    BORDER_L = 55
    BORDER_R = 45
    HEIGHT = 240
    MIN_WIDTH = 20
    MIN_HEIGHT = 20
    CENTER = WIDTH // 2
    ROI = [BORDER_L, 0, WIDTH - BORDER_L - BORDER_R, HEIGHT]
    ROI_FAR = [BORDER_L - 5, 0, WIDTH - BORDER_L - BORDER_R + 10, 120]
    ROI_NEAR = [BORDER_L, 50, WIDTH - BORDER_L - BORDER_R, HEIGHT]
    BLACK_THRESHOLD = [(0, 35, -128, 127, -128, 127)]
    GREEN_THRESHOLD = [(30, 70, -60, -20, -10, 40)]
    LINE_ANGLE_POS_RANGE = range(0, 90)
    LINE_ANGLE_NEG_RANGE = range(91, 180)

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
        self._angles = deque([], 100)
        self._offsets = deque([], 100)


    def update(self):
        self._img = sensor.snapshot()
        self._img_gs = self._img.to_grayscale(copy=True)

        self._blobs_near = self._img_gs.find_blobs([(0, 50)],
            roi=self.ROI_NEAR,
            pixels_threshold=150,
            area_threshold=80,
            merge=True)

        self._blobs_far = self._img_gs.find_blobs([(0, 50)],
            roi=self.ROI_FAR,
            pixels_threshold=150,
            area_threshold=80,
            merge=True)

        self.draw_debug()


    def find_lines(self):
        if self._img_gs is None:
            return []

        lines = self._img_gs.find_lines(
            roi=self.ROI_NEAR, threshold=1000, theta_margin=25, rho_margin=25)

        return lines


    def get_angle_and_offset(self):
        if not self._blobs_near:
            return None, None

        lines = self.find_lines()


        relevant_lines = []
        for line in lines:
            # store only relevant lines
            if (line.theta() in self.LINE_ANGLE_POS_RANGE) \
                or (line.theta() in self.LINE_ANGLE_NEG_RANGE):
                relevant_lines.append(line)

        for line in relevant_lines:
            # normalize line agles and store
            if (line.theta() in self.LINE_ANGLE_POS_RANGE):
                self._angles.append(line.theta())
            elif (line.theta() in self.LINE_ANGLE_NEG_RANGE):
                self._angles.append(line.theta() - 180)
            self._img.draw_line(line.line(), color=(255, 0, 0))
            # calculate offset from center and store
            offset = ((line.x1() + line.x2()) / 2) - self.CENTER
            self._offsets.append(offset)

        if len(self._angles) == 0 or len(self._offsets) == 0:
            return None, None

        angle = int(sum(self._angles) / len(self._angles))
        offset = int(sum( self._offsets) / len( self._offsets))
        self._img.draw_string(240, 210, "{}".format(angle), color=(255, 0, 0), scale=2)
        self._img.draw_string(60, 210, "{}".format(offset), color=(255, 0, 0), scale=2)
        return angle, offset


    def draw_debug(self, verbose=False):
        self._img.draw_rectangle(self.ROI_FAR, color=(0, 100, 0), thickness=1)
        self._img.draw_rectangle(self.ROI_NEAR, color=(0, 0, 0), thickness=1)
        if verbose:
            for blob in self._blobs_far:
                self._img.draw_rectangle(blob.rect(), color=(0, 100, 0), thickness=3)
            for blob in self._blobs_near:
                self._img.draw_rectangle(blob.rect(), color=(100, 0, 0), thickness=3)


class Robot:
    BASE_SPEED = 40
    MIN_SPEED = 20
    MAX_SPEED = 80

    def __init__(self):
        self._motors = Motors()
        self._camera = Camera()
        self._pid_angle = PidControl(max_corr=60, kp=2.0, ki=0.1, kd=1.0)
        self._pid_offset = PidControl(max_corr=60, kp=0.5, ki=0.0, kd=0.6)

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
        angle, offset = self._camera.get_angle_and_offset()

        if angle is None or offset is None:
            return

        offset_corr = 0
        if offset < 10:
            offset_corr = -20
        elif offset > 10:
            offset_corr = 20

        angle_corr = self._pid_angle.update(angle)
        offset_corr = self._pid_offset.update(offset)
        print("OC:{}".format(offset_corr))

        left_speed = self.BASE_SPEED + offset_corr + angle_corr
        right_speed = self.BASE_SPEED - offset_corr - angle_corr

        #print("LS:{}".format(left_speed))
        #print("RS:{}".format(right_speed))

        #self._drive(left_speed, right_speed)


    def _drive(self, left_speed, right_speed):
        self._motors.run(left_speed, right_speed)


robot = Robot()

def init():
    robot.init()

def start():
    clock = time.clock()
    robot.calibrate()

    while True:
        clock.tick() # next cycle
        robot.navigate()
        #print("FPS:{}".format(clock.fps()))

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
