import sensor
import pyb
import time

uart = pyb.UART(1, 57600, timeout_char=50)

class PidControl:
    KP = 2.0
    KI = 0.1
    KD = 1.0
    MAX_CORR = 60

    def __init__(self):
        self._previous_error = 0
        self._integral = 0

    def update(self, error):
        p = error * self.KP

        self._integral += error
        self._integral = max(-100, min(100, self._integral))
        i = self._integral * self.KI

        d = (error - self._previous_error) * self.KD
        self._previous_error = error

        cv = p + i + d
        cv = max(-self.MAX_CORR, min(self.MAX_CORR, cv))

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


    def get_angle(self):
        if not self._blobs_near:
            return None

        lines = self.find_lines()

        min_range = range(0, 45)
        max_range = range(135, 180)
        relevant_lines = []
        for line in lines:
            if (line.theta() in min_range) or (line.theta() in max_range):
                relevant_lines.append(line)

        angles = []
        for line in relevant_lines:
            if (line.theta() in min_range):
                angles.append(line.theta())
            elif (line.theta() in max_range):
                angles.append(line.theta() - 180)
            self._img.draw_line(line.line(), color=(255, 0, 0))

        if len(angles) == 0:
            return None

        angle = sum(angles) / len(angles)
        self._img.draw_string(240, 210, "{}".format(int(angle)), color=(255, 0, 0), scale=2)

        return angle


    def draw_debug(self, verbose=False):
        self._img.draw_rectangle(self.ROI_FAR, color=(0, 100, 0), thickness=1)
        self._img.draw_rectangle(self.ROI_NEAR, color=(100, 0, 0), thickness=1)
        if verbose:
            for blob in self._blobs_far:
                self._img.draw_rectangle(blob.rect(), color=(0, 100, 0), thickness=3)
            for blob in self._blobs_near:
                self._img.draw_rectangle(blob.rect(), color=(100, 0, 0), thickness=3)


class Robot:
    BASE_SPEED = 80
    MAX_CORRECTION = 60
    MIN_SPEED = 20
    MAX_SPEED = 80

    def __init__(self):
        self._motors = Motors()
        self._camera = Camera()
        self._pid = PidControl()

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
        self._self.update() # get all raw data
        angle = self._self.get_angle()

        correction = 0
        if (angle < -5):
            correction = 5
        if (angle > 5):
            correction = - 5

        left_speed = self.BASE_SPEED - correction
        right_speed = self.BASE_SPEED + correction

        if left_speed < self.MIN_SPEED and left_speed > -self.MIN_SPEED:
            left_speed = self.MIN_SPEED if left_speed >= 0 else -self.MIN_SPEED

        if right_speed < self.MIN_SPEED and right_speed > -self.MIN_SPEED:
            right_speed = self.MIN_SPEED if right_speed >= 0 else -self.MIN_SPEED


        #correction_value = self._pid.update(angle)
        print("LS:{}".format(left_speed))
        print("RS:{}".format(right_speed))

        self._drive(left_speed, right_speed)


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
