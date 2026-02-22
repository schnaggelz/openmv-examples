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
    ROI_NEAR = [BORDER_L, 100, WIDTH - BORDER_L - BORDER_R, HEIGHT]
    LINE_ANGLE_POS_RANGE = range(0, 90)
    LINE_ANGLE_NEG_RANGE = range(91, 180)
    GREEN_THRESHOLD = [(20, 80, -70, -15, 10, 60)]

    def __init__(self):
        sensor.reset()
        sensor.set_pixformat(sensor.RGB565)
        sensor.set_framesize(sensor.QVGA)
        sensor.skip_frames(time=1000)

        sensor.set_auto_gain(False, gain_db=10)
        sensor.set_auto_whitebal(False)
        sensor.set_auto_exposure(False, exposure_us=10000)

        self._image = None
        self._lines = []
        self._rects = []
        self._last_angle = 0
        self._last_offset = 0
        self._angles = deque([], 10)
        self._offsets = deque([], 10)

    @property
    def image(self):
        return self._image

    def update(self):
        img = sensor.snapshot()

        self._lines = img.find_lines(
            roi=self.ROI_NEAR, threshold=1000, theta_margin=25, rho_margin=25)

        self._rects = img.find_blobs(self.GREEN_THRESHOLD, roi=self.ROI_FAR,
                                     pixels_threshold=300, merge=True)

        self.draw_debug(img)
        self._image = img

    def get_angle_and_offset(self):

        relevant_lines = []
        for line in self._lines:
            # store only relevant lines
            if (line.theta() in self.LINE_ANGLE_POS_RANGE \
                or line.theta() in self.LINE_ANGLE_NEG_RANGE):
                relevant_lines.append(line)

        for line in relevant_lines:
            # normalize line agles and store
            if (line.theta() in self.LINE_ANGLE_POS_RANGE):
                self._angles.append(line.theta())
            elif (line.theta() in self.LINE_ANGLE_NEG_RANGE):
                self._angles.append(line.theta() - 180)
            # calculate offset from center and store
            offset = ((line.x1() + line.x2()) / 2) - self.CENTER
            self._offsets.append(offset)

        angle = int(sum(self._angles) / len(self._angles))
        offset = int(sum( self._offsets) / len( self._offsets))
        return angle , offset

    def get_num_rects_lr(self):

        num_rects_l = 0
        num_rects_r = 0
        for rect in self._rects:
            center = rect.cx()
            if center <= self.WIDTH:
                num_rects_l += 1
            else:
                num_rects_r += 1
        return num_rects_l, num_rects_r

    def draw_debug(self, img):

        img.draw_rectangle(self.ROI_FAR, color=(0, 100, 0), thickness=1)
        img.draw_rectangle(self.ROI_NEAR, color=(0, 0, 0), thickness=1)

        angle = offset = 0
        if len(self._angles) > 0 and len(self._offsets) > 0:
            angle = int(sum(self._angles) / len(self._angles))
            offset = int(sum(self._offsets) / len(self._offsets))

        img.draw_string(20, 210, "{}".format(offset), color=(255, 0, 0), scale=2)
        img.draw_string(260, 210, "{}".format(angle), color=(255, 0, 0), scale=2)

        for line in self._lines:
            img.draw_line(line.line(), color=(255, 0, 0))

        for rect in self._rects:
            img.draw_rectangle(rect.rect(), color=(0, 255, 0))


class Robot:
    BASE_SPEED = 50
    MIN_SPEED = 20
    MAX_SPEED = 80

    class Direction:
        NONE = 0
        FORWARD = 1
        BACKWARD = 2
        LEFT = 3
        RIGHT = 4
        MAX = 5

        def __init__(self, state=NONE):
            self._state = state

        @property
        def state(self):
            return self._state

        @state.setter
        def state(self, value):
            if value < self.NONE or value >= self.MAX:
                raise ValueError("STATE")
            self._state = value


    def __init__(self):
        self._motors = Motors()
        self._camera = Camera()
        self._pid_angle = PidControl(max_corr=60, kp=2.0, ki=0.1, kd=1.0)
        self._pid_offset = PidControl(max_corr=60, kp=0.5, ki=0.1, kd=0.6)

        self._direction = Robot.Direction()
        self._direction_cooldown = 0
        self._calibrated = True

    def init(self):
        pass

    def stop(self):
        self._drive(0, 0)

    def calibrate(self):
        self._calibrated = True

    def get_direction(self):
        rl, rr = self._camera.get_num_rects_lr()
        if rr + rl >= 2:
            self._direction.state = Robot.Direction.BACKWARD
        elif rl == 1:
            self._direction.state = Robot.Direction.LEFT
        elif rr == 1:
            self._direction.state = Robot.Direction.RIGHT
        else:
            self._direction.state = Robot.Direction.FORWARD
        return self._direction

    def navigate_back(self):
        self._camera.image.draw_string(240, 210, "BACK", color=(0, 0, 0), scale=2)
        return True

    def navigate_left(self):
        self._camera.image.draw_string(240, 210, "LEFT", color=(0, 0, 0), scale=2)
        return True

    def navigate_right(self):
        self._camera.image.draw_string(240, 210, "RIGHT", color=(0, 0, 0), scale=2)
        return True

    def navigate_forward(self):
        angle, offset = self._camera.get_angle_and_offset()
        if angle is None or offset is None:
            return

        angle_corr = self._pid_angle.update(angle)
        offset_corr = self._pid_offset.update(offset)

        left_speed = self.BASE_SPEED + offset_corr + angle_corr
        right_speed = self.BASE_SPEED - offset_corr - angle_corr

        #print("LS:{}".format(left_speed))
        #print("RS:{}".format(right_speed))

        #self._motors.run(left_speed, right_speed)
        return True

    def navigate(self):

        self._camera.update() # get all raw data to capture only once

        if self.get_direction() == Robot.Direction.BACKWARD:
            return self.navigate_back()

        if self.get_direction() == Robot.Direction.LEFT:
            return self.navigate_left()

        if self.get_direction() == Robot.Direction.RIGHT:
            return self.navigate_right()

        return self.navigate_forward()


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
