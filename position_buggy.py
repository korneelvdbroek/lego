from pybricks.pupdevices import Motor, Remote
from pybricks.hubs import TechnicHub
from pybricks.parameters import Port, Direction, Stop, Button, Color
from pybricks.tools import wait
from pybricks.tools import multitask, run_task
from pybricks.geometry import Axis
from pybricks.tools import StopWatch
import umath
import urandom


class Buggy:
    # buggy characteristics
    # TODO: write method which does these tests and prints output
    cm_for_1000_degrees = 153. / 1106. * 1000  # distance in cm when drive-engine rotates 1000 degrees
    steer_angle_max_deg = 22  # max angle of front wheels
    buggy_length_cm = 15.5  # distance between front and back axel

    INIT = 0
    READY = 1
    DRIVE = 2
    SLEEP = 3
    CRASHED = 4

    def __init__(self):
        self.mode = self.INIT
        print()
        print(f"Starting...")

        # Initialize the hub
        self.hub = TechnicHub()

        # Initialize the motors.
        print(f"Motors", end="")
        self.hub.light.animate(colors=[Color.RED, Color.BLACK], interval=250)
        try:
            self._engine_drive = Motor(Port.A, Direction.COUNTERCLOCKWISE)
            self._engine_steer = Motor(Port.C)
        except Exception as e:
            # flash magenta: issue with motors
            wait(2 * 1000)
            raise e
        print(f" initialized...")

        # Connect to the remote.
        print(f"Remote control", end="")
        self.hub.light.animate(colors=[Color.CYAN, Color.BLACK], interval=250)
        try:
            self.remote = Remote()
        except Exception as e:
            # flash red: remote is missing
            wait(5 * 1000)
            raise e
        print(f" initialized...")

        # Calibrate steering
        print(f"Steering", end="")
        self.hub.light.animate(colors=[Color.MAGENTA, Color.BLACK], interval=250)
        self.engine_steer_angle_max_deg = self.calibrate_steering()
        print(f" initialized...")

        # Lower the acceleration so the car starts and stops realistically.
        self._engine_drive.control.limits(acceleration=1000)

        # buggy state data
        self.x = 0
        self.y = 0
        self.direction_angle = 0

        # buggy state
        self._action = None
        self._stop_loops = False
        self._stop_loops = False
        self._stop_action = False

        # remote button data
        self.button_pressed = False

        # initialization finished
        self.mode = self.READY
        self.hub.light.on(Color.GREEN)

    def calibrate_steering(self):
        # Find the steering endpoint on the left and right.
        # The middle is in between.
        left_end = self._engine_steer.run_until_stalled(-500, then=Stop.HOLD)
        wait(500)
        right_end = self._engine_steer.run_until_stalled(500, then=Stop.HOLD)

        # We are now at the right. Reset this angle to be half the difference.
        # That puts zero in the middle.
        self._engine_steer.reset_angle(float(right_end) - (right_end + left_end) / 2)
        engine_steer_angle_max_deg = 0.8 * self._engine_steer.angle()

        # reset steer to neutral position
        self._engine_steer.run_target(speed=200, target_angle=0, wait=True)

        return engine_steer_angle_max_deg

    async def tracking_loop(self):
        self._stop_loops = False
        drive_angle_old = self._engine_drive.angle()
        while not self._stop_loops:
            # drive angle
            drive_angle = self._engine_drive.angle()
            drive_angle_delta = drive_angle - drive_angle_old

            # track: distance, angle, position
            distance_delta = self.drive_angle_to_cm(drive_angle_delta)
            self.direction_angle = self.hub.imu.heading() / 180 * umath.pi
            self.x += distance_delta * umath.cos(self.direction_angle)
            self.y += distance_delta * umath.sin(self.direction_angle)

            if self._engine_drive.stalled():
                self.mode = self.CRASHED

            #
            await wait(10)
            drive_angle_old = drive_angle

    async def drive_xy(self, x_target, y_target, speed=500):
        self._stop_action = False
        self.mode = self.DRIVE
        self._engine_drive.run(speed)

        accepted_error_cm = 20
        drive_forward_old = True
        while (x_target - self.x) ** 2 + (
                y_target - self.y) ** 2 > accepted_error_cm ** 2 and not self.mode == self.CRASHED and not self._stop_action:
            # see backup computation
            steer_angle = umath.atan2(2 * self.buggy_length_cm * (
                        (y_target - self.y) * umath.cos(self.direction_angle) - (x_target - self.x) * umath.sin(
                    self.direction_angle)),
                                      (x_target - self.x) ** 2 + (y_target - self.y) ** 2)

            drive_forward = self.is_forward(x_target - self.x, y_target - self.y)

            steer_angle_deg = 180 / umath.pi * steer_angle

            steer_angle_deg = min(max(steer_angle_deg, -self.steer_angle_max_deg), self.steer_angle_max_deg)
            engine_steer_angle_deg = steer_angle_deg / self.steer_angle_max_deg * self.engine_steer_angle_max_deg

            # send instructions to engine
            self._engine_steer.run_target(speed=500, target_angle=engine_steer_angle_deg, wait=False)
            if drive_forward != drive_forward_old:
                if drive_forward:
                    self._engine_drive.run(speed)
                else:
                    self._engine_drive.run(-speed)

            # print(f"steering:")
            # print(f"  drive_forward = {drive_forward}")
            # print(f"  direction_angle = {self.direction_angle * 180 / umath.pi}")
            # print(f"  (y_target - self.y) = {(y_target - self.y)}")
            # print(f"  cos(direction_angle) = {umath.cos(self.direction_angle)}")
            # print(f"  (x_target - self.x) = {(x_target - self.x)}")
            # print(f"  sin(direction_angle) = {umath.sin(self.direction_angle)}")
            # print(f"  {steer_angle} = atan({2 * self.buggy_length_cm * ((y_target - self.y) * umath.cos(self.direction_angle) - (x_target - self.x) * umath.sin(self.direction_angle))} / {(x_target - self.x) ** 2 + (y_target - self.y) ** 2})")
            # print(f"  steer_angle_deg = {steer_angle_deg:3.0f} (engine_steer_angle_deg = {engine_steer_angle_deg:3.0f})")

            # increment
            await wait(20)
            drive_forward_old = drive_forward

        self._engine_drive.stop()
        self._engine_steer.stop()

        # update mode
        if self.mode != self.CRASHED:
            self.mode = self.READY

        return

    def is_forward(self, delta_x, delta_y):
        """Returns True if car can reach point by driving forward"""
        return delta_x * umath.cos(self.direction_angle) + delta_y * umath.sin(self.direction_angle) >= 0

    def circle_center(self, x_start, y_start, x_end, y_end):
        D2 = (x_end - x_start)**2 + (y_end - y_start)**2

        # note: R can be negative!
        denominator = 2 * (y_end - y_start) * umath.cos(self.direction_angle) - 2 * (x_end - x_start) * umath.sin(self.direction_angle)
        if abs(denominator) > 1e-7:
            R = D2 / denominator
        else:
            R = 1e7

        xc = x_start - R * umath.sin(self.direction_angle)
        yc = y_start + R * umath.cos(self.direction_angle)

        return xc, yc

    def is_no_go(self, delta_x, delta_y):
        """Return True if car cannot reach point (since turn is too sharp to get there)"""
        no_go_radius = self.buggy_length_cm / umath.tan(self.steer_angle_max_deg / 180 * umath.pi)

        inside_no_go_left = (delta_x + no_go_radius * umath.sin(self.direction_angle)) ** 2 + (
                            delta_y - no_go_radius * umath.cos(self.direction_angle)) ** 2 < 1.05 * no_go_radius ** 2
        inside_no_go_right = (delta_x - no_go_radius * umath.sin(self.direction_angle)) ** 2 + (
                            delta_y + no_go_radius * umath.cos(self.direction_angle)) ** 2 < 1.05 * no_go_radius ** 2
        return inside_no_go_left or inside_no_go_right

    async def wait(self, time_ms):
        self._stop_action = False
        self.mode = self.SLEEP

        timer = StopWatch()
        while timer.time() < time_ms and not self._stop_action:
            await wait(20)

        self.mode = self.READY

    def drive_angle_to_cm(self, angle):
        return angle / 1000 * self.cm_for_1000_degrees

    def cm_to_drive_angle(self, cm):
        return cm / self.cm_for_1000_degrees * 1000

    async def steer(self, angle):
        self._engine_steer.run_target(speed=500, target_angle=angle, wait=False)

    def run(self, event_loop, external_state):
        run_task(self._run(event_loop, external_state))

    async def _run(self, event_loop, external_state):
        await multitask(self.tracking_loop(),
                        self._event_loop_wrapper(event_loop, external_state),
                        self._action_loop())

    async def _event_loop_wrapper(self, event_loop, external_state):
        print()
        print(f"Starting program")

        while not self._stop_loops:
            await event_loop(self.remote.buttons.pressed())
            await wait(20)

        # stop all tasks
        self._stop_action = True
        self._stop_loops = True
        print(f"Program is done")

    async def _action_loop(self):
        while not self._stop_loops:
            if self._action is not None:
                await self._action
                self._action = None
            await wait(20)

    def create_action(self, action):
        if self._action is None:
            self._action = action
        else:
            print(f"Waiting for previous task to finish...")

    def stop_action(self):
        """Stop the current action"""
        self._stop_action = True

    def exit(self):
        """Exit the event loop"""
        self._stop_loops = True


class Floorplan:
    def __init__(self, floorplan_str, dimension_cm=100):
        # validation:
        assert floorplan_str.count('-') == 1

        # convert to 2D array (we store as tuple of tuples, to enforce immutability)
        self.plan = tuple(tuple(row) for row in floorplan_str.split("\n"))[1:-1]
        self.plan = tuple(reversed(self.plan))

        self.height = len(self.plan)  # y-direction
        self.width = len(self.plan[0])  # x-direction
        self.dimension_cm = dimension_cm

        self.offset_x = None
        for y in range(self.height):
            for x in range(self.width):
                if self.plan[y][x] == '-':
                    self.offset_x = x
                    self.offset_y = y
                    break
            if self.offset_x is not None: break

    def coord2pos(self, x, y):
        pos_x = (int(x) + self.dimension_cm // 2) // self.dimension_cm
        pos_y = (int(y) + self.dimension_cm // 2) // self.dimension_cm
        return pos_x, pos_y

    def __str__(self):
        return self._str(self.plan)

    @staticmethod
    def _str(plan):
        return '\n'.join([''.join(row) for row in reversed(plan)])

    def __call__(self, x, y):
        pos_x, pos_y = self.coord2pos(x, y)
        return self.get_char(pos_x, pos_y)

    def get_char(self, pos_x, pos_y):
        pos_x += self.offset_x
        pos_y += self.offset_y
        if 0 <= pos_x < self.width and 0 <= pos_y < self.height:
            char = self.plan[pos_x][pos_y]
        else:
            char = 'X'

        return char

    def print_circle_path(self, x0, y0, x1, y1, xc, yc):
        plan = [list(row) for row in self.plan]

        path = self.circle_path(x0, y0, x1, y1, xc, yc)
        for (pos_x, pos_y) in path:
            if 0 <= pos_y + self.offset_y and pos_y + self.offset_y < self.height and 0 <= pos_x + self.offset_x and pos_x + self.offset_x < self.width:
                plan[pos_y + self.offset_y][pos_x + self.offset_x] = "*"

        print(self._str(plan))

    def is_circle_path_free(self, x0, y0, x1, y1, xc, yc):
        path = self.circle_path(x0, y0, x1, y1, xc, yc)
        for (pos_x, pos_y) in path:
            if self.get_char(pos_x, pos_y) == "X":
                return False
        return True

    def circle_path(self, x0, y0, x1, y1, xc, yc):
        """Returns True if circle path from (x0, y0) to (x1, y1)
        along the circle with center (xc, yc) and radius R
        is free of obstructions"""
        R = umath.sqrt((x0 - xc)**2 + (y0 - yc)**2)
        assert umath.sqrt((x1 - xc)**2 + (y1 - yc)**2) - R < 1e-2, f"Should be equal: {umath.sqrt((x1 - xc)**2 + (y1 - yc)**2)} = {R}"

        alpha_start = umath.atan2(y0 - yc, x0 - xc)  # -180 -> 180 deg
        alpha_end = umath.atan2(y1 - yc, x1 - xc)  # -180 -> 180 deg

        # compute delta_alpha  (-180 <= delta_alpha < 180)
        # and re-define alpha_end: (-360 <= alpha_end < 360)
        delta_alpha = alpha_end - alpha_start
        if delta_alpha < -umath.pi:
            delta_alpha += 2 * umath.pi
            alpha_end += 2 * umath.pi
        elif umath.pi <= delta_alpha:
            delta_alpha -= 2 * umath.pi
            alpha_end -= 2 * umath.pi
        sign_alpha = +1 if delta_alpha >= 0 else -1

        pos_x, pos_y = self.coord2pos(x0, y0)
        alpha = alpha_start
        path = []

        while (alpha < alpha_end and sign_alpha > 0) or (alpha_end < alpha and sign_alpha < 0):
            path += [(pos_x, pos_y)]

            # lines defining the current grid-square
            x_l = pos_x * self.dimension_cm - self.dimension_cm // 2  # left
            x_r = pos_x * self.dimension_cm + self.dimension_cm // 2  # right
            y_b = pos_y * self.dimension_cm - self.dimension_cm // 2  # bottom
            y_t = pos_y * self.dimension_cm + self.dimension_cm // 2  # top

            # angles of intersections between circle and grid-square lines
            alpha_l0 = umath.acos((x_l - xc) / R) if abs(x_l - xc) <= R else None  # 0 -> 180 deg
            alpha_l1 = -umath.acos((x_l - xc) / R) if abs(x_l - xc) <= R else None  # -0 -> -180 deg

            alpha_r0 = umath.acos((x_r - xc) / R) if abs(x_r - xc) <= R else None  # 0 -> 180 deg
            alpha_r1 = -umath.acos((x_r - xc) / R) if abs(x_r - xc) <= R else None  # -0 -> -180 deg

            alpha_b0 = umath.asin((y_b - yc) / R) if abs(y_b - yc) <= R else None  # -90 ->  90 deg
            alpha_b1 = umath.pi - umath.asin((y_b - yc) / R) if abs(y_b - yc) <= R else None  # 90 ->  270 deg

            alpha_t0 = umath.asin((y_t - yc) / R) if abs(y_t - yc) <= R else None  # -90 ->  90 deg
            alpha_t1 = umath.pi - umath.asin((y_t - yc) / R) if abs(y_t - yc) <= R else None  # 90 ->  270 deg

            alpha_steps =[alpha_l0, alpha_l1, alpha_r0, alpha_r1, alpha_b0, alpha_b1, alpha_t0, alpha_t1]
            alpha_steps = [(i, alpha_i - alpha) for i, alpha_i in enumerate(alpha_steps) if alpha_i is not None]

            # re-define angles:
            # *    0 <= alpha_step < 360 if sign_alpha > 0
            # * -360 <= alpha_step <   0 if sign_alpha < 0
            alpha_steps = [(i, phi - 2 * umath.pi * (phi // (2 * umath.pi))) for (i, phi) in alpha_steps]
            if sign_alpha < 0:
                alpha_steps = [(i, phi - 2 * umath.pi) for (i, phi) in alpha_steps]

            # pick smallest angle (but not zero)
            (direction, alpha_step) = min((t for t in alpha_steps if abs(t[1]) > 1e-5), key = lambda alpha_step: abs(alpha_step[1]))

            if direction == 0 or direction == 1:
                pos_x -= 1
            elif direction == 2 or direction == 3:
                pos_x += 1
            elif direction == 4 or direction == 5:
                pos_y -= 1
            elif direction == 6 or direction == 7:
                pos_y += 1

            # increment
            alpha += alpha_step
            # input()
        return path


class BuggyControl:
    NOP = 0
    GO_HOME = 3
    GOING_HOME = 4

    def __init__(self):
        self.buggy = Buggy()

#        y+
#        y+
#  x- x- -  x+ x+ >
#        y-
#        y-

        floorplan_str = """
XXXXXXXXXXXX
X.....XXX..X
X.....XXX..X
X..-.......X
X.....XXX..X
X.....XXX..X
XXXXXXXXXXXX
"""

        self.floorplan = Floorplan(floorplan_str, 50)
        self.state = BuggyControl.NOP

    async def buggy_events(self, buttons_pressed):
        # determine next action
        if self.buggy.mode == self.buggy.READY:
            if self.state == BuggyControl.GO_HOME:
                print(f"Returning home!")
                # TODO: what if this is in no-go zone?
                x_target, y_target = (0, 0)
                self.buggy.hub.light.animate([Color.GREEN, Color.BLACK], interval=100)
                self.state = BuggyControl.GOING_HOME

                # drive
                speed = 1000
                print(f"Driving ({self.buggy.x:4.0f}, {self.buggy.y:4.0f}) => ({x_target:4.0f}, {y_target:4.0f})   speed = {speed}")
                self.buggy.create_action(self.buggy.drive_xy(x_target, y_target, speed))

            elif self.state == BuggyControl.GOING_HOME:
                wait_time = 2
                self.buggy.create_action(self.buggy.wait(wait_time * 1000))
                self.state = BuggyControl.NOP

            elif urandom.uniform(0, 1) < 0.01:
                wait_time = urandom.randint(1, 10)
                print(f"Pause ({wait_time}s)")
                self.buggy.hub.light.on(Color.BLACK)
                self.buggy.create_action(self.buggy.wait(wait_time * 1000))

            elif urandom.uniform(0, 1) < 0.001 and not buttons_pressed:
                wait_time = urandom.randint(60, 120)
                print(f"Long pause ({wait_time}s)")
                self.buggy.hub.light.on(Color.BLACK)
                self.buggy.create_action(self.buggy.wait(wait_time * 1000))

            else:
                # pick new position outside no-go zone
                counter = 0
                while True:
                    x_target_delta = urandom.uniform(-150, 150)
                    y_target_delta = urandom.uniform(-150, 150)

                    x_target_new = self.buggy.x + x_target_delta
                    y_target_new = self.buggy.y + y_target_delta

                    xc, yc = self.buggy.circle_center(self.buggy.x, self.buggy.y, x_target_new, y_target_new)

                    # check if it's in the no-go zone (turn is too sharp to get there...)
                    if not self.buggy.is_no_go(x_target_delta, y_target_delta) and \
                            ((self.buggy.is_forward(x_target_delta, y_target_delta) and counter < 10) or (counter >= 10)) and \
                            self.floorplan.is_circle_path_free(self.buggy.x, self.buggy.y, x_target_new, y_target_new, xc, yc):
                        break

                    counter += 1

                x_target = x_target_new
                y_target = y_target_new

                self.floorplan.print_circle_path(self.buggy.x, self.buggy.y, x_target_new, y_target_new, xc, yc)

                colors = [Color.RED, Color.ORANGE, Color.YELLOW, Color.GREEN, Color.CYAN, Color.BLUE, Color.VIOLET,
                        Color.MAGENTA]
                self.buggy.hub.light.animate([colors[urandom.randint(0, len(colors) - 1)] for i in range(4)],
                                        interval=urandom.randint(1, 10) * 100)

                # select speed
                speed = 1000  # urandom.randint(500, 1000)
                if urandom.uniform(0, 1) < 0.01:
                    speed = 500

                # drive
                print(f"Driving ({self.buggy.x:4.0f}, {self.buggy.y:4.0f}) => ({x_target:4.0f}, {y_target:4.0f})   speed = {speed}")
                self.buggy.create_action(self.buggy.drive_xy(x_target, y_target, speed))

        elif self.buggy.mode == self.buggy.CRASHED:
            print(f"Crashed => recovering")
            self.buggy.hub.light.animate([Color.RED, Color.BLACK], interval=200)
            self.buggy.create_action(self.buggy.wait(2 * 1000))
            self.state = BuggyControl.GO_HOME
            self.buggy.stop_action()

        # change state
        if self.state != BuggyControl.GO_HOME and self.state != BuggyControl.GOING_HOME and len(buttons_pressed) != 0:
            print(f"Key pressed => going home")
            self.state = BuggyControl.GO_HOME
            self.buggy.stop_action()

    def run(self):
        self.buggy.run(self.buggy_events, self.state)



buggy = BuggyControl()
buggy.run()
