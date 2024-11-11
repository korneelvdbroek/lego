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
        self.hub.light.animate(colors=[Color.BLUE, Color.BLACK], interval=250)
        try:
            self.remote = Remote()
        except Exception as e:
            # flash red: remote is missing
            wait(5 * 1000)
            raise e
        print(f" initialized...")

        # Calibrate steering
        print(f"Steering", end="")
        self.hub.light.animate(colors=[Color.CYAN, Color.BLACK], interval=250)
        self.engine_steer_angle_max_deg = self.calibrate_steering()
        print(f" initialized...")

        # Lower the acceleration so the car starts and stops realistically.
        self._engine_drive.control.limits(acceleration=1000)

        # buggy state data
        self.x = 0
        self.y = 0
        self.direction_angle = 0

        # buggy state
        self._task = None
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

            drive_forward = (x_target - self.x) * umath.cos(self.direction_angle) + (y_target - self.y) * umath.sin(
                self.direction_angle) >= 0

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
            external_state = await event_loop(self, self.remote.buttons.pressed(), external_state)
            await wait(20)

        # stop all tasks
        self._stop_action = True
        self._stop_loops = True
        print(f"Program is done")

    async def _action_loop(self):
        while not self._stop_loops:
            if self._task is not None:
                print(f"running task = {self._task}")
                await self._task
                self._task = None
            await wait(20)

    def create_task(self, task):
        if self._task is None:
            self._task = task
        else:
            print(f"Waiting for previous task to finish...")

    def stop(self):
        """Stop the current action"""
        self._stop_action = True

    def exit(self):
        """Exit the event loop"""
        self._stop_loops = True


class BuggyState:
    NOP = 0
    GO_HOME = 3
    GOING_HOME = 4


async def buggy_events(buggy, buttons_pressed, state):
    # determine next action
    if buggy.mode == buggy.READY:
        if state == BuggyState.GO_HOME:
            print(f"Return home!")
            x_target, y_target = (0, 0)
            buggy.hub.light.animate([Color.GREEN, Color.BLACK], interval=100)
            state = BuggyState.GOING_HOME

            # drive
            speed = 1000
            buggy.create_task(buggy.drive_xy(x_target, y_target, speed))

        elif state == BuggyState.GOING_HOME:
            wait_time = 2
            buggy.create_task(buggy.wait(wait_time * 1000))
            state = BuggyState.NOP

        elif urandom.uniform(0, 1) < 0.1:
            wait_time = urandom.randint(1, 10)
            print(f"Pause ({wait_time}s)")
            buggy.hub.light.on(Color.BLACK)
            buggy.create_task(buggy.wait(wait_time * 1000))

        elif urandom.uniform(0, 1) < 0.01 and not buggy.button_pressed:
            wait_time = urandom.randint(60, 120)
            print(f"Long pause ({wait_time}s)")
            buggy.hub.light.on(Color.BLACK)
            buggy.create_task(buggy.wait(wait_time * 1000))

        else:
            # pick new position outside no-go zone
            no_go_radius = buggy.buggy_length_cm / umath.tan(buggy.steer_angle_max_deg / 180 * umath.pi)
            while True:
                x_target_delta = urandom.uniform(-150, 150)
                y_target_delta = urandom.uniform(-150, 150)

                # check if it's in the no-go zone (turn is too sharp to get there...)
                # TODO: correct formula for non-zero self.direction_angle!
                if (x_target_delta + no_go_radius * umath.sin(buggy.direction_angle)) ** 2 + (
                        y_target_delta - no_go_radius * umath.cos(
                        buggy.direction_angle)) ** 2 > 1.05 * no_go_radius ** 2 and \
                        (x_target_delta - no_go_radius * umath.sin(buggy.direction_angle)) ** 2 + (
                        y_target_delta + no_go_radius * umath.cos(
                    buggy.direction_angle)) ** 2 > 1.05 * no_go_radius ** 2 and \
                        -150 < buggy.x + x_target_delta < 150 and -150 < buggy.y + y_target_delta < 150:
                    break

            x_target = buggy.x + x_target_delta
            y_target = buggy.y + y_target_delta

            colors = [Color.RED, Color.ORANGE, Color.YELLOW, Color.GREEN, Color.CYAN, Color.BLUE, Color.VIOLET,
                      Color.MAGENTA]
            buggy.hub.light.animate([colors[urandom.randint(0, len(colors) - 1)] for i in range(4)],
                                    interval=urandom.randint(1, 10) * 100)

            # select speed
            speed = 1000  # urandom.randint(500, 1000)
            if urandom.uniform(0, 1) < 0.1:
                speed = 500

            # drive
            print(f"Driving ({buggy.x:4.0f}, {buggy.y:4.0f}) => ({x_target:4.0f}, {y_target:4.0f})   speed = {speed}")
            buggy.create_task(buggy.drive_xy(x_target, y_target, speed))

    elif buggy.mode == buggy.CRASHED:
        print(f"Crashed => recovering")
        buggy.hub.light.animate([Color.RED, Color.BLACK], interval=200)
        buggy.create_task(buggy.wait(2 * 1000))

    # change state
    if state != BuggyState.GO_HOME and state != BuggyState.GOING_HOME and len(buttons_pressed) != 0:
        print(f"Key pressed => going home")
        state = BuggyState.GO_HOME
        buggy.stop()

    return state


buggy = Buggy()
buggy_state = BuggyState.NOP
buggy.run(buggy_events, buggy_state)
