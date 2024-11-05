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
    cm_for_1000_degrees = 153. / 1106. * 1000  # distance in cm when drive-engine rotates 1000 degrees
    steer_angle_max = 15  # max angle of front wheels
    buggy_length_cm = 15.5  # distance between front and back axel

    def __init__(self):
        print()
        print(f"Starting...")

        # Initialize the hub
        self.hub = TechnicHub()
        self.hub.light.animate(colors=[Color.GREEN, Color.BLACK], interval=500)

        # Initialize the motors.
        try:
            self._engine_drive = Motor(Port.A, Direction.COUNTERCLOCKWISE)
            self._engine_steer = Motor(Port.C)
        except Exception as e:
            # flash magenta: issue with motors
            self.hub.light.animate(colors=[Color.RED, Color.BLACK], interval=250)
            wait(2 * 1000)
            raise e
        print(f"Motors initialized...")

        # Connect to the remote.
        # try:
        #     self.remote = Remote()
        # except Exception as e:
        #     # flash red: remote is missing
        #     self.hub.light.animate(colors=[Color.ORANGE, Color.BLACK], interval=250)
        #     wait(5 * 1000)
        #     raise e
        # print(f"Remote control initialized...")

        self.engine_steer_angle_max = self.calibrate_steering()
        print(f"Steering initialized...")

        # Lower the acceleration so the car starts and stops realistically.
        self._engine_drive.control.limits(acceleration=1000)

        # position-data
        self._program_stop = False
        self.x = 0
        self.y = 0
        self.direction_angle = 0

        # initialization finished
        self.hub.light.on(Color.GREEN)

    def calibrate_steering(self):
        # Find the steering endpoint on the left and right.
        # The middle is in between.
        left_end = self._engine_steer.run_until_stalled(-500, then=Stop.BRAKE)
        wait(500)
        right_end = self._engine_steer.run_until_stalled(500, then=Stop.BRAKE)

        # We are now at the right. Reset this angle to be half the difference.
        # That puts zero in the middle.
        self._engine_steer.reset_angle(right_end - (right_end + left_end) / 2)
        engine_steer_angle_max = 0.9 * self._engine_steer.angle()

        # reset steer to neutral position
        self._engine_steer.run_target(speed=200, target_angle=0, wait=True)

        return engine_steer_angle_max

    async def track_task(self):
        self._program_stop = False
        drive_angle_old = self._engine_drive.angle()
        while not self._program_stop:
            # drive angle
            drive_angle = self._engine_drive.angle()
            drive_angle_delta = drive_angle - drive_angle_old

            # distance
            distance_delta = self.drive_angle_to_cm(drive_angle_delta)

            # angle
            self.direction_angle = self.hub.imu.heading()

            # position
            self.x += distance_delta * umath.cos(self.direction_angle / 180 * umath.pi)
            self.y += distance_delta * umath.sin(self.direction_angle / 180 * umath.pi)

            print(f"(x, y) = ({self.x}, {self.y})   (angle = {self.direction_angle})")
            # crash detection
            # print(f"  (a_x, a_y) = ({self.hub.imu.acceleration(Axis.X)}, {self.hub.imu.acceleration(Axis.Y)})")
            await wait(10)

            #
            drive_angle_old = drive_angle

    async def drive(self, speed, duration=1):
        self._engine_drive.run(speed)
        await wait(duration * 1000.)

    async def drive_angle(self, speed, angle):
        drive_angle_start = self._engine_drive.angle()
        self._engine_drive.run(speed)
        while self._engine_drive.angle() < drive_angle_start + angle and not self._engine_drive.stalled():
            await wait(20)
        self._engine_drive.stop()

    async def drive_distance(self, speed, distance_cm):
        await self.drive_angle(speed, self.cm_to_drive_angle(distance_cm))

    async def drive_xy(self, x_target, y_target, speed=500):
        if x_target >= self.x:
            self._engine_drive.run(speed)
        else:
            # drive backwards
            self._engine_drive.run(-speed)
        self._engine_drive.run(speed)

        accepted_error_cm = 20
        while (x_target - self.x) ** 2 + (y_target - self.y) ** 2 > accepted_error_cm**2:
            # see backup computation
            steer_angle = 180 / umath.pi * umath.atan2(self.buggy_length_cm * (y_target - self.y),
                                                       2 * ((x_target - self.x) ** 2 + (y_target - self.y) ** 2))

            steer_angle = min(max(steer_angle, -self.steer_angle_max), self.steer_angle_max)
            engine_steer_angle = steer_angle / self.steer_angle_max * self.engine_steer_angle_max
            # print(f"steer_angle = {steer_angle} (engine_steer_angle = {engine_steer_angle})")

            self._engine_steer.run_target(speed=500, target_angle=engine_steer_angle, wait=False)
            await wait(20)
        self._engine_drive.stop()
        self._engine_steer.stop()

    async def stop(self):
        self._engine_drive.stop()

    async def program_stop(self):
        self._program_stop = True

    def drive_angle_to_cm(self, angle):
        return angle / 1000 * self.cm_for_1000_degrees

    def cm_to_drive_angle(self, cm):
        return cm / self.cm_for_1000_degrees * 1000


buggy = Buggy()


async def buggy_program2():
    x_target = 0
    y_target = 0
    no_go_radius = buggy.buggy_length_cm / umath.tan(buggy.steer_angle_max)
    for i in range(10):

        while True:
            x_target_delta = urandom.uniform(-50, 50)
            y_target_delta = urandom.uniform(-50, 50)
            # check if it's in the no-go zone (turn is too sharp to get there...)
            if (x_target_delta)**2 + (y_target_delta - no_go_radius)**2 > 1.05 * no_go_radius and \
               (x_target_delta)**2 + (y_target_delta + no_go_radius)**2 > 1.05 * no_go_radius and \
               -100 < x_target + x_target_delta < 100 and -100 < y_target + y_target_delta < 100:
                break

        x_target += x_target_delta
        y_target += y_target_delta
        speed = urandom.randint(0, 1000)
        print(f"(x_target, y_target) = ({x_target}, {y_target})   speed = {speed}")
        await buggy.drive_xy(x_target, y_target, speed)

        if urandom.uniform(0, 1) < 0.1:
            await wait(urandom.randint(1, 4)*500)


async def buggy_program():
    # await buggy.drive_angle(500, 1000)
    # await wait(1000)
    # await buggy.drive_distance(200, 100)
    # await buggy.drive_distance(200, 100)
    await buggy.drive_xy(x_target=200, y_target=50)
    # await wait(1000)


async def buggy_program_wrapper():
    print()
    print(f"Starting program")
    await buggy_program2()
    await buggy.program_stop()
    print(f"Program is done")


async def main():
    await multitask(buggy.track_task(), buggy_program_wrapper())


run_task(main())
