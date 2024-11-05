from pybricks.pupdevices import Motor, Remote
from pybricks.hubs import TechnicHub
from pybricks.parameters import Port, Direction, Stop, Button, Color
from pybricks.tools import wait
from pybricks.geometry import Axis
from pybricks.tools import StopWatch
import umath


class Buggy:
    def __init__(self):
        # Initialize the hub
        self.hub = TechnicHub()
        self.hub.light.animate(colors=[Color.GREEN, Color.BLACK], interval=500)

        # Initialize the motors.
        try:
            self.engine_steer = Motor(Port.A)
            self.engine_drive = Motor(Port.B, Direction.COUNTERCLOCKWISE)
        except Exception as e:
            # flash red: remote is missing
            self.hub.light.animate(colors=[Color.MAGENTA, Color.BLACK], interval=250)
            wait(5 * 1000)
            raise e

        # Lower the acceleration so the car starts and stops realistically.
        self.engine_drive.control.limits(acceleration=1000)

        # Connect to the remote.
        try:
            self.remote = Remote()
        except Exception as e:
            # flash red: remote is missing
            self.hub.light.animate(colors=[Color.RED, Color.BLACK], interval=250)
            wait(5 * 1000)
            raise e

        # Find the steering endpoint on the left and right.
        # The middle is in between.
        print(self.engine_steer.load(), self.engine_steer.stalled())
        left_end = self.engine_steer.run_until_stalled(-500, then=Stop.BRAKE)  # , duty_limit=100)
        print(self.engine_steer.load(), self.engine_steer.stalled())
        print(f"left_end = {left_end}")
        self.engine_steer.stop()
        wait(500)
        print(self.engine_steer.load(), self.engine_steer.stalled())
        right_end = self.engine_steer.run_until_stalled(500, then=Stop.BRAKE)  # , duty_limit=100)
        print(self.engine_steer.load(), self.engine_steer.stalled())
        print(f"right_end = {right_end}")
        self.engine_steer.stop()

        # We are now at the right. Reset this angle to be half the difference.
        # That puts zero in the middle.
        self.engine_steer.reset_angle(right_end - (right_end + left_end) / 2)
        self.engine_steer_angle_max = 0.8 * self.engine_steer.angle()
        self.engine_steer.run_target(speed=200, target_angle=0, wait=True)
        self.engine_steer.stop()

        # drive settings
        self.drive_speed = 1000
        self.engine_steer_angle = 1.00 * self.engine_steer_angle_max
        self.wheel_steer_angle_max = 25
        print(f"engine_steer_angle = {self.engine_steer_angle}")
        print(f"engine_steer_angle_max = {self.engine_steer_angle_max}")
        assert self.engine_steer_angle <= self.engine_steer_angle_max, f"chosen steer_angle ({self.engine_steer_angle}) needs to be smaller than the engine_steer_angle_max ({self.engine_steer_angle_max})"

        self.hub.light.on(Color.GREEN)

    def dist2angle(self, distance_cm):
        angle = distance_cm / 29.7 * 360  # 30.7, 31.7
        return angle

    def deg2rad(self, angle):
        return angle / 360 * 2 * umath.pi

    def run_online(self):
        sleep = 20

        direction_angle_z = 0
        direction_target = direction_angle_z

        watch = StopWatch()
        watch.reset()
        t0 = 0
        while True:
            t = watch.time()

            buttons_pressed = self.remote.buttons.pressed()

            moving = umath.fabs(self.engine_drive.speed()) > 2  # int
            if moving:
                # only update direction_angle_z when driving!
                # integrate the angular velocity (errors...)
                v_z_angular = self.hub.imu.angular_velocity(Axis.Z)
                direction_angle_z += v_z_angular * (t - t0) / 1000
            else:
                direction_angle_z = 0
                direction_target = direction_angle_z

            # Choose the drive speed based on the left controls.
            drive_speed = 0
            if Button.LEFT_PLUS in buttons_pressed:
                drive_speed += 1000
            if Button.LEFT_MINUS in buttons_pressed:
                drive_speed -= 1000
            if Button.RIGHT in buttons_pressed:
                drive_speed /= 2

            if Button.RIGHT_PLUS in buttons_pressed:
                self.engine_steer.run_target(speed=500, target_angle=+self.engine_steer_angle, wait=False)
                direction_target = direction_angle_z - umath.copysign(1.0, drive_speed) * self.wheel_steer_angle_max
            elif Button.RIGHT_MINUS in buttons_pressed:
                self.engine_steer.run_target(speed=500, target_angle=-self.engine_steer_angle, wait=False)
                direction_target = direction_angle_z + umath.copysign(1.0, drive_speed) * self.wheel_steer_angle_max
            elif moving:
                # counter-steer in case we do not maintain direction
                angle_deviation = direction_angle_z - direction_target
                engine_angle_deviation = angle_deviation / self.wheel_steer_angle_max * self.engine_steer_angle_max / 3
                max_angle_deviation = self.engine_steer_angle
                engine_angle_deviation = max(min(engine_angle_deviation, +max_angle_deviation), -max_angle_deviation)
                engine_angle_deviation = umath.copysign(1.0, drive_speed) * engine_angle_deviation
                self.engine_steer.run_target(speed=500, target_angle=engine_angle_deviation, wait=False)

            # Apply the selected speed.
            if drive_speed != 0:
                self.engine_drive.run(drive_speed)
            else:
                direction_target = direction_angle_z
                self.engine_drive.brake()  # could also use .brake() here to stop more suddenly

            # wait a bit
            t_execution = t - watch.time()
            wait(sleep - t_execution)
            t0 = t

    def forward(self, distance_cm=30):
        drive_angle = self.dist2angle(distance_cm)
        self.engine_drive.run_angle(self.drive_speed, drive_angle, then=Stop.BRAKE, wait=False)
        print(f"start driving...")
        sleep = 20
        angle_z = 0
        watch = StopWatch()
        watch.reset()
        t0 = 0
        while not self.engine_drive.done():
            t = watch.time()
            v_angular = self.hub.imu.angular_velocity(Axis.Z)
            angle_z += v_angular * (t - t0) / 1000
            print(f"  {angle_z:6.2f}, {v_angular}, {t - t0}")
            self.engine_steer.run_target(speed=500, target_angle=angle_z, wait=False)
            # if angle_z > 3.:
            #     self.engine_steer.run_target(speed=500, target_angle=angle_z, wait=False)
            #     print(f"    correcting +")
            # elif angle_z < -3.:
            #     self.engine_steer.run_target(speed=500, target_angle=angle_z, wait=False)
            #     print(f"    correcting -")
            # else:
            #     self.engine_steer.run_target(speed=500, target_angle=0, wait=False)
            wait(sleep)
            t0 = t
        print(f"angle_z = {angle_z}")
        print(f"stop driving...")

    def backward(self, distance_cm):
        self.forward(-distance_cm)

    def left(self):
        self.turn(-self.engine_steer_angle)

    def right(self):
        self.turn(self.engine_steer_angle)

    def turn(self, steer_angle_engine):
        self.engine_steer.run_target(100, steer_angle_engine, wait=True)
        # 20 is the distance in cm between front and back wheels
        # 23 is the angle of the wheels when the steer_engine is at steer_angle
        distance_cm = umath.pi / 2 * 20 / umath.sin(self.deg2rad(28))
        drive_angle = self.dist2angle(distance_cm)

        print(f"start turning...")
        self.engine_drive.run(self.drive_speed)
        sleep = 10
        angle_z = 0
        watch = StopWatch()
        watch.reset()
        t0 = 0
        while angle_z < 180:
            t = watch.time()
            v_angular = self.hub.imu.angular_velocity(Axis.Z)
            print(f"  {v_angular}, {t - t0}")
            angle_z += v_angular * (t - t0) / 1000
            wait(sleep)
            t0 = t

        self.engine_drive.stop()
        while v_angular >= 1:
            t = watch.time()
            v_angular = self.hub.imu.angular_velocity(Axis.Z)
            print(f"  {v_angular}, {t - t0}, stop")
            angle_z += v_angular * (t - t0) / 1000
            wait(sleep)
            t0 = t

        print(f"z-angle = {angle_z}")
        print(f"stop turning...")

    def light(self, colors=None, interval=500):
        if colors is None:
            colors = [Color.BLUE, Color.BLACK]
        elif type(colors) is not list:
            colors = [colors]
        self.hub.light.animate(colors, interval)

    def stop(self, t=1):
        self.engine_steer.run_target(speed=200, target_angle=0, wait=False)
        self.engine_drive.brake()
        wait(time=t / 1000)


# hanna's
buggy = Buggy()
# buggy.left()
buggy.run_online()
# buggy.light([Color.RED, Color.ORANGE, Color.YELLOW, Color.GREEN, Color.BLUE, Color.MAGENTA],12)
# buggy.forward(30)
# wait(34*1000)
# buggy.light([Color.RED,Color.BLUE,Color.GREEN,Color.YELLOW])
# buggy.right()
# buggy.forward(111111)
# buggy.right()
# buggy.forward(4546)

# wait before we end
wait(2 * 1000)
