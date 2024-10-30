from pybricks.hubs import TechnicHub
from pybricks.pupdevices import DCMotor, Motor, Remote
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

hub = TechnicHub()

engine_right = Motor(Port.B, Direction.COUNTERCLOCKWISE)
train_engine = DCMotor(Port.A, Direction.CLOCKWISE)

# Lower the acceleration so the car starts and stops realistically.
engine_right.control.limits(acceleration=10000)


# Initialize the hub
hub = TechnicHub()
hub.light.animate(colors=[Color.GREEN, Color.BLACK], interval=500)

# Connect to the remote.
try:
    remote = Remote()
except Exception as e:
    # flash red: remote is missing
    hub.light.animate(colors=[Color.RED, Color.BLACK], interval=250)
    wait(5 * 1000)
    raise e

hub.light.on(Color.GREEN)

sleep_ms = 60

speed_left = 0
old_speed_left = 0.
speed_right = 0
speed_delta = 50
speed_delta_left = 5

while True:
    buttons_pressed = remote.buttons.pressed()

    if Button.LEFT_PLUS in buttons_pressed:
        speed_left += speed_delta_left
    if Button.LEFT_MINUS in buttons_pressed:
        speed_left -= speed_delta_left
    if Button.LEFT in buttons_pressed:
        speed_left = 0

    if Button.RIGHT_PLUS in buttons_pressed:
        speed_right += speed_delta
    if Button.RIGHT_MINUS in buttons_pressed:
        speed_right -= speed_delta
    if Button.RIGHT in buttons_pressed:
        speed_right = 0

    # engine_left.run(speed_left)
    if old_speed_left != speed_left:
        print(speed_left)
        train_engine.dc(speed_left)
        old_speed_left = speed_left
    engine_right.run(speed_right)

    # wait a bit
    wait(sleep_ms)


