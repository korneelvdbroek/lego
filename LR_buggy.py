from pybricks.pupdevices import Motor, Remote
from pybricks.hubs import TechnicHub
from pybricks.parameters import Port, Direction, Stop, Button, Color
from pybricks.tools import wait
from pybricks.geometry import Axis
from pybricks.tools import StopWatch
import umath



engine_right = Motor(Port.A, Direction.COUNTERCLOCKWISE)
engine_left = Motor(Port.B, Direction.CLOCKWISE)

# Lower the acceleration so the car starts and stops realistically.
engine_right.control.limits(acceleration=10000)
engine_left.control.limits(acceleration=10000)

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


sleep = 20

while True:
    buttons_pressed = remote.buttons.pressed()

    # Choose the drive speed based on the left controls.
    speed_left = 0
    speed_right = 0
    normal_speed = 500
    turn_speed_delta = 0.50
    turbo = 2.00
    if Button.LEFT_PLUS in buttons_pressed:
        speed_left = normal_speed
        speed_right = normal_speed
    if Button.LEFT_MINUS in buttons_pressed:
        speed_left = -normal_speed
        speed_right = -normal_speed

    if Button.RIGHT_PLUS in buttons_pressed:
        speed_right *= turn_speed_delta
    if Button.RIGHT_MINUS in buttons_pressed:
        speed_left *= turn_speed_delta

    if (Button.RIGHT in buttons_pressed) or (Button.LEFT in buttons_pressed):
        speed_left *= turbo
        speed_right *= turbo
        hub.light.on(Color.MAGENTA)
    else:
        hub.light.on(Color.GREEN)

    engine_right.run(speed_right)
    engine_left.run(speed_left)

    # wait a bit
    wait(sleep)
