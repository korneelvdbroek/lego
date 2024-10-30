from pybricks.hubs import TechnicHub
from pybricks.pupdevices import Motor, Remote
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

hub = TechnicHub()

print()
print("START:")

# Initialize the hub
hub = TechnicHub()
hub.light.animate(colors=[Color.GREEN, Color.BLACK], interval=500)

# scan for 2 motors
engine_right = engine_left = None
ports_it = iter([Port.A, Port.B, Port.C, Port.D])
while True:
    try:
        port = next(ports_it)
    except StopIteration:
        break

    try:
        engine_right = Motor(port, Direction.COUNTERCLOCKWISE)
        break
    except OSError as err:
        pass

while True:
    try:
        port = next(ports_it)
    except StopIteration:
        break

    try:
        engine_left = Motor(port, Direction.COUNTERCLOCKWISE)
        break
    except OSError as err:
        pass

print(f"right engine = {engine_right}")
print(f"left engine = {engine_left}")

if engine_left is None:
    engine_left = engine_right

if engine_right is None:
    # flash red: remote is missing
    hub.light.animate(colors=[Color.ORANGE, Color.BLACK], interval=250)
    wait(5 * 1000)
    raise SystemExit("Closing no engines found..")

# Lower the acceleration so the car starts and stops realistically.
engine_right.control.limits(acceleration=10000)
engine_left.control.limits(acceleration=10000)

# Connect to the remote.
try:
    remote = Remote()
except Exception as e:
    # flash red: remote is missing
    hub.light.animate(colors=[Color.RED, Color.BLACK], interval=250)
    wait(5 * 1000)
    raise e

hub.light.on(Color.GREEN)

sleep_ms = 50

speed_left = 0
speed_right = 0
speed_delta = 25

while True:
    buttons_pressed = remote.buttons.pressed()

    if Button.LEFT_PLUS in buttons_pressed:
        speed_left += speed_delta
        engine_left.run(speed_left)
    if Button.LEFT_MINUS in buttons_pressed:
        speed_left -= speed_delta
        engine_left.run(speed_left)
    if Button.LEFT in buttons_pressed:
        speed_left = 0
        engine_left.stop()

    if Button.RIGHT_PLUS in buttons_pressed:
        speed_right += speed_delta
        engine_right.run(speed_right)
    if Button.RIGHT_MINUS in buttons_pressed:
        speed_right -= speed_delta
        engine_right.run(speed_right)
    if Button.RIGHT in buttons_pressed:
        speed_right = 0
        engine_right.stop()

    # wait a bit
    wait(sleep_ms)


