from pybricks.hubs import TechnicHub
from pybricks.pupdevices import Motor, Remote
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

hub = TechnicHub()

engine_right = Motor(Port.B, Direction.COUNTERCLOCKWISE)

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

sleep_ms = 20
normal_speed = 500
turbo = 2
while True:
    buttons_pressed = remote.buttons.pressed()

    speed = 0
    if Button.LEFT_PLUS in buttons_pressed:
        speed = normal_speed
    if Button.LEFT_MINUS in buttons_pressed:
        speed = -normal_speed

    if (Button.RIGHT in buttons_pressed) or (Button.LEFT in buttons_pressed):
        speed *= turbo
        hub.light.on(Color.MAGENTA)
    else:
        hub.light.on(Color.GREEN)

    engine_right.run(speed)

    # wait a bit
    wait(sleep_ms)


