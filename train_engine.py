from pybricks.hubs import TechnicHub
from pybricks.pupdevices import DCMotor, Remote
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

hub = TechnicHub()

train_engine = DCMotor(Port.A, Direction.CLOCKWISE)

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

speed = 0.
old_speed = 0.
color = Color.BLACK
button_dead_timer = None
while True:
    buttons_pressed = remote.buttons.pressed()

    if Button.LEFT in buttons_pressed:
        speed = 0.
        color = Color.BLACK
        train_engine.stop()
    elif Button.RIGHT in buttons_pressed:
        speed = 0.
        color = Color.BLACK
        train_engine.brake()
    else:  
        if Button.LEFT_PLUS in buttons_pressed:
            speed = 30.
        if Button.LEFT_MINUS in buttons_pressed:
            speed = -30.
        if Button.RIGHT_PLUS in buttons_pressed and button_dead_timer is None:
            button_dead_timer = StopWatch()
            speed = min(100., speed + 10.)
        if Button.RIGHT_MINUS in buttons_pressed and button_dead_timer is None:
            button_dead_timer = StopWatch()
            speed = max(-100., speed - 10.)

        if button_dead_timer is not None and button_dead_timer.time() > 100:
            button_dead_timer = None

        if abs(speed) == 0.:
            color = Color.BLACK
        elif 0. < abs(speed) and abs(speed) <= 50.:
            color = Color.GREEN
        elif abs(speed) <= 80.:
            color = Color.ORANGE
        else: 
            color = Color.RED


        if old_speed != speed:
            print(speed)
            train_engine.dc(speed)
            old_speed = speed

    remote.light.on(color)

    # wait a bit
    wait(sleep_ms)


