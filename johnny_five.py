from pybricks.pupdevices import Motor, Remote
from pybricks.hubs import TechnicHub
from pybricks.parameters import Port, Direction, Stop, Button, Color
from pybricks.tools import wait
from pybricks.geometry import Axis
from pybricks.tools import StopWatch
import umath



engine_right = Motor(Port.A, Direction.COUNTERCLOCKWISE)
engine_left = Motor(Port.C, Direction.CLOCKWISE)

v_max, _, _ = engine_right.control.limits()

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


sleep_ms = 10

speed = 0.
speed_left_old = 0
speed_right_old = 0
color = Color.WHITE
button_dead_timer = None
button_dead_timer_ms = 200
turn_speed_delta = -1.00
speed_steps = 8
turn_right = False
turn_left = False
while True:
    buttons_pressed = remote.buttons.pressed()

    if Button.LEFT in buttons_pressed or Button.CENTER in buttons_pressed:
        speed = 0.
    else:
        if button_dead_timer is None:
            if Button.LEFT_PLUS in buttons_pressed:
                button_dead_timer = StopWatch()
                speed = min(v_max, speed + v_max / speed_steps)
            if Button.LEFT_MINUS in buttons_pressed:
                button_dead_timer = StopWatch()
                speed = max(-v_max, speed - v_max / speed_steps)
        else: 
            if button_dead_timer.time() > button_dead_timer_ms:
                # reset dead-time for button
                button_dead_timer = None

    # Turning
    speed_left = speed
    speed_right = speed
    if Button.RIGHT_PLUS in buttons_pressed:
        if turn_right == False and speed > 0.5 * v_max: 
            engine_left.hold()
            engine_right.hold()
            speed_left = 0
            speed_right = 0
        turn_right = True
        speed_right *= turn_speed_delta
    else:
        if turn_right == True and speed > 0.5 * v_max: 
            engine_left.hold()
            engine_right.hold()
            speed_left = 0
            speed_right = 0
        turn_right = False
    if Button.RIGHT_MINUS in buttons_pressed:
        if turn_left == False and speed > 0.5 * v_max: 
            engine_left.hold()
            engine_right.hold()
            speed_left = 0
            speed_right = 0
        turn_left = True
        speed_left *= turn_speed_delta
    else:
        if turn_left == True and speed > 0.5 * v_max: 
            engine_left.hold()
            engine_right.hold()
            speed_left = 0
            speed_right = 0
        turn_left = False

    # Decide light 
    if abs(speed) == 0.:
        color = Color.WHITE
    elif abs(speed) <= 0.50 * v_max:
        color = Color.GREEN
    elif abs(speed) <= 0.80 * v_max:
        color = Color.ORANGE
    else: 
        color = Color.RED
    hub.light.on(color)

    # update engine speed
    if speed_right_old != speed_right:
        print(f"{speed_left}, {speed_right}")
        engine_right.run(speed_right)
        speed_right_old = speed_right
    if speed_left_old != speed_left:
        print(f"{speed_left}, {speed_right}")
        engine_left.run(speed_left)
        speed_left_old = speed_left

    # wait a bit
    wait(sleep_ms)
