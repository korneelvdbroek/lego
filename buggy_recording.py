from pybricks.pupdevices import Motor, Remote
from pybricks.hubs import TechnicHub
from pybricks.parameters import Port, Direction, Stop, Button, Color
from pybricks.tools import wait
from pybricks.geometry import Axis
from pybricks.tools import StopWatch
import micropython
import umath


class ControlButton:
  MIN_CLICK_MS = 30
  OFF_RESET_MS = 200
  LONG_PRESS_MS = 800

  def __init__(self, light_on_fn, control_button=Button.CENTER):
    self.control_button = control_button
    self.light_on_fn = light_on_fn

    self.center_button_on_1 = 0
    self.center_button_off_1 = 0
    self.center_button_on_2 = 0
    self.center_button_off_2 = 0

  def increase_counters(self, buttons_pressed, sleep_ms):
    if self.control_button in buttons_pressed:
      if (0 < self.center_button_on_1) and (
          self.MIN_CLICK_MS < self.center_button_off_1):
        self.center_button_on_2 += sleep_ms
        self.center_button_off_2 = 0
      else:
        self.center_button_on_1 += sleep_ms
        self.center_button_off_1 = 0
        self.center_button_on_2 = 0
        self.center_button_off_2 = 0
        if self.LONG_PRESS_MS < self.center_button_on_1:
          self.light_on_fn()
    else:
      if (self.MIN_CLICK_MS < self.center_button_on_1) and (
          self.center_button_off_1 < self.OFF_RESET_MS) and (
          self.center_button_on_2 == 0) and (
          self.center_button_off_2 == 0):
        self.center_button_off_1 += sleep_ms
        self.center_button_on_2 = 0
        self.center_button_off_2 = 0
      elif (0 < self.center_button_on_1) and (
          0 < self.center_button_off_1) and (
          self.MIN_CLICK_MS < self.center_button_on_2) and (
          self.center_button_off_2 < self.OFF_RESET_MS):
        self.center_button_off_2 += sleep_ms
      else:
        self.reset_counters()

  def reset_counters(self):
    self.center_button_on_1 = 0
    self.center_button_off_1 = 0
    self.center_button_on_2 = 0
    self.center_button_off_2 = 0

  def _reset(self, pressed):
    # print(f"{self.center_button_on_1:4d}, {self.center_button_off_1:4d}, {self.center_button_on_2:4d}, {self.center_button_off_2:4d}")

    if pressed:
      self.reset_counters()
    return pressed

  def long_pressed(self):
    pressed = (self.LONG_PRESS_MS < self.center_button_on_1) and (
        0 < self.center_button_off_1) and (
                  self.center_button_on_2 == 0) and (
                  self.center_button_off_2 == 0)

    return self._reset(pressed)

  def pressed(self):
    pressed = (0 < self.center_button_on_1) and (
        0 < self.center_button_off_1) and (
                  self.center_button_on_2 == 0) and (
                  self.center_button_off_2 == 0)

    return self._reset(pressed)

  def double_clicked(self):
    pressed = (0 < self.center_button_on_1) and (
        0 < self.center_button_off_1) and (
                  0 < self.center_button_on_2) and (
                  0 < self.center_button_off_2)

    return self._reset(pressed)


class RemoteButtons:
  RECORDING = 1
  REPLAYING = 2

  def __init__(self, callback_fn, init_fn):
    self.callback_fn = callback_fn
    self.init_fn = init_fn

    # Connect to the remote.
    self.remote = Remote()
    self.control_button = ControlButton(self.light_on_fn)

    self.mode = None
    self.stop_watch = StopWatch()
    self.recorded_sequence = []

    # state variables in REPLAYING mode
    self.replay_generator = None
    self.replay_buttons_current = tuple()
    self.replay_t, self.replay_buttons_new = (None, tuple())

  def light_on_fn(self):
    self.remote.light.on(Color.RED)

  def record(self, t, buttons_pressed):
    buttons_pressed_old = self.recorded_sequence[-1][1] if self.recorded_sequence else None
    if buttons_pressed_old is None or buttons_pressed != buttons_pressed_old:
      tuple_to_record = (t, buttons_pressed)
      print(tuple_to_record)
      self.recorded_sequence.append(tuple_to_record)

  def replay(self, t):
    while self.replay_t is not None and self.replay_t < t:
      self.replay_buttons_current = self.replay_buttons_new
      try:
        self.replay_t, self.replay_buttons_new = next(self.replay_generator)
      except StopIteration as e:
        self.stop_replaying()
        break

    return self.replay_buttons_current

  def get_buttons_pressed(self):
    t = self.stop_watch.time()
    buttons_pressed = self.remote.buttons.pressed()

    if self.mode == self.RECORDING:
      # only record if buttons pressed changed!
      buttons_pressed_filtered = tuple(b for b in buttons_pressed if b != self.control_button.control_button)
      self.record(t, buttons_pressed_filtered)

      return buttons_pressed
    elif self.mode == self.REPLAYING:
      if self.control_button.control_button in self.remote.buttons.pressed():
        return self.replay(t) + (self.control_button.control_button, )
      else:
        return self.replay(t)
    else:
      return buttons_pressed

  def start_recording(self):
    if self.mode == self.REPLAYING:
      self.stop_replaying()
    print(f"start recording...")

    self.mode = self.RECORDING
    self.recorded_sequence = []
    self.stop_watch.reset()
    self.stop_watch.resume()
    self.remote.light.on(Color.RED)

  def stop_recording(self):
    print(f"stop recording...")
    self.mode = None
    self.stop_watch.pause()
    self.remote.light.on(Color.BLUE)

  def start_replaying(self):
    if self.mode == self.RECORDING:
      self.stop_recording()
    print(f"start replaying...")

    self.mode = self.REPLAYING
    self.replay_generator = iter(self.recorded_sequence)
    self.replay_buttons_current = tuple()
    self.replay_t, self.replay_buttons_new = (0, tuple())
    self.stop_watch.reset()
    self.stop_watch.resume()
    self.remote.light.on(Color.MAGENTA)

  def stop_replaying(self):
    print(f"stop replaying...")
    self.mode = None
    self.replay_buttons_current = tuple()
    self.replay_t, self.replay_buttons_new = (None, tuple())
    self.stop_watch.pause()
    self.remote.light.on(Color.BLUE)

  def loop(self, sleep_ms=10):
    loop_watch = StopWatch()
    loop_watch.reset()

    self.mode = None
    t_minus1 = 0
    state = self.init_fn()
    while True:
      t = loop_watch.time()
      buttons_pressed = self.get_buttons_pressed()

      self.control_button.increase_counters(buttons_pressed, sleep_ms)

      if self.mode is None and self.control_button.long_pressed():
        self.start_recording()
      elif self.mode == RemoteButtons.RECORDING and self.control_button.pressed():
        self.stop_recording()
      elif self.mode is None and self.control_button.double_clicked():
        print(f"control button double click...")
        self.start_replaying()
      elif self.mode == RemoteButtons.REPLAYING and self.control_button.pressed():
        self.stop_replaying()

      # write-out what's happening
      state = self.callback_fn(t, t_minus1, buttons_pressed, state)

      # wait a bit
      t_execution = t - loop_watch.time()
      time_left_to_sleep_ms = sleep_ms - t_execution
      if time_left_to_sleep_ms < 5:
        print(f"warning: time left to sleep = {time_left_to_sleep_ms}")
      wait(time_left_to_sleep_ms)
      t_minus1 = t


class Buggy:
  def __init__(self):
    # Initialize the motors.
    self.engine_steer = Motor(Port.B)
    self.engine_drive = Motor(Port.A, Direction.COUNTERCLOCKWISE)

    # Lower the acceleration so the car starts and stops realistically.
    self.engine_drive.control.limits(acceleration=1000)

    # Initialize the hub
    self.hub = TechnicHub()
    self.hub.light.animate(colors=[Color.GREEN, Color.BLACK], interval=500)

    # Find the steering endpoint on the left and right.
    # The middle is in between.
    left_end = self.engine_steer.run_until_stalled(-200, then=Stop.HOLD)
    right_end = self.engine_steer.run_until_stalled(200, then=Stop.HOLD)
    # We are now at the right. Reset this angle to be half the difference.
    # That puts zero in the middle.
    self.engine_steer.reset_angle((right_end - left_end) / 2)
    self.steer_angle_max = self.engine_steer.angle()
    self.engine_steer.run_target(speed=200, target_angle=0, wait=True)

    # drive settings
    self.drive_speed = 1000
    self.steer_angle = 1.00 * self.steer_angle_max
    assert self.steer_angle <= self.steer_angle_max, f"chosen steer_angle ({self.steer_angle}) needs to be smaller than the steer_angle_max ({self.steer_angle_max})"

    self.hub.light.on(Color.GREEN)

  def dist2angle(self, distance_cm):
    angle = distance_cm / 29.7 * 360  # 30.7, 31.7
    return angle

  def deg2rad(self, angle):
    return angle / 360 * 2 * umath.pi

  def run_online_callback(self, t, t0, buttons_pressed, state):
    direction_angle_z, direction_target = state

    moving = umath.fabs(self.engine_drive.speed()) > 2  # int
    if moving:
      # only update direction_angle_z when driving!
      # integrate the angular velocity (errors...)
      v_z_angular = self.hub.imu.angular_velocity(Axis.Z)
      direction_angle_z += v_z_angular * (t - t0) / 1000.
    else:
      direction_angle_z = 0.
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
      self.engine_steer.run_target(speed=500, target_angle=+self.steer_angle, wait=False)
      direction_target = direction_angle_z - umath.copysign(1.0, drive_speed) * 25
    elif Button.RIGHT_MINUS in buttons_pressed:
      self.engine_steer.run_target(speed=500, target_angle=-self.steer_angle, wait=False)
      direction_target = direction_angle_z + umath.copysign(1.0, drive_speed) * 25
    elif moving:
      # counter-steer in case we do not maintain direction
      angle_deviation = direction_angle_z - direction_target
      max_angle_deviation = self.steer_angle
      angle_deviation = max(min(angle_deviation, +max_angle_deviation), -max_angle_deviation)
      angle_deviation = umath.copysign(1.0, drive_speed) * angle_deviation
      self.engine_steer.run_target(speed=500, target_angle=angle_deviation, wait=False)

    # Apply the selected speed.
    if drive_speed != 0:
      self.engine_drive.run(drive_speed)
    else:
      direction_target = direction_angle_z
      self.engine_drive.brake()

    state_new = (direction_angle_z, direction_target)
    return state_new

  def callback_init(self):
    direction_angle_z = 0
    direction_target = 0
    state_init = (direction_angle_z, direction_target)
    return state_init

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
    self.turn(-self.steer_angle)

  def right(self):
    self.turn(self.steer_angle)

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
    v_angular = None
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


buggy = Buggy()
remote_buttons = RemoteButtons(buggy.run_online_callback, buggy.callback_init)
remote_buttons.loop()
