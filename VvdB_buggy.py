from pybricks.pupdevices import Motor, Remote
from pybricks.hubs import TechnicHub
from pybricks.parameters import Port, Direction, Stop, Button, Color
from pybricks.tools import wait
from pybricks.geometry import Axis
from pybricks.tools import StopWatch
import umath


class RemoteButtons:
  RECORDING = 1
  REPLAYING = 2

  def __init__(self):
    # Connect to the remote.
    self.remote = Remote()

    self.mode = None
    self.recorded_sequence = []
    self.stop_watch = StopWatch()

  def pressed(self):
    if self.mode == self.RECORDING:
      buttons_pressed = self.remote.buttons.pressed()

      # only record if buttons pressed changed!
      buttons_pressed_old = self.recorded_sequence[-1][1] if self.recorded_sequence else None
      if buttons_pressed_old is None or buttons_pressed != buttons_pressed_old:
        tuple_to_record = (self.stop_watch.time(), buttons_pressed)
        print(tuple_to_record)
        self.recorded_sequence.append(tuple_to_record)

      return buttons_pressed
    elif self.mode == self.REPLAYING:
      while self.replay_t is not None and self.replay_t < self.stop_watch.time():
        self.replay_buttons_current = self.replay_buttons_new
        try:
          self.replay_t, self.replay_buttons_new = next(self.replay_generator)
        except StopIteration as e:
          self.stop_replaying()
          self.replay_t, self.replay_buttons_new = (None, None)

      return self.replay_buttons_current
    else:
      return self.remote.buttons.pressed()

  def start_recording(self):
    self.stop_replaying()

    self.mode = self.RECORDING
    self.recorded_sequence = []
    self.stop_watch.reset()
    self.stop_watch.resume()
    self.remote.light.on(Color.RED)

  def stop_recording(self):
    self.mode = None
    self.stop_watch.pause()
    self.remote.light.on(Color.BLUE)

  def start_replaying(self):
    self.stop_recording()

    self.mode = self.REPLAYING
    self.replay_generator = iter(self.recorded_sequence)
    self.replay_buttons_current = ()
    try:
      self.replay_t, self.replay_buttons_new = next(self.replay_generator)
    except StopIteration as e:
      self.stop_replaying()
      self.replay_t, self.replay_buttons_new = (None, None)
    self.stop_watch.reset()
    self.stop_watch.resume()
    self.remote.light.on(Color.MAGENTA)      

  def stop_replaying(self):
    self.mode = None
    self.stop_watch.pause()
    self.remote.light.on(Color.BLUE)


class Buggy:
  def __init__(self):
    # Initialize the motors.
    self.engine_steer = Motor(Port.B)
    self.engine_drive = Motor(Port.A, Direction.COUNTERCLOCKWISE)

    # Lower the acceleration so the car starts and stops realistically.
    # self.engine_drive.control.limits(acceleration=1000)

    # Initialize the hub
    self.hub = TechnicHub()
    self.hub.light.animate(colors=[Color.GREEN, Color.BLACK], interval=500)

    # Connect to the remote.
    try:
      self.remotebuttons = RemoteButtons()
    except Exception as e:
      # flash red: remote is missing
      self.hub.light.animate(colors=[Color.RED, Color.BLACK], interval=250)
      wait(5 * 1000)
      raise e

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

  def run_online(self):
    sleep = 20   # 20ms

    direction_angle_z = 0
    direction_target = direction_angle_z

    watch = StopWatch()
    watch.reset()
    t0 = 0
    while True:
      t = watch.time()

      moving = umath.fabs(self.engine_drive.speed()) > 2  # int
      if moving:
        # only update direction_angle_z when driving!
        # integrate the angular velocity (errors...)
        v_z_angular = self.hub.imu.angular_velocity(Axis.Z)
        direction_angle_z += v_z_angular * (t - t0) / 1000
      else:
        direction_angle_z = 0
        direction_target = direction_angle_z

      buttons_pressed = self.remotebuttons.pressed()

      # Choose the drive speed based on the left controls.
      drive_speed = 0
      if Button.LEFT_PLUS in buttons_pressed:
        drive_speed += 1000
      if Button.LEFT_MINUS in buttons_pressed:
        drive_speed -= 1000
      if Button.RIGHT in buttons_pressed:
        drive_speed /= 2

      natural_overshoot_angle = 30
      if Button.RIGHT_PLUS in buttons_pressed:
        self.engine_steer.run_target(speed=500, target_angle=+self.steer_angle, wait=False)
        direction_target = direction_angle_z - umath.copysign(1.0, drive_speed) * natural_overshoot_angle
      elif Button.RIGHT_MINUS in buttons_pressed:
        self.engine_steer.run_target(speed=500, target_angle=-self.steer_angle, wait=False)
        direction_target = direction_angle_z + umath.copysign(1.0, drive_speed) * natural_overshoot_angle
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


remote_buttons = RemoteButtons()
watch = StopWatch()
watch.reset()
sleep_ms = 20
center_button_on_1 = 0
center_button_off = 0
center_button_on_2 = 0
mode = None
replay_buttons_pressed_old = None
while True:
  t = watch.time()
  buttons_pressed = remote_buttons.pressed()

  if Button.CENTER in buttons_pressed:
    if 100 < center_button_off and center_button_off < 500:
      center_button_on_2 += sleep_ms
    else:  
      center_button_on_1 += sleep_ms
      center_button_off = 0
      center_button_on_2 = 0
  else:
    if 100 < center_button_on_1 and center_button_on_1 < 500:
      center_button_off += sleep_ms
      center_button_on_2 = 0
    else:
      center_button_on_1 = 0
      center_button_off = 0
      center_button_on_2 = 0

  if mode == None and 2000 < center_button_on_1:
    center_button_on_1 = 0
    center_button_off = 0
    center_button_on_2 = 0    

    print("start recording...")
    remote_buttons.start_recording()
  elif remote_buttons.mode == RemoteButtons.RECORDING and 500 < center_button_on_1:
    center_button_on_1 = 0
    center_button_off = 0
    center_button_on_2 = 0    

    print("stop recording...")
    mode = None
    remote_buttons.stop_recording()

  elif remote_buttons.mode == None and 200 < center_button_on_2:
    center_button_on_1 = 0
    center_button_off = 0
    center_button_on_2 = 0  

    print("start replaying...")
    remote_buttons.start_replaying()

  if remote_buttons.mode == RemoteButtons.REPLAYING:
    if replay_buttons_pressed_old != buttons_pressed:
      print(f"{t}: {buttons_pressed}")
      replay_buttons_pressed_old = buttons_pressed

  # wait a bit
  t_execution = t - watch.time()
  wait(sleep_ms - t_execution)

# # hanna's
# buggy = Buggy()
# # buggy.left()
# buggy.run_online()
# # buggy.light([Color.RED, Color.ORANGE, Color.YELLOW, Color.GREEN, Color.BLUE, Color.MAGENTA],12)
# # buggy.forward(30)
# # wait(34*1000)
# # buggy.light([Color.RED,Color.BLUE,Color.GREEN,Color.YELLOW])
# # buggy.right()
# # buggy.forward(111111)
# # buggy.right()
# # buggy.forward(4546)
# 
# # wait before we end 
# wait(2 * 1000)
