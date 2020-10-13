#!/usr/bin/env python3
# type: ignore
import time
import math
import atexit
import numpy as np
import threading
import random
import cereal.messaging as messaging
import argparse
from common.params import Params
from common.realtime import Ratekeeper
from lib.helpers import FakeSteeringWheel

STEER_RATIO = 25.

parser = argparse.ArgumentParser(description='Bridge between CARLA and openpilot.')
parser.add_argument('--autopilot', action='store_true')
parser.add_argument('--joystick', action='store_true')
parser.add_argument('--realmonitoring', action='store_true')
# --long_test creates a vehicle in front in order to test longitudinal control
parser.add_argument('--long_test', action='store_true')
# --hil enables human control of the Carla vehicle by syncing dyno and carla speeds.
# Lateral control is managed by Carla's autopilot
parser.add_argument('--hil', action='store_true')

args = parser.parse_args()

#pm = messaging.PubMaster(['frame', 'sensorEvents', 'can'])
# We only want to send Carla frames to Openpilot
pm = messaging.PubMaster(['frame'])

W, H = 1164, 874

def steer_rate_limit(old, new):
  # Rate limiting to 0.5 degrees per step
  limit = 0.5
  if new > old + limit:
    return old + limit
  elif new < old - limit:
    return old - limit
  else:
    return new

def cam_callback(image):
  img = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
  img = np.reshape(img, (H, W, 4))
  img = img[:, :, [0, 1, 2]].copy()

  dat = messaging.new_message('frame')
  dat.frame = {
    "frameId": image.frame,
    "image": img.tostring(),
    "transform": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
  }
  pm.send('frame', dat)

def imu_callback(imu):
  #print(imu, imu.accelerometer)

  dat = messaging.new_message('sensorEvents', 2)
  dat.sensorEvents[0].sensor = 4
  dat.sensorEvents[0].type = 0x10
  dat.sensorEvents[0].init('acceleration')
  dat.sensorEvents[0].acceleration.v = [imu.accelerometer.x, imu.accelerometer.y, imu.accelerometer.z]
  # copied these numbers from locationd
  dat.sensorEvents[1].sensor = 5
  dat.sensorEvents[1].type = 0x10
  dat.sensorEvents[1].init('gyroUncalibrated')
  dat.sensorEvents[1].gyroUncalibrated.v = [imu.gyroscope.x, imu.gyroscope.y, imu.gyroscope.z]
  pm.send('sensorEvents', dat)

def health_function():
  pm = messaging.PubMaster(['health'])
  health_sock = messaging.pub_sock('health')
  rk = Ratekeeper(1.0)
  while 1:
    dat = messaging.new_message('health')

    dat.valid = True
    dat.health = {
      'ignitionLine': True,
      'ignition_can': True,
      'hwType': "greyPanda",
      'controlsAllowed': True
    }
    pm.send('health', dat)

    rk.keep_time()

def fake_driver_monitoring():
  if args.realmonitoring:
    return
  pm = messaging.PubMaster(['driverState'])
  while 1:
    dat = messaging.new_message('driverState')
    dat.driverState.faceProb = 1.0
    pm.send('driverState', dat)
    time.sleep(0.1)

def go():
  # health_function and fake_driver_monitoring are only needed if there is no Panda connected
  #threading.Thread(target=health_function).start()
  #threading.Thread(target=fake_driver_monitoring).start()

  client = carla.Client("127.0.0.1", 2000)
  client.set_timeout(5.0)
  world = client.load_world('Town04')
  settings = world.get_settings()
  settings.fixed_delta_seconds = 0.05
  world.apply_settings(settings)

  weather = carla.WeatherParameters(
      cloudiness=0.1,
      precipitation=0.0,
      precipitation_deposits=0.0,
      wind_intensity=0.0,
      sun_azimuth_angle=15.0,
      sun_altitude_angle=75.0)
  world.set_weather(weather)

  blueprint_library = world.get_blueprint_library()
  # for blueprint in blueprint_library.filter('sensor.*'):
  #    print(blueprint.id)
  # exit(0)

  world_map = world.get_map()
  vehicle_bp = random.choice(blueprint_library.filter('vehicle.tesla.model3'))
  vehicle = world.spawn_actor(vehicle_bp, world_map.get_spawn_points()[16]) #Point 283 is right in front for long control

  max_steer_angle = vehicle.get_physics_control().wheels[0].max_steer_angle
  # make tires less slippery
  # wheel_control = carla.WheelPhysicsControl(tire_friction=5)
  physics_control = vehicle.get_physics_control()
  physics_control.mass = 2326
  # physics_control.wheels = [wheel_control]*4
  physics_control.torque_curve = [[20.0, 500.0], [5000.0, 500.0]]
  physics_control.gear_switch_time = 0.0
  vehicle.apply_physics_control(physics_control)

  if args.long_test:
    tm = client.get_trafficmanager()
    tm_port = tm.get_port()

    vehicle_test_bp = random.choice(blueprint_library.filter('vehicle.tesla.model3'))
    vehicle_test = world.spawn_actor(vehicle_test_bp, world_map.get_spawn_points()[283])  # Point 283 is right in front for long control
    vehicle_test.apply_physics_control(physics_control)
    vehicle_test.set_autopilot(True, tm_port)
    tm.vehicle_percentage_speed_difference(vehicle_test, -10)

  if args.hil:
    if not args.long_test:
      tm = client.get_trafficmanager()
      tm_port = tm.get_port()

    vehicle.set_autopilot(True, tm_port)
    tm.ignore_lights_percentage(vehicle, 100)
    tm.distance_to_leading_vehicle(vehicle, 0)

  if args.autopilot:
    vehicle.set_autopilot(True)
  # print(vehicle.get_speed_limit())

  blueprint = blueprint_library.find('sensor.camera.rgb')
  blueprint.set_attribute('image_size_x', str(W))
  blueprint.set_attribute('image_size_y', str(H))
  blueprint.set_attribute('fov', '70')
  blueprint.set_attribute('sensor_tick', '0.05')
  transform = carla.Transform(carla.Location(x=0.8, z=1.45))
  camera = world.spawn_actor(blueprint, transform, attach_to=vehicle)
  camera.listen(cam_callback)

  # reenable IMU
  imu_bp = blueprint_library.find('sensor.other.imu')
  imu = world.spawn_actor(imu_bp, transform, attach_to=vehicle)
  #imu.listen(imu_callback)

  def destroy():
    print("clean exit")
    imu.destroy()
    camera.destroy()
    vehicle.destroy()
    print("done")
  atexit.register(destroy)

  # controls loop
  getcontrols = messaging.SubMaster(['carControl', 'carState','controlsState'])
  carla_state = messaging.PubMaster(['carlaState'])
  rk = Ratekeeper(100, print_delay_threshold=0.05)

  # init
  #A_throttle = 2.
  #A_brake = 2.
  A_steer_torque = 1.
  fake_wheel = FakeSteeringWheel()
  is_openpilot_engaged = False
  in_reverse = False

  throttle_out = 0
  brake_out = 0
  steer_out = steer_op = 0.0

  old_steer = steer_out

  vc = carla.VehicleControl(throttle=0, steer=0, brake=0, reverse=False)
  while 1:
    cruise_button = 0

    vel = vehicle.get_velocity()
    speed = math.sqrt(vel.x**2 + vel.y**2 + vel.z**2) * 3.6


    getcontrols.update(0)
    #print('sendcan update')
    # The angle of the Carla vehicle is sent to controlsd as the steering angle of the vehicle
    angle_carla = messaging.new_message('carlaState')
    angle_carla.carlaState = {"angle": steer_op}
    carla_state.send('carlaState', angle_carla)

    # Get controls from Openpilot
    throttle_op = getcontrols['carControl'].actuators.gas  # [0,1]
    brake_op = getcontrols['carControl'].actuators.brake  # [0,1]
    vel_dino = getcontrols['carState'].vEgo  #mps
    steer_op = getcontrols['controlsState'].angleSteersDes  # degrees [-180,180]
    #print("steer_op = {}".format(steer_out))
    steer_out = steer_op



    #steer_out = steer_rate_limit(old_steer, steer_out)

    old_steer = steer_out
    steer_carla = steer_out / (max_steer_angle * STEER_RATIO * -1)

    steer_carla = np.clip(steer_carla, -1, 1)
    #print("vel_dino = {}".format(vel_dino*2.2))

    # OP reads in meters per second
    # Carla reads in meters per second
    if abs(vel_dino - speed ) > 0.1:
      # Get the coordinates of a vector that points in the direction the vehicle is going and project the speed in that direction
      fwd = vehicle.get_transform().rotation.get_forward_vector()
      vehicle.set_velocity(carla.Vector3D(vel_dino * fwd.x,
                                          vel_dino * fwd.y, vel_dino * fwd.z))
    #vel = vehicle.get_velocity()
    #speed = math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)
    #print("carla_speed = {}".format(speed*2.2))
    #print("steer_carla = {}".format(steer_carla))
    throttle_out = throttle_op/0.6
    brake_out = brake_op
      #steer_angle_out = fake_wheel.angle

        # print(steer_torque_op)
      # print(steer_angle_out)
      #vc = carla.VehicleControl(throttle=throttle_out, steer=steer_angle_out / 3.14, brake=brake_out, reverse=in_reverse)
    vc.throttle = throttle_out
    #print('Throttle_Carla = {}'.format(throttle_out))
    if throttle_out != 0.0 or brake_out != 0.0:
      vc.steer = steer_carla
    else:
      vc.steer = 0
    vc.brake = brake_out

    # Openpilot controls are only applied if we're not running with human control
    if not args.hil:
      vehicle.apply_control(vc)
    elif args.hil:
      fwd = vehicle.get_transform().rotation.get_forward_vector()
      vehicle.set_velocity(carla.Vector3D(vel_dino * fwd.x,
                                          vel_dino * fwd.y, vel_dino * fwd.z))

    # Code below changes the speed of the vehicle in front to whatever value we want
    #fwd_test = vehicle_test.get_transform().rotation.get_forward_vector()
    #vel_test = vehicle_test.get_velocity()
    #speed_test = math.sqrt(vel_test.x ** 2 + vel_test.y ** 2 + vel_test.z ** 2) * 3.6
    # vehicle_test.set_velocity(carla.Vector3D(vel_dino * fwd_test.x,
     #                                          vel_dino * fwd_test.y, vel_dino * fwd_test.z))


    #print("speed_test = {}".format(speed_test*0.62))
    rk.keep_time()

if __name__ == "__main__":
  params = Params()
  params.delete("Offroad_ConnectivityNeeded")
  from selfdrive.version import terms_version, training_version
  params.put("HasAcceptedTerms", terms_version)
  params.put("CompletedTrainingVersion", training_version)
  params.put("CommunityFeaturesToggle", "1")
  params.put("CalibrationParams", '{"vanishing_point": [582.06, 442.78], "valid_blocks": 20, "calib_radians":[0, -0.0036804510179076896, -0.001153260986851604]}')


  # no carla, still run
  try:
    import carla
  except ImportError:
    print("WARNING: NO CARLA")
    while 1:
      time.sleep(1)

  from multiprocessing import Process, Queue
  #q = Queue()
  #p = Process(target=go)
  #p.daemon = True
  #p.start()
  go()

  # We don't want to control the Carla vehicle with the keyboard so the lines below are commented out
  #if args.joystick:
    # start input poll for joystick
  #  from lib.manual_ctrl import wheel_poll_thread
  #  wheel_poll_thread(q)
  #else:
    # start input poll for keyboard
   # from lib.keyboard_ctrl import keyboard_poll_thread
   # keyboard_poll_thread(q)


