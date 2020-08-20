#!/usr/bin/env python3
import time
import zmq
import generated.demo.protoc_pb2 as proto

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
from lib.can import can_function, sendcan_function
from lib.helpers import FakeSteeringWheel
from selfdrive.car.honda.values import CruiseButtons

import cv2

W, H = 1164, 874

pm = messaging.PubMaster(['frame', 'sensorEvents', 'can'])
context = zmq.Context()
velocity = proto.VehicleVelocity()

def webcam_capture():
  eon_focal_length = 910.0  # pixels
  webcam_focal_length = -908.0 / 1.5  # pixels

  eon_intrinsics = np.array([
    [eon_focal_length, 0., 1164 / 2.],
    [0., eon_focal_length, 874 / 2.],
    [0., 0., 1.]])

  webcam_intrinsics = np.array([
    [-webcam_focal_length, 0., 1280 / 2 / 1.5],
    [0., -webcam_focal_length, 720 / 2 / 1.5],
    [0., 0., 1.]])


  trans_webcam_to_eon_rear = np.dot(eon_intrinsics, np.linalg.inv(webcam_intrinsics))
  videoCapture = cv2.VideoCapture(0)
  videoCapture.set(cv2.CAP_PROP_FRAME_WIDTH, 853)
  videoCapture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
  while 1:
    ret, img = videoCapture.read()

    if ret:
      img = cv2.warpPerspective(img, trans_webcam_to_eon_rear, (1164, 874), borderMode=cv2.BORDER_CONSTANT, borderValue=0)
      #cv2.imshow("preview", img)
      webcam_send(img)

  #cv2.destroyWindow("preview")

def webcam_send(frame):
  #H = frame.shape[0]
  #W = frame.shape[1]
  #img = np.frombuffer(frame, dtype=np.dtype("uint8"))
  #img = frame[:, :, [1, 2, 0]].copy()
  #img = np.reshape(frame, (H, W, 3))

  dat = messaging.new_message('frame')
  dat.frame = {
    "frameId": frame_global,
    "image": frame.tobytes(),
    "transform": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
  }
  pm.send('frame', dat)

def health_function():
  pm = messaging.PubMaster(['health'])
  rk = Ratekeeper(1.0)
  while 1:
    dat = messaging.new_message('health')
    dat.valid = True
    dat.health = {
      'ignitionLine': True,
      'hwType': "greyPanda",
      'controlsAllowed': True,
      #'gasInterceptorDetected': True
    }
    pm.send('health', dat)
    #rk.keep_time()

def fake_driver_monitoring():
  #if args.realmonitoring:
    #return
  pm = messaging.PubMaster(['driverState'])
  while 1:
    dat = messaging.new_message('driverState')
    dat.driverState.faceProb = 1.0
    pm.send('driverState', dat)
    time.sleep(0.1)

def image_processing(image):
    img = np.frombuffer(image.image_data, dtype=np.dtype("uint8"))
    img = np.reshape(img, (H, W, 4))
    img = img[:, :, [0, 1, 2]].copy()

    dat = messaging.new_message('frame')
    dat.frame = {
        "frameId": image.frame,
        "image": img.tostring(),
        "transform": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    }
    pm.send('frame', dat)

def imu_processing(imu):
  #print(imu, imu.accelerometer)

  dat = messaging.new_message('sensorEvents', 2)
  dat.sensorEvents[0].sensor = 4
  dat.sensorEvents[0].type = 0x10
  dat.sensorEvents[0].init('acceleration')
  dat.sensorEvents[0].acceleration.v = [imu.accel_x, imu.accel_y, imu.accel_z]
  # copied these numbers from locationd
  dat.sensorEvents[1].sensor = 5
  dat.sensorEvents[1].type = 0x10
  dat.sensorEvents[1].init('gyroUncalibrated')
  dat.sensorEvents[1].gyroUncalibrated.v = [imu.gyro_x, imu.gyro_y, imu.gyro_z]
  pm.send('sensorEvents', dat)

def create_socket(address):
    socket = context.socket(zmq.SUB)
    print('Binding sockets')
    socket.connect("tcp://192.168.1.15:"+address)
    socket.subscribe("")
    return socket

def get_imu():
  imu = proto.IMU()
  imu_socket = create_socket('5001')
  while True:
    message_imu = imu_socket.recv()
    imu.ParseFromString(message_imu)
    imu_processing(imu)

def get_velocity():

  vel_socket = create_socket('5002')
  while True:
    message_vel = vel_socket.recv()
    velocity.ParseFromString(message_vel)


def go(q):
  global frame_global
  frame_global = 0
  threading.Thread(target=health_function).start()
  threading.Thread(target=fake_driver_monitoring).start()
  threading.Thread(target=webcam_capture).start()
  threading.Thread(target=get_imu).start()
  threading.Thread(target=get_velocity).start()

  socket_VehicleControl = context.socket(zmq.PUB)
  socket_VehicleControl.bind("tcp://*:5003")

  #Initialize Protobuffers
 # protobuffers_recv = [distance = proto.RadarDistance(),
                     # imu = proto.IMU(),
                    #  velocity = proto.VehicleVelocity(),
                     # img = proto.Camera()]

  #distance = proto.RadarDistance()


  #img = proto.Camera()
  VehicleControl_msg = proto.VehicleControlValue()



  sendcan = messaging.sub_sock('sendcan')
  #rk = Ratekeeper(100, print_delay_threshold=0.5)
  rk = Ratekeeper(100, print_delay_threshold=1)
  # init
  A_throttle = 2.
  A_brake = 2.
  A_steer_torque = 1.
  fake_wheel = FakeSteeringWheel()
  is_openpilot_engaged = True
  in_reverse = False

  throttle_out = 0.5
  brake_out = 0
  steer_angle_out = 0

  while True:
      frame_global = rk.frame
      #  Wait for next message from CARLA
      #message_radar = sockets[0].recv()
      #distance.ParseFromString(message_radar)

      cruise_button = 0

      # check for a input message, this will not block
      if not q.empty():
        print("here")
        message = q.get()

        m = message.split('_')
        if m[0] == "steer":
          steer_angle_out = float(m[1])
          fake_wheel.set_angle(steer_angle_out)  # touching the wheel overrides fake wheel angle
          # print(" === steering overriden === ")
        if m[0] == "throttle":
          throttle_out = float(m[1]) / 100.
          if throttle_out > 0.3:
            cruise_button = CruiseButtons.CANCEL
            is_openpilot_engaged = False
        if m[0] == "brake":
          brake_out = float(m[1]) / 100.
          if brake_out > 0.3:
            cruise_button = CruiseButtons.CANCEL
            is_openpilot_engaged = False
        if m[0] == "reverse":
          in_reverse = not in_reverse
          cruise_button = CruiseButtons.CANCEL
          is_openpilot_engaged = False
        if m[0] == "cruise":
          if m[1] == "down":
            cruise_button = CruiseButtons.DECEL_SET
            is_openpilot_engaged = True
          if m[1] == "up":
            cruise_button = CruiseButtons.RES_ACCEL
            is_openpilot_engaged = True
          if m[1] == "cancel":
            cruise_button = CruiseButtons.CANCEL
            is_openpilot_engaged = False

      #message_img = sockets[3].recv()
      #img.ParseFromString(message_img)
      #image_processing(img)
      #cruise_button = CruiseButtons.RES_ACCEL
      #cruise_button = CruiseButtons.DECEL_SET
      speed = math.sqrt(velocity.vel_x ** 2 + velocity.vel_y ** 2 + velocity.vel_z ** 2) * 3.6
      #print(speed)
      can_function(pm, speed, fake_wheel.angle, rk.frame, cruise_button=cruise_button,
                   is_engaged=is_openpilot_engaged)

      if rk.frame % 1 == 0:  # 20Hz?
          throttle_op, brake_op, steer_torque_op = sendcan_function(sendcan)
          # print(" === torq, ",steer_torque_op, " ===")
          if is_openpilot_engaged:
              fake_wheel.response(steer_torque_op * A_steer_torque, speed)
              throttle_out = throttle_op * A_throttle
              brake_out = brake_op * A_brake
              steer_angle_out = fake_wheel.angle
              # print(steer_torque_op)
          # print(steer_angle_out)
              #print('throttle_op {}'.format(throttle_op))
              #print('brake_op {}'.format(brake_op))
              #print('steer_torque_op {}'.format(steer_torque_op))
              if (throttle_out > 0) or (brake_out > 0) or (steer_angle_out > 0):
                print('throttle_out {}'.format(throttle_out))
                print('brake_out {}'.format(brake_out))
                print('steer_angle_out {}'.format(steer_angle_out))

          VehicleControl_msg.throttle = throttle_out
          VehicleControl_msg.brake = brake_out
          VehicleControl_msg.steering_angle = steer_angle_out / 3.14

          socket_VehicleControl.send(VehicleControl_msg.SerializeToString())
      rk.keep_time()


if __name__ == "__main__":
    from multiprocessing import Process, Queue

    params = Params()
    params.delete("Offroad_ConnectivityNeeded")
    # from selfdrive.version import terms_version, training_version
    training_version = b"0.2.0"
    terms_version = b"2"

    params.put("HasAcceptedTerms", terms_version)
    params.put("CompletedTrainingVersion", training_version)
    params.put("CommunityFeaturesToggle", "1")
    params.put("CalibrationParams", '{"vanishing_point": [582.06, 442.78], "valid_blocks": 20}')



    q = Queue()
    p = Process(target=go, args=(q,))
    p.daemon = True
    p.start()

    from lib.keyboard_ctrl import keyboard_poll_thread
    keyboard_poll_thread(q)
