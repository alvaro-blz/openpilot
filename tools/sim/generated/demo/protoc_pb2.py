# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: demo.protoc

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='demo.protoc',
  package='float_send_proto',
  syntax='proto3',
  serialized_options=None,
  serialized_pb=_b('\n\x0b\x64\x65mo.protoc\x12\x10\x66loat_send_proto\"$\n\rRadarDistance\x12\x13\n\x0bradar_depth\x18\x01 \x01(\x02\"h\n\x03IMU\x12\x0f\n\x07\x61\x63\x63\x65l_x\x18\x01 \x01(\x02\x12\x0f\n\x07\x61\x63\x63\x65l_y\x18\x02 \x01(\x02\x12\x0f\n\x07\x61\x63\x63\x65l_z\x18\x03 \x01(\x02\x12\x0e\n\x06gyro_x\x18\x04 \x01(\x02\x12\x0e\n\x06gyro_y\x18\x05 \x01(\x02\x12\x0e\n\x06gyro_z\x18\x06 \x01(\x02\">\n\x0fVehicleVelocity\x12\r\n\x05vel_x\x18\x01 \x01(\x02\x12\r\n\x05vel_y\x18\x02 \x01(\x02\x12\r\n\x05vel_z\x18\x03 \x01(\x02\"J\n\x06\x43\x61mera\x12\r\n\x05width\x18\x01 \x01(\x05\x12\x0e\n\x06height\x18\x02 \x01(\x05\x12\x12\n\nimage_data\x18\x03 \x01(\x0c\x12\r\n\x05\x66rame\x18\x04 \x01(\x05\"N\n\x13VehicleControlValue\x12\x10\n\x08throttle\x18\x01 \x01(\x02\x12\r\n\x05\x62rake\x18\x02 \x01(\x02\x12\x16\n\x0esteering_angle\x18\x03 \x01(\x02\x62\x06proto3')
)




_RADARDISTANCE = _descriptor.Descriptor(
  name='RadarDistance',
  full_name='float_send_proto.RadarDistance',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='radar_depth', full_name='float_send_proto.RadarDistance.radar_depth', index=0,
      number=1, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=33,
  serialized_end=69,
)


_IMU = _descriptor.Descriptor(
  name='IMU',
  full_name='float_send_proto.IMU',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='accel_x', full_name='float_send_proto.IMU.accel_x', index=0,
      number=1, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='accel_y', full_name='float_send_proto.IMU.accel_y', index=1,
      number=2, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='accel_z', full_name='float_send_proto.IMU.accel_z', index=2,
      number=3, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='gyro_x', full_name='float_send_proto.IMU.gyro_x', index=3,
      number=4, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='gyro_y', full_name='float_send_proto.IMU.gyro_y', index=4,
      number=5, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='gyro_z', full_name='float_send_proto.IMU.gyro_z', index=5,
      number=6, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=71,
  serialized_end=175,
)


_VEHICLEVELOCITY = _descriptor.Descriptor(
  name='VehicleVelocity',
  full_name='float_send_proto.VehicleVelocity',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='vel_x', full_name='float_send_proto.VehicleVelocity.vel_x', index=0,
      number=1, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='vel_y', full_name='float_send_proto.VehicleVelocity.vel_y', index=1,
      number=2, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='vel_z', full_name='float_send_proto.VehicleVelocity.vel_z', index=2,
      number=3, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=177,
  serialized_end=239,
)


_CAMERA = _descriptor.Descriptor(
  name='Camera',
  full_name='float_send_proto.Camera',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='width', full_name='float_send_proto.Camera.width', index=0,
      number=1, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='height', full_name='float_send_proto.Camera.height', index=1,
      number=2, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='image_data', full_name='float_send_proto.Camera.image_data', index=2,
      number=3, type=12, cpp_type=9, label=1,
      has_default_value=False, default_value=_b(""),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='frame', full_name='float_send_proto.Camera.frame', index=3,
      number=4, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=241,
  serialized_end=315,
)


_VEHICLECONTROLVALUE = _descriptor.Descriptor(
  name='VehicleControlValue',
  full_name='float_send_proto.VehicleControlValue',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='throttle', full_name='float_send_proto.VehicleControlValue.throttle', index=0,
      number=1, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='brake', full_name='float_send_proto.VehicleControlValue.brake', index=1,
      number=2, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='steering_angle', full_name='float_send_proto.VehicleControlValue.steering_angle', index=2,
      number=3, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=317,
  serialized_end=395,
)

DESCRIPTOR.message_types_by_name['RadarDistance'] = _RADARDISTANCE
DESCRIPTOR.message_types_by_name['IMU'] = _IMU
DESCRIPTOR.message_types_by_name['VehicleVelocity'] = _VEHICLEVELOCITY
DESCRIPTOR.message_types_by_name['Camera'] = _CAMERA
DESCRIPTOR.message_types_by_name['VehicleControlValue'] = _VEHICLECONTROLVALUE
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

RadarDistance = _reflection.GeneratedProtocolMessageType('RadarDistance', (_message.Message,), dict(
  DESCRIPTOR = _RADARDISTANCE,
  __module__ = 'demo.protoc_pb2'
  # @@protoc_insertion_point(class_scope:float_send_proto.RadarDistance)
  ))
_sym_db.RegisterMessage(RadarDistance)

IMU = _reflection.GeneratedProtocolMessageType('IMU', (_message.Message,), dict(
  DESCRIPTOR = _IMU,
  __module__ = 'demo.protoc_pb2'
  # @@protoc_insertion_point(class_scope:float_send_proto.IMU)
  ))
_sym_db.RegisterMessage(IMU)

VehicleVelocity = _reflection.GeneratedProtocolMessageType('VehicleVelocity', (_message.Message,), dict(
  DESCRIPTOR = _VEHICLEVELOCITY,
  __module__ = 'demo.protoc_pb2'
  # @@protoc_insertion_point(class_scope:float_send_proto.VehicleVelocity)
  ))
_sym_db.RegisterMessage(VehicleVelocity)

Camera = _reflection.GeneratedProtocolMessageType('Camera', (_message.Message,), dict(
  DESCRIPTOR = _CAMERA,
  __module__ = 'demo.protoc_pb2'
  # @@protoc_insertion_point(class_scope:float_send_proto.Camera)
  ))
_sym_db.RegisterMessage(Camera)

VehicleControlValue = _reflection.GeneratedProtocolMessageType('VehicleControlValue', (_message.Message,), dict(
  DESCRIPTOR = _VEHICLECONTROLVALUE,
  __module__ = 'demo.protoc_pb2'
  # @@protoc_insertion_point(class_scope:float_send_proto.VehicleControlValue)
  ))
_sym_db.RegisterMessage(VehicleControlValue)


# @@protoc_insertion_point(module_scope)
