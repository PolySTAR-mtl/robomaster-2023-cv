# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from detection/Detections.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import detection.msg

class Detections(genpy.Message):
  _md5sum = "5a80ac5cf722ceea32bf50e93318cacc"
  _type = "detection/Detections"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """# Detections.msg
## List of detected bounding boxes

# Header header
Detection[] detections

uint32 timelapse
================================================================================
MSG: detection/Detection
# Detection.msg
## Bounding box with class and confidence

# Constants

# TODO
# uint8 car
# uint8 armor_module
# ...

# Bounding box
float32 x
float32 y
float32 w
float32 h

# class
uint8 cls

float32 confidence"""
  __slots__ = ['detections','timelapse']
  _slot_types = ['detection/Detection[]','uint32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       detections,timelapse

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Detections, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.detections is None:
        self.detections = []
      if self.timelapse is None:
        self.timelapse = 0
    else:
      self.detections = []
      self.timelapse = 0

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      length = len(self.detections)
      buff.write(_struct_I.pack(length))
      for val1 in self.detections:
        _x = val1
        buff.write(_get_struct_4fBf().pack(_x.x, _x.y, _x.w, _x.h, _x.cls, _x.confidence))
      _x = self.timelapse
      buff.write(_get_struct_I().pack(_x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.detections is None:
        self.detections = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.detections = []
      for i in range(0, length):
        val1 = detection.msg.Detection()
        _x = val1
        start = end
        end += 21
        (_x.x, _x.y, _x.w, _x.h, _x.cls, _x.confidence,) = _get_struct_4fBf().unpack(str[start:end])
        self.detections.append(val1)
      start = end
      end += 4
      (self.timelapse,) = _get_struct_I().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      length = len(self.detections)
      buff.write(_struct_I.pack(length))
      for val1 in self.detections:
        _x = val1
        buff.write(_get_struct_4fBf().pack(_x.x, _x.y, _x.w, _x.h, _x.cls, _x.confidence))
      _x = self.timelapse
      buff.write(_get_struct_I().pack(_x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.detections is None:
        self.detections = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.detections = []
      for i in range(0, length):
        val1 = detection.msg.Detection()
        _x = val1
        start = end
        end += 21
        (_x.x, _x.y, _x.w, _x.h, _x.cls, _x.confidence,) = _get_struct_4fBf().unpack(str[start:end])
        self.detections.append(val1)
      start = end
      end += 4
      (self.timelapse,) = _get_struct_I().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_4fBf = None
def _get_struct_4fBf():
    global _struct_4fBf
    if _struct_4fBf is None:
        _struct_4fBf = struct.Struct("<4fBf")
    return _struct_4fBf
