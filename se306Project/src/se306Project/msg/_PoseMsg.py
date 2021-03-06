"""autogenerated by genpy from se306Project/PoseMsg.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg
import std_msgs.msg

class PoseMsg(genpy.Message):
  _md5sum = "6e8ef06f4a76cd2ca058c153a18cd476"
  _type = "se306Project/PoseMsg"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """# A special position message which defines who the sender is
# String sender: The type of robot sending the message
#		Possible Values:
#			farmer
#			sheepdog
#			sheep
#			grass
#			truck
# Pose position: Position of the sender.
#		Pose2D messages are composed as:
#			float64 x
#			float64 y
#			float64 theta

std_msgs/String sender

geometry_msgs/Pose2D position

================================================================================
MSG: std_msgs/String
string data

================================================================================
MSG: geometry_msgs/Pose2D
# This expresses a position and orientation on a 2D manifold.

float64 x
float64 y
float64 theta
"""
  __slots__ = ['sender','position']
  _slot_types = ['std_msgs/String','geometry_msgs/Pose2D']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       sender,position

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(PoseMsg, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.sender is None:
        self.sender = std_msgs.msg.String()
      if self.position is None:
        self.position = geometry_msgs.msg.Pose2D()
    else:
      self.sender = std_msgs.msg.String()
      self.position = geometry_msgs.msg.Pose2D()

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
      _x = self.sender.data
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_3d.pack(_x.position.x, _x.position.y, _x.position.theta))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.sender is None:
        self.sender = std_msgs.msg.String()
      if self.position is None:
        self.position = geometry_msgs.msg.Pose2D()
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.sender.data = str[start:end].decode('utf-8')
      else:
        self.sender.data = str[start:end]
      _x = self
      start = end
      end += 24
      (_x.position.x, _x.position.y, _x.position.theta,) = _struct_3d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self.sender.data
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_3d.pack(_x.position.x, _x.position.y, _x.position.theta))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.sender is None:
        self.sender = std_msgs.msg.String()
      if self.position is None:
        self.position = geometry_msgs.msg.Pose2D()
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.sender.data = str[start:end].decode('utf-8')
      else:
        self.sender.data = str[start:end]
      _x = self
      start = end
      end += 24
      (_x.position.x, _x.position.y, _x.position.theta,) = _struct_3d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_3d = struct.Struct("<3d")
