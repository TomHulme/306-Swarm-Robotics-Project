"""autogenerated by genpy from se306Project/GrassInfoMsg.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class GrassInfoMsg(genpy.Message):
  _md5sum = "91886dce45328a90758c81220163f8df"
  _type = "se306Project/GrassInfoMsg"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """int64 grassNum
int64 xPos
int64 yPos
int64 grassHeight

"""
  __slots__ = ['grassNum','xPos','yPos','grassHeight']
  _slot_types = ['int64','int64','int64','int64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       grassNum,xPos,yPos,grassHeight

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(GrassInfoMsg, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.grassNum is None:
        self.grassNum = 0
      if self.xPos is None:
        self.xPos = 0
      if self.yPos is None:
        self.yPos = 0
      if self.grassHeight is None:
        self.grassHeight = 0
    else:
      self.grassNum = 0
      self.xPos = 0
      self.yPos = 0
      self.grassHeight = 0

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
      _x = self
      buff.write(_struct_4q.pack(_x.grassNum, _x.xPos, _x.yPos, _x.grassHeight))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 32
      (_x.grassNum, _x.xPos, _x.yPos, _x.grassHeight,) = _struct_4q.unpack(str[start:end])
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
      _x = self
      buff.write(_struct_4q.pack(_x.grassNum, _x.xPos, _x.yPos, _x.grassHeight))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 32
      (_x.grassNum, _x.xPos, _x.yPos, _x.grassHeight,) = _struct_4q.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_4q = struct.Struct("<4q")
