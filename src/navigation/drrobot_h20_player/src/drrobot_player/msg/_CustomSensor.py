"""autogenerated by genmsg_py from CustomSensor.msg. Do not edit."""
import roslib.message
import struct

import roslib.msg

class CustomSensor(roslib.message.Message):
  _md5sum = "99688dde61b19bf7f23d2791779ffdd3"
  _type = "drrobot_player/CustomSensor"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """# custom Sensor information message from DrRobot Robot.

Header header    	# timestamp in the header is the time the driver
		 	# returned the battery/power reading
string robot_type	# robot type, I90 series, sentinel3 or Hawk/H20 Power/Motion

#make sure what sensor is on your expanded sensor port
uint32[] customADData		# custom AD channel 8 channel on the board, which channel is avaiable please contact Dr Robot
uint8 customIO    #expanded IO

================================================================================
MSG: roslib/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

"""
  __slots__ = ['header','robot_type','customADData','customIO']
  _slot_types = ['Header','string','uint32[]','uint8']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       header,robot_type,customADData,customIO
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(CustomSensor, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = roslib.msg._Header.Header()
      if self.robot_type is None:
        self.robot_type = ''
      if self.customADData is None:
        self.customADData = []
      if self.customIO is None:
        self.customIO = 0
    else:
      self.header = roslib.msg._Header.Header()
      self.robot_type = ''
      self.customADData = []
      self.customIO = 0

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    @param buff: buffer
    @type  buff: StringIO
    """
    try:
      _x = self
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.robot_type
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      length = len(self.customADData)
      buff.write(_struct_I.pack(length))
      pattern = '<%sI'%length
      buff.write(struct.pack(pattern, *self.customADData))
      buff.write(_struct_B.pack(self.customIO))
    except struct.error, se: self._check_types(se)
    except TypeError, te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    @param str: byte array of serialized message
    @type  str: str
    """
    try:
      if self.header is None:
        self.header = roslib.msg._Header.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.robot_type = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sI'%length
      start = end
      end += struct.calcsize(pattern)
      self.customADData = struct.unpack(pattern, str[start:end])
      start = end
      end += 1
      (self.customIO,) = _struct_B.unpack(str[start:end])
      return self
    except struct.error, e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    @param buff: buffer
    @type  buff: StringIO
    @param numpy: numpy python module
    @type  numpy module
    """
    try:
      _x = self
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.robot_type
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      length = len(self.customADData)
      buff.write(_struct_I.pack(length))
      pattern = '<%sI'%length
      buff.write(self.customADData.tostring())
      buff.write(_struct_B.pack(self.customIO))
    except struct.error, se: self._check_types(se)
    except TypeError, te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    @param str: byte array of serialized message
    @type  str: str
    @param numpy: numpy python module
    @type  numpy: module
    """
    try:
      if self.header is None:
        self.header = roslib.msg._Header.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.robot_type = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sI'%length
      start = end
      end += struct.calcsize(pattern)
      self.customADData = numpy.frombuffer(str[start:end], dtype=numpy.uint32, count=length)
      start = end
      end += 1
      (self.customIO,) = _struct_B.unpack(str[start:end])
      return self
    except struct.error, e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_3I = struct.Struct("<3I")
_struct_B = struct.Struct("<B")