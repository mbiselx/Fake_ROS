import re
import os
import numpy as np
try : # import ROS if it exists
	import rospy
	has_ROS = True
except ImportError : # else do something stupid
	import fake_ros.fake_rospy as rospy   # NOTE : this only works on windows



def read_free_file(file) :
	while True :
		try :
			os.rename(file, file) # check if file is in use
			f = open(file, 'r')
			string = f.read()
			f.close()
			break
		except :
			rospy.Rate(100).sleep()
	return string


# AnyMsgs
class AnyMsg():
    """ a AnyMsgs template type """
    def __init__(self, data, converter=None):
        if not converter : converter = self.convert
        self.data = converter(data)
    def __str__(self):
        """
        This is used to print to the "ROS topic" file, or to the terminal
        """
        return str(self.data)
    def convert(val):
        """
        this should convert from a UTF-8 string to the python type
        """
        return val
    def read(file, msg_class=None):
        """
        This is used to read the "ROS topic" file
        """
        if not msg_class : msg_class = AnyMsg
        string = read_free_file(file)
        msg = msg_class(string, msg_class.convert)
        return msg


# AnyMsgs.msg.Float64
class Float64(AnyMsg):
    """docstring for Float64."""
    def __init__(self, data=0., converter=None):
        super(type(self), self).__init__(data, self.convert)
    def read(file) :
        return AnyMsg.read(file, Float64)
    def convert(self, val):
        return float(val)

# AnyMsgs.msg.UInt8
class Uint8(AnyMsg):
    """docstring for Uint8."""
    def __init__(self, data=0, converter=None):
        super(type(self), self).__init__(data, self.convert)
    def read(file) :
        return AnyMsg.read(file, Uint8)
    def convert(self, val):
        return int(val)

# AnyMsgs.msg.Bool
class Bool(AnyMsg):
    """docstring for Bool."""
    def __init__(self, data=True, converter=None):
        super(type(self), self).__init__(data, self.convert)
    def read(file) :
        return AnyMsg.read(file, Bool)
    def convert(self, val):
        return bool(val)

# AnyMsgs.msg.String
class String(AnyMsg):
    """docstring for String."""
    def __init__(self, data='', converter=None):
        super(type(self), self).__init__(data, self.convert)
    def read(file) :
        return AnyMsg.read(file, String)
    def convert(self, val):
        return str(val)


# AnyMsgs.msg.Header
class Header():
	def __init__(self, stamp=rospy.Time(), frame_id=''):
		self.stamp = stamp
		self.frame_id = frame_id
	def __str__(self):
		return "H : {}s, {}".format(str(self.stamp), self.frame_id)
	def read(string):
		"""
		this should convert from a UTF-8 string to the python object
		"""
		if os.sep in string : # we assume it is a filename
			string = read_free_file(string)

		try :
			stamp, frame_id = string.split(', ')
			stamp = float(re.findall(r"(\d*\.\d+|\d+)", stamp)[0])
			return Header(rospy.Time.from_sec(stamp), frame_id)
		except Exception as e:
			rospy.logerr("could not read message : " + str(e))
			return Header()



# other bs
class Vector3():
	def __init__(self, x=0, y=0, z=0):
		self.x = float(x)
		self.y = float(y)
		self.z = float(z)
	def __str__(self, sep=', ') :
		return sep.join([str(self.x), str(self.y), str(self.z)])
	def read(string) :
		if os.sep in string : # we assume it is a filename
			string = read_free_file(file)
		try :
			elems = re.findall(r"[-+]?(?:\d*\.\d+|\d+)", string)
			if len(elems) != 3 : raise Exception("Bad number of arguments given!")
			return Vector3(*elems)
		except Exception as e:
			rospy.logerr("could not read message : " + str(e))
			return Vector3()
class Quaternion(Vector3):
	def __init__(self, x=0, y=0, z=0, w=0):
		super(Quaternion, self).__init__(x,y,z)
		self.w = float(w)
	def __str__(self, sep=', '):  #printed as [w, x, y, z]
		out = super(Quaternion, self).__str__(sep)
		return sep.join([str(self.w), out])
	def read(string) :
		if os.sep in string : # we assume it is a filename
			string = read_free_file(string)
		try :
			elems = re.findall(r"[-+]?(?:\d*\.\d+|\d+)", string) #printed as [w, x, y, z]
			if len(elems) != 4 : raise Exception("Bad number of arguments given!")
			return Quaternion(elems[1], elems[2], elems[4], elems[0])
		except Exception as e:
			rospy.logerr("could not read message : " + str(e))
			return Quaternion()

# sensor_msgs.msg.Imu
class Imu():
	def __init__(self):
		self.header = Header()
		self.orientation = Quaternion()
		self.orientation_covariance = -np.ones(9)
		self.angular_velocity = Vector3()
		self.angular_velocity_covariance = -np.ones(9)
		self.linear_acceleration = Vector3()
		self.linear_acceleration_covariance = -np.ones(9)

	def __str__(self):
		out = str(self.header) + "\n"
		out = out + "O  : [" + str(self.orientation) + "]\n"
		out = out + "sO : "  + str(self.orientation_covariance.astype(float)) + "\n"
		out = out + "R  : [" + str(self.angular_velocity) + "]\n"
		out = out + "sR : "  + str(self.angular_velocity_covariance.astype(float)) + "\n"
		out = out + "A  : [" + str(self.linear_acceleration) + "]\n"
		out = out + "sA : "  + str(self.linear_acceleration_covariance.astype(float)) + "\n"
		return out

	def parse_cov(string) :
		return np.array(re.findall(r"[-+]?(?:\d*\.\d+|\d+)", string), dtype=float)

	def read(string):
		self = Imu()
		if os.sep in string : # we assume it is a filename
			string = read_free_file(string)

		try :
			lines = string.split('\n')
			self.header = Header.read(lines[0])
			self.orientation = Quaternion.read(lines[1])
			self.orientation_covariance = Imu.parse_cov(lines[2])
			self.angular_velocity = Vector3.read(lines[3])
			self.angular_velocity_covariance = Imu.parse_cov(lines[4])
			self.linear_acceleration = Vector3.read(lines[5])
			self.linear_acceleration_covariance = Imu.parse_cov(lines[6])

			return self
		except Exception as e:
			rospy.logerr("could not read message : " + str(e))
			return self

# sensor_msgs.msg.LaserScan
class LaserScan():
	def __init__(self):
		self.header = Header()

		self.range_min = 0.0
		self.range_max = float('inf')
		self.ranges    = list()
		self.intensities = list()

		self.angle_min = -float('inf')
		self.angle_max =  float('inf')
		self.angle_increment = 0

		self.time_increment = 0
		self.scan_time = 0

	def __str__(self):
		out  = str(self.header) + "\n"
		out += "t : {}, {}\n".format(self.time_increment, self.scan_time)
		out += "angles : [{} : {} : {}]\n".format(self.angle_min, self.angle_max, self.angle_increment)
		out += "range_info : [{} : {}]\n".format(self.range_min, self.range_max)
		out += "ranges : {}\n".format(self.ranges)
		out += "intensities : {}\n".format(self.intensities)

		return out

	def read(string):
		self = LaserScan()
		if os.sep in string : # we assume it is a filename
			string = read_free_file(string)

		try :
			lines = string.split('\n')
			self.header = Header.read(lines[0])
			self.time_increment, self.scan_time = re.findall(r"[-+]?(?:\d*\.\d+|\d+)", lines[1])
			self.angle_min, self.angle_max, self.angle_increment = re.findall(r"[-+]?(?:\d*\.\d+|\d+)", lines[2])
			self.range_min, self.range_max = re.findall(r"[-+]?(?:\d*\.\d+|\d+)", lines[3])
			self.ranges = re.findall(r"[-+]?(?:\d*\.\d+|\d+)", lines[4])
			self.intensities = re.findall(r"[-+]?(?:\d*\.\d+|\d+)", lines[5])

			return self
		except Exception as e:
			rospy.logerr("could not read message : " + str(e))
			return self


# nav_msgs.msg.Twist
class Twist():
	def __init__(self, x=0, y=0, z=0, rx=0, ry=0, rz=0):
		self.linear = Vector3(x, y, z)
		self.angular= Vector3(rx, ry, rz)

	def __str__(self):
		out = "Linear : [" + str(self.linear) + "]\n"
		out = out + "Angular : [" + str(self.angular) + "]\n"
		return out

	def read(string):
		if os.sep in string : # we assume it is a filename
			string = read_free_file(string)
		try :
			lines = string.split('\n')
			linear  = Vector3.read(lines[0])
			angular = Vector3.read(lines[1])
			return Twist(linear.x, linear.y, linear.z, angular.x, angular.y, angular.z)
		except Exception as e:
			rospy.logerr("could not read message : " + str(e))
			return self
