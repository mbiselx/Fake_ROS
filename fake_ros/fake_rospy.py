import os
import time
import atexit
import random
import threading
try :
	from termcolor import colored
	if os.name == 'nt' : os.system('color') # enable colors in the windows terminal
except :
	print("termcolor could not be found on your system")

	if os.name == 'nt' : os.system('color')

	COLORS = dict(list(zip(['grey', 'red', 'green', 'yellow', \
			                'blue', 'magenta', 'cyan', 'white'], \
			               list(range(30, 38)))))
	RESET = '\033[0m'

	def colored(text, color=None) :
		""" simplified termcolor.colored() """
		fmt_str = '\033[%dm%s'
		if color is not None:
			text = fmt_str % (COLORS[color], text)
		return text + RESET

_caller_id = ""
_is_init   = False
_is_ok     = True

class ROSInterruptException(Exception) :
	pass

class Publisher():
	def __init__(self, topic, msg_class, queue_size=None):
		self.msg_class = msg_class
		topic_path  = os.path.join(os.path.dirname(os.path.dirname(os.path.realpath(__file__))), "fake_ros_topics")
		# topic_path  = os.path.join(os.getcwd(), "fake_ros_topics")
		if not os.path.exists(topic_path) :
			os.makedirs(topic_path)
		if topic[0] == "/": topic = topic[1:] # sanitize topic
		self.topic  = os.path.join(topic_path, topic)

	def publish(self, msg):
		if not type(msg) == self.msg_class :
			msg = self.msg_class(msg)

		try :
			os.rename(self.topic, self.topic) # check if file is in use
		except :
			Rate(100).sleep()
		f = open(self.topic, "w")
		f.write(str(msg))
		f.close()


class Subscriber():
	def __init__(self, topic, msg_class, callback, queue_size=None):

		self.msg_class = msg_class
		self.callback  = callback

		topic_path  = os.path.join(os.path.dirname(os.path.dirname(os.path.realpath(__file__))), "fake_ros_topics")
		# topic_path = os.path.join(os.getcwd(), "fake_ros_topics")
		if not os.path.exists(topic_path) :
			os.makedirs(topic_path)
		if topic[0] == "/": topic = topic[1:] # sanitize topic
		self.topic = os.path.join(topic_path, topic)
		open(self.topic, "w").close()

		self._cached_stamp = os.stat(self.topic).st_mtime

		# okay, i know polling is ugly, but i can't come up with something better, so here we are
		self.thread = threading.Timer(.01, self.handler)
		self.thread.daemon = True
		self.thread.start()

	def handler(self):
		global _is_ok
		try :
			stamp = os.stat(self.topic).st_mtime
			if stamp != self._cached_stamp:
				self._cached_stamp = stamp
				# the topic file has been modified. read it and execute the callback
				self.callback(self.msg_class.read(self.topic))

			self.thread = threading.Timer(.01, self.handler)
			self.thread.daemon = True
			self.thread.start()
		except KeyboardInterrupt :
			_is_ok = False
		except Exception as e :
			_is_ok = False
			logfatal(str(e))


# node stuff
def init_node(name, anonymous=False) :
	global _caller_id
	global _is_init
	if anonymous :
		_caller_id = "%s_%s_%s"%(name, os.getpid(), int(time.time()*1000))
	else :
		_caller_id = name
	_is_init = True

def get_caller_id():
	return _caller_id

def spin() :
	global _is_ok
	rate = Rate(2)
	while _is_ok:
		try :
			rate.sleep()
		except KeyboardInterrupt :
			_is_ok = False
		except Exception as e:
			logerr(str(e))
			_is_ok = False

def is_initialized() : # TODO: maybe core.is_initialized() ?
	return _is_init

def is_shutdown():
	return not _is_ok

def on_shutdown(callback=None):
	atexit.register(callback)


# time stuff
class Rate():
	def __init__(self, rate):
		self.dt = 1/rate
	def sleep(self):
		global _is_ok
		try :
			time.sleep(self.dt)
		except KeyboardInterrupt :
			if _is_init :
				logdebug("keyboard interrupt, shutting down")
				_is_ok = False
			else :
				raise KeyboardInterrupt
		except Exception as e :
			_is_ok = False
			logerr(str(e))

class Time():
	def __init__(self, secs=0, nsecs=0):
		self.secs = int(secs)
		self.nsecs = int(nsecs)

	def __str__(self):
		return str(self.secs)+"."+str(self.nsecs).zfill(9)

	def to_sec(self):
		return self.secs+self.nsecs*1e-9
	def to_nsec(self):
		return int(self.secs*1e9+self.nsecs)

	def now(self=None):
		if not self :
			self = Time()
		self.secs = int(get_time())
		self.nsecs = int((get_time()-self.secs)*1e9)
		return self

	def from_sec(s):
		return Time(secs=int(s), nsecs=int((s-int(s))*1e9))

def get_time():
	return time.time()

def get_rostime():
    return Time.now()


# logging stuff
def logdebug(msg):
	print("[DEBUG] [" + str(get_rostime()) + "]: " + str(msg))

def loginfo(msg):
	print("[INFO] [" + str(get_rostime()) + "]: " + str(msg))

def logwarn(msg):
	print(colored("[WARNING] [" + str(get_rostime()) + "]: " + str(msg), color='yellow'))

def logerr(msg):
	print(colored("[ERROR] [" + str(get_rostime()) + "]: " + str(msg), color='red'))

def logfatal(msg):
	print(colored("[FATAL] [" + str(get_rostime()) + "]: " + str(msg), color='red'))
