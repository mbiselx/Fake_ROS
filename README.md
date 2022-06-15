### Warning

> O, that way madness lies; let me shun that;

--  Shakespeare, _King Lear_


Turn Back.

Installing ROS on windows [is possible](http://wiki.ros.org/Installation/Windows).
You should do that instead.


# Fake ROS

### what it isn't

ROS


### what it is

The idea of this Fake ROS mini-project was to allow me to test ROS code without
having to go through the hassle of installing ROS, because I'm lazy.
So I spent many, many hours recreating, debugging and testing this, to end up
with a tiny subset of ROS' abilities and no guarantee that any of it really works.

Fake ROS is a partial copy of the `rospy` API, which provides **some** of ROS'
functionalities, such as a time service, a publishing/subscriber service for a
few common message types and the beloved colorful ROS [WARNING], [ERROR] and
[FATAL] logs (though it doesn't actually log anything, it just prints to terminal).

ROS topics are handled by writing the messages to a in the directory `fake_ros_topics`,
and only reading it when the file has been modified. This means that high-speed topics
don't really work very well.
Similarly, due to jankiness, the queue size is 1.

Fake ROS _should_ run on Windows and on any Python 3 and Python 2.7. with minimal dependencies (the only necessary non-standard library import is `numpy`, I think).


## How to use it

It's not even a proper package (yet), so "installing" it consists of cloning the repo somewhere, running the `setup.py` script (which adds the repo to your Python path), and then :

```python
try :                   # import ROS if it exists
	import rospy
	import rospkg
	from   std_msgs.msg import String
except ImportError :    # else do something stupid
	import fake_ros.fake_rospy  as rospy   
    import fake_ros.fake_rospkg as rospkg
	from   fake_ros.fake_msgs   import String  
```

and _it just works_<sup>TM</sup>

![todd-howard-it-just-works](https://user-images.githubusercontent.com/62802642/173197648-489aeb10-f271-4425-810b-fc60b43abf39.gif)

At least... the standard ROS tutorial [talker and listener](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29)
should work, with like one line in the listener that needs to be changed, because I couldn't be bothered to do a C-style print for the loggers.


## supported messages
#### std_msgs :
- Float64
- Uint8
- Bool
- String
- Header

#### geometry_msgs :
- Vector3
- Quaternion
- Twist

#### sensor_msgs :
- Imu
- LaserScan

## support
No. you brought this upon yourself.
