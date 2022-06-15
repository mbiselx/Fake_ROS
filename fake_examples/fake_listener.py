#!/usr/bin/env python
# the shebang is ignored in ROS, so we can force the non-ROS environment

try : # import ROS if it exists
	import rospy
	from std_msgs.msg import String
except ImportError : # else do something stupid
	print("Sorry, I can't find ROS on your system\nSubstituiting fake garbage instead")
	import fake_ros.fake_rospy as rospy
	from fake_ros.fake_msgs import String


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + " : I heard : {}".format(data))

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("chatter", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
