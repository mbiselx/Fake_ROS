#!/usr/bin/env python

try : # import ROS if it exists
	import rospy
	from std_msgs.msg import String
except ImportError : # else do something stupid
	print("Sorry, I can't find ROS on your system\nSubstituiting fake garbage instead")
	import fake_ros.fake_rospy as rospy
	from fake_ros.fake_msgs import String


def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        msg = "hell world"
        rospy.loginfo(str(msg))
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
