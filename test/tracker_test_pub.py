#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import PoseStamped

tracker_test_target = PoseStamped()
#tracker_test_target.pose.position.x = 0
#tracker_test_target.pose.position.y = 0
#tracker_test_target.pose.position.z = 0

def callback(data):
    global tracker_test_target
    tracker_test_target = data
    tracker_test_target.pose.position.x -= 5
    tracker_test_target.pose.position.z = 30

if __name__ == '__main__':
    rospy.init_node('tracker_test_pub', anonymous=True)
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, callback)
    pub = rospy.Publisher('tracker_result', PoseStamped, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    try:
        while not rospy.is_shutdown():
            pub.publish(tracker_test_target)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass

    rospy.spin()
