#!/usr/bin/env python
import rospy
import tf
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry

class erase_pitch_motion:
    def __init__(self):
        rospy.init_node('erase_pitch_motion', anonymous=True)
        self.scan_sub = rospy.Subscriber("/icart_mini/odom", Odometry, self.callback_odom, queue_size=1)
        self.scan_sub = rospy.Subscriber("/tf", TFMessage, self.callback_odom, queue_size=1)
        self.tf_robot_position = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()

    def callback_odom(self, data):
        try:
            (trans, rot) = self.listener.lookupTransform('/base_link', '/odom', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return
        eular = tf.transformations.euler_from_quaternion(rot, "szxy")
        self.tf_robot_position.sendTransform((trans[0], trans[1], trans[2]),
            tf.transformations.quaternion_from_euler(eular[0], eular[1], eular[2], "szxy"),
            rospy.Time.now(), 'robot_position', 'base_link')

if __name__ == '__main__':
    mls = erase_pitch_motion()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")
