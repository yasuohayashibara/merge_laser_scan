#!/usr/bin/env python
import rospy
import tf
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import LaserScan
from laser_geometry import LaserProjection
from sensor_msgs.msg import PointCloud2

from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry

class merge_laser_scan:
    def __init__(self):
        rospy.init_node('merge_laser_scan', anonymous=True)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.callback_scan, queue_size=1)
        self.pc_pub = rospy.Publisher("/laser_point_cloud", PointCloud2, queue_size=1)
        self.listener = tf.TransformListener()
        self.tf_robot_position = tf.TransformBroadcaster()

    def callback_scan(self, data):
        scan = LaserScan()
        scan = data
        projector = LaserProjection()
        cloud = projector.projectLaser(scan)

        self.pc_pub.publish(cloud)

#        try:
#            (trans, rot) = self.listener.lookupTransform('/base_link', '/odom', rospy.Time(0))
#        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
#            return
#        eular = tf.transformations.euler_from_quaternion(rot, "szxy")
#        self.tf_robot_position.sendTransform((trans[0], trans[1], trans[2]),
#            tf.transformations.quaternion_from_euler(eular[0], eular[1], eular[2], "szxy"),
#            rospy.Time.now(), 'robot_position', 'base_link')

if __name__ == '__main__':
    mls = merge_laser_scan()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")
