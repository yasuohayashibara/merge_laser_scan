#!/usr/bin/env python
import rospy
import tf
import tf2_ros
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import LaserScan
from laser_geometry import LaserProjection
from sensor_msgs.msg import PointCloud2, PointField
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from std_msgs.msg import Header

class merge_laser_scan:
    def __init__(self):
        rospy.init_node('merge_laser_scan', anonymous=True)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.callback_scan, queue_size=1)
        self.pc_pub = rospy.Publisher("/laser_point_cloud", PointCloud2, queue_size=1)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.tf_robot_position = tf.TransformBroadcaster()

    def callback_scan(self, data):
        try:
            transform = self.tfBuffer.lookup_transform("odom", "base_link", rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return

        scan = LaserScan()
        scan = data
        projector = LaserProjection()
        cloud = projector.projectLaser(scan)
        fixed_frame_cloud = do_transform_cloud(cloud, transform)
        points = pc2.read_points(fixed_frame_cloud, field_names=['x', 'y', 'z'], skip_nans=True)
        data = []
        for x, y, z in points:
            if x > 2 and x < 7 and y > -2 and y < 2:
                data.append([x, y, z, 0xff0000])
            else:
                data.append([x, y, z, 0xffffff])

        HEADER = Header(frame_id='/odom')
        FIELDS = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
        ]
        filterd_cloud = pc2.create_cloud(HEADER, FIELDS, data)
        self.pc_pub.publish(filterd_cloud)

if __name__ == '__main__':
    mls = merge_laser_scan()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")
