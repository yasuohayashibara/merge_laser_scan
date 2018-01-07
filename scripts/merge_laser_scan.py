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
from geometry_msgs.msg import Quaternion
import math

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

        trans = transform.transform.translation
        rot = transform.transform.rotation
        eular = tf.transformations.euler_from_quaternion((rot.x, rot.y, rot.z, rot.w), "szxy")
        quat = tf.transformations.quaternion_from_euler(eular[0], 0, 0, "szxy")
        transform.transform.translation.z = 0
        transform.transform.rotation = Quaternion(quat[0], quat[1], quat[2], quat[3])

        self.tf_robot_position.sendTransform((trans.x, trans.y, 0),
            (quat[0], quat[1], quat[2], quat[3]),
            rospy.Time.now(), 'robot_position', 'odom')

        scan = LaserScan()
        scan = data
        projector = LaserProjection()
        cloud = projector.projectLaser(scan)
        fixed_frame_cloud = do_transform_cloud(cloud, transform)
        points = pc2.read_points(fixed_frame_cloud, field_names=['x', 'y', 'z'], skip_nans=True)
        pole = [[0,0,0],[0,0,0],[0,0,0]]
        data = []
        for x, y, z in points:
            if x > 5 and x < 7 and y > -2 and y < 2:
                data.append([x, y, z, 0xff0000])
                pole[0][0] += x
                pole[0][1] += y
                pole[0][2] += 1
            elif x > 2 and x < 4 and y > -2 and y < 0:
                data.append([x, y, z, 0x00ff00])
                pole[1][0] += x
                pole[1][1] += y
                pole[1][2] += 1
            elif x > 2 and x < 4 and y > 0 and y < 2:
                data.append([x, y, z, 0x0000ff])
                pole[2][0] += x
                pole[2][1] += y
                pole[2][2] += 1
            else:
                data.append([x, y, z, 0xffff00])
        for p in pole:
            if p[2] != 0:
                p[0] /= p[2]
                p[1] /= p[2]

        if pole[0][2] == 0 or (pole[1][2] == 0 and pole[2][2] == 0):
            return

        trans_diff = [6.0 - pole[0][0], 0.0 - pole[0][1]]
        if pole[1][2] != 0:
            rot_diff = math.atan2(-1, -3) - math.atan2(pole[1][1] - pole[0][1], pole[1][0] - pole[0][0])
        elif pole[2][2] != 0:
            rot_diff = math.atan2(1, -3) - math.atan2(pole[2][1] - pole[0][1], pole[2][0] - pole[0][0])

        data_pole = []
        for x, y, z, color in data:
            tx = x + trans_diff[0] - 6.0
            ty = y + trans_diff[1]
            rx = tx * math.cos(rot_diff) - ty * math.sin(rot_diff) + 6.0
            ry = tx * math.sin(rot_diff) + ty * math.cos(rot_diff)
            data_pole.append([rx, ry, z, color])

#        print data_pole

        HEADER = Header(frame_id='/odom')
        FIELDS = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
        ]
        filterd_cloud = pc2.create_cloud(HEADER, FIELDS, data_pole)
        self.pc_pub.publish(filterd_cloud)

if __name__ == '__main__':
    mls = merge_laser_scan()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")
