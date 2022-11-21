from builtins import print
import tf2_ros
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import rospy
global pub
global subscriber
global tfBuffer


def createScanMsg(angle_min, angle_increment, ranges):
    laser_frequency = 40
    scan_time = rospy.Time.now()
    scan = LaserScan()
    scan.header.stamp = scan_time
    scan.header.frame_id = "base_scan"
    scan.angle_min = angle_min
    scan.angle_max = 6.28318977355957
    scan.angle_increment = angle_increment
    scan.time_increment = 0.0
    scan.range_min = 0.11999999731779099
    scan.range_max = 3.5
    scan.ranges = ranges
    return scan


def cart2pol(x, y):
    rho = np.sqrt(x**2 + y**2)
    phi = np.arctan2(y, x)
    return(rho, phi)


def talker():
    global pub
    pub = rospy.Publisher('my_scan', LaserScan, queue_size=10)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        ranges = np.ones(360) * np.inf
        angle_min = 0
        angle_increment = 0.017501922324299812
        try:
            symbols = ["star", "x", "power", "n1", "n2",
                       "n3", "n4", "n5", "n6", "n7", "n8", "n9"]
            # for symbol in symbols:
            trans = tfBuffer.lookup_transform(
                "base_scan", "symbol_"+"star", rospy.Time(0))
            trans2 = tfBuffer.lookup_transform(
                "base_scan", "odom", rospy.Time(0))

            rho, phi = cart2pol(trans.transform.translation.x,
                                trans.transform.translation.y)
            i = (phi - angle_min) / angle_increment
            ranges[int(i)] = rho
            print(tfBuffer.all_frames_as_string())

            scan_msg = createScanMsg(angle_min, angle_increment, ranges)
            pub.publish(scan_msg)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue


if __name__ == '__main__':
    try:
        rospy.init_node('custom_scanner', anonymous=True)
        rospy.sleep(2)
        # przygotowanie i wstępne wypełnienie bufora transformacji
        tfBuffer = tf2_ros.Buffer(rospy.Duration(120))
        listener = tf2_ros.TransformListener(tfBuffer)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
