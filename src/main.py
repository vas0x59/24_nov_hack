import rospy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf
import numpy as np
import math 
rospy.init_node("v_and_v_node")

pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)


odom_xyt = (0, 0, 0) 
odom_0_xyt = None

lidar_corordinates = []


def proc_ranges(rng):
    crds = [np.array([math.cos(math.radians(i) - math.pi)*j, math.sin(math.radians(i) - math.pi)*j]) for i, j in enumerate(rng) if abs(j) != float('inf')]
    return crds


def scan_cb(mes):
    global lidar_corordinates
    lidar_corordinates = proc_ranges(mes.ranges)


def odom_cb(mes):
    global odom_xyt, odom_0_xyt, odom_updated
    odom_yaw = tf.transformations.euler_from_quaternion([
        mes.pose.pose.orientation.x, mes.pose.pose.orientation.y, mes.pose.pose.orientation.z, mes.pose.pose.orientation.w])[2]
    if odom_0_xyt is None:
        odom_0_xyt = (mes.pose.pose.position.x, mes.pose.pose.position.y, odom_yaw)
    odom_xyt = (mes.pose.pose.position.x-odom_0_xyt[0], mes.pose.pose.position.y-odom_0_xyt[1], odom_yaw-odom_0_xyt[2])



rospy.Subscriber("/scan", LaserScan, scan_cb)
rospy.Subscriber("/odom", Odometry, odom_cb)

while not rospy.is_shutdown():
    print(odom_xyt)
    rospy.sleep(0.1)


