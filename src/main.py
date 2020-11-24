import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseArray, Pose, Point
from nav_msgs.msg import Odometry
import tf
import numpy as np
import math 
rospy.init_node("v_and_v_node")

pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
left_points_pub = rospy.Publisher("/left_points", PoseArray, queue_size=1)


odom_xyt = (0, 0, 0) 
odom_0_xyt = None

lidar_corordinates = []

def fix_a(a):
    if a < -math.pi:
        return a + 2*math.pi
    elif a > math.pi:
        return a - 2*math.pi
    else:
        return a

def proc_ranges(rng):
    crds = np.array([np.array([math.cos(math.radians(i) - math.pi)*j, math.sin(math.radians(i) - math.pi)*j]) for i, j in enumerate(rng) if abs(j) != float('inf')])
    return crds


def scan_cb(mes):
    global lidar_corordinates
    lidar_corordinates = proc_ranges(mes.ranges)
    
def get_left_points(lidar_corordinates):
    return np.array([p for p in lidar_corordinates if p[1] < 0 and -0.1 <= p[0] <= 0.1])

def get_forward_points(lidar_corordinates):
    return np.array([p for p in lidar_corordinates if p[0] < 0 and -0.05 <= p[1] <= 0.05])


def get_forward_wall_dist(points):
    if len(points) > 0:
        return abs(points.mean(axis=0)[0])
    else:
        return float('nan')

def get_follow_data(points):
    mean_dist = abs(points.mean(axis=0)[1])
    wall_angle = 

def odom_cb(mes):
    global odom_xyt, odom_0_xyt, odom_updated
    odom_yaw = tf.transformations.euler_from_quaternion([
        mes.pose.pose.orientation.x, mes.pose.pose.orientation.y, mes.pose.pose.orientation.z, mes.pose.pose.orientation.w])[2]
    if odom_0_xyt is None:
        odom_0_xyt = (mes.pose.pose.position.x, mes.pose.pose.position.y, odom_yaw)
    odom_xyt = (mes.pose.pose.position.x-odom_0_xyt[0], mes.pose.pose.position.y-odom_0_xyt[1], fix_a(odom_yaw-odom_0_xyt[2]))



rospy.Subscriber("/scan", LaserScan, scan_cb)
rospy.Subscriber("/odom", Odometry, odom_cb)

while not rospy.is_shutdown():
    # print(odom_xyt)
    forward_points = get_forward_points(lidar_corordinates)
    left_points = get_left_points(lidar_corordinates)
    print(get_forward_wall_dist(forward_points))
    left_points_pub.publish(PoseArray(poses=[Pose(position=Point(x=i[0], y=i[1], z=0)) for i in list(left_points)+list(forward_points)], header=Header(frame_id="base_scan")))
    rospy.sleep(0.1)


