import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseArray, Pose, Point
from nav_msgs.msg import Odometry
import tf
import numpy as np
import math 
from PID import PID
rospy.init_node("v_and_v_node")

pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
left_points_pub = rospy.Publisher("/left_points", PoseArray, queue_size=1)


odom_xyt = (0, 0, 0) 
odom_0_xyt = None

lidar_corordinates = []

pid = PID(kP=1.8, kD=0.1)

SPEED_LOW = 0.1
SPEED_HIGH = 0.2

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

def get_follow_data(points, target_l = 0.3):
    if len(points) > 0:
        mean_dist = abs(points.mean(axis=0)[1])
        coef = np.polyfit(points[:, 0],points[:, 1],1)
        poly1d_fn = np.poly1d(coef) 
        x1 = -0.1; y1 = poly1d_fn(x1)
        x2 = 0.1; y2 = poly1d_fn(x2)
        
        wall_angle = math.atan2(y2-y1, x2-x1)
        print("wall_angle", wall_angle, "mean_dist", mean_dist)
        return (mean_dist-target_l)/0.3, wall_angle/(math.pi/4)
    else:
        return None
def get_error(fl_d):
    return fl_d[0]*0.6 + fl_d[1]*0.4

def get_speed(fl_d, f_d):
    if abs(fl_d[0]) > 1 or f_d < 0.5:
        return SPEED_LOW
    else:
        return SPEED_HIGH

def follow(lidar_corordinates, target_l = 0.3): # v, a, d
    global pid, left_points_pub
    left_points = get_left_points(lidar_corordinates)
    forward_points = get_forward_points(lidar_corordinates)
    left_points_pub.publish(PoseArray(poses=[Pose(position=Point(x=i[0], y=i[1], z=0)) for i in list(left_points)+list(forward_points)], header=Header(frame_id="base_scan")))
    d = get_forward_wall_dist(forward_points)
    fl_d = get_follow_data(left_points, target_l)
    if (fl_d is None):
        return (0, 0, d)
    l_v = get_speed(fl_d, d)
    e = get_error(fl_d)
    a_v = 0
    if not (e == float('nan')):
        a_v = pid.calc(e)
    print("FL_D", fl_d, "FORWARD_D", d, "ERR", e, "OUT_A", a_v, "OUT_L", l_v)
    return (l_v, a_v, d) 

def odom_cb(mes):
    global odom_xyt, odom_0_xyt, odom_updated
    odom_yaw = tf.transformations.euler_from_quaternion([
        mes.pose.pose.orientation.x, mes.pose.pose.orientation.y, mes.pose.pose.orientation.z, mes.pose.pose.orientation.w])[2]
    if odom_0_xyt is None:
        odom_0_xyt = (mes.pose.pose.position.x, mes.pose.pose.position.y, odom_yaw)
    odom_xyt = (mes.pose.pose.position.x-odom_0_xyt[0], mes.pose.pose.position.y-odom_0_xyt[1], fix_a(odom_yaw-odom_0_xyt[2]))



rospy.Subscriber("/scan", LaserScan, scan_cb)
rospy.Subscriber("/odom", Odometry, odom_cb)
def vel_right(vel):
    pub_vel = Twist()
    pub_vel.angular.z = vel
    pub.publish(pub_vel)
def move_right():
    global odom_xyt,odom_0_xyt
    t = fix_a(odom_xyt[2] - (math.pi/2)+0.05)
    print("TURN START", t, odom_xyt[2])
    while abs(fix_a(t - odom_xyt[2])) > 0.05:
        vel_right(-0.2)
        print("TURN ",t,  odom_xyt[2], t - odom_xyt[2])
    vel_right(0)

def stop():
    global pub
    out = Twist()
    out.linear.x = 0
    out.angular.z = 0
    pub.publish(out)

turn_c = 0

while not rospy.is_shutdown():
    target_l = 0.3
    if turn_c == 0:
        target_l = 0.5
    v, a, d = follow(lidar_corordinates, target_l)
    if d < 0.305:
        stop()
        if turn_c < 5:
            raw_input()
            odom_0_xyt = odom_xyt
            move_right()
            stop()
            turn_c+=1
        else:
            break
        continue
        # break

    out = Twist()
    out.linear.x = v
    out.angular.z = a
    pub.publish(out)
    
    
    # left_points_pub.publish(PoseArray(poses=[Pose(position=Point(x=i[0], y=i[1], z=0)) for i in list(left_points)+list(forward_points)], header=Header(frame_id="base_scan")))
    rospy.sleep(0.001)
stop()
# pub.publish(out)

