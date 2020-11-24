import rospy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf

rospy.init_node("v_and_v_node")

pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

odom_xyt = (0, 0, 0)

lidar_corordinates = []


def proc_ranges(rng):
    crds = [np.array([math.cos(math.radians(i) - math.pi)*j, math.sin(math.radians(i) - math.pi)*j]) for i, j in enumerate(rng) if abs(j) != float('inf')]
    return crds


def scan_cb(mes: LaserScan):
    global lidar_corordinates
    lidar_corordinates = proc_ranges(mes.ranges)


def odom_cb(mes: Odometry):
    global odom_xyt
    odom_yaw = tf.transformations.euler_from_quaternion([
        data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])[2]
    odom_xyt = (mes.pose.pose.position.x, mes.pose.pose.position.y, odom_yaw)



rospy.Subscriber("/scan", LaserScan, scan_cb)
rospy.Subscriber("/odom", Odometry, odom_cb)



