import rospy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

rospy.init_node("v_and_v_node")

pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

odom_xyt = (0, 0, 0)

def scan_cb(mes: LaserScan):


def odom_cb(mes: Odometry):
    global odom_xyt
    odom_xyt = (mes.pose.pose.position.x, mes.pose.pose.position.y, mes.pose.pose.orientation)



rospy.Subscriber("/scan", LaserScan, scan_cb)
rospy.Subscriber("/odom", Odometry, odom_cb)



