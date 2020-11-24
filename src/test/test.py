import rospy
import tf
import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

rospy.init_node('hack')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

global odom_xyt
odom_xyt = (0, 0, 0)
odom_0_xyt = None

def fix_a(a):
    if a < -math.pi:
        return a + 2*math.pi
    elif a > math.pi:
        return a - 2*math.pi
    else:
        return a

def odom_cb(mes):
    global odom_xyt, odom_0_xyt
    odom_yaw = tf.transformations.euler_from_quaternion([
        mes.pose.pose.orientation.x, mes.pose.pose.orientation.y, mes.pose.pose.orientation.z, mes.pose.pose.orientation.w])[2]
    if odom_0_xyt is None:
        odom_0_xyt = (mes.pose.pose.position.x, mes.pose.pose.position.y, odom_yaw)
    odom_xyt = (mes.pose.pose.position.x-odom_0_xyt[0], mes.pose.pose.position.y-odom_0_xyt[1], fix_a(odom_yaw-odom_0_xyt[2]))

rospy.Subscriber('/odom',Odometry, odom_cb)
def main():
    global odom_xyt,odom_0_xyt
    if odom_xyt[2] > -1*(math.pi/2)+0.13:
        move_right(-0.2)
    else:
        move_right(0)
        exit()
    print(odom_xyt)
def move_right(vel):
    pub_vel = Twist()
    pub_vel.angular.z = vel
    pub.publish(pub_vel)

while not rospy.is_shutdown():
    main()
    rospy.sleep(0.1)