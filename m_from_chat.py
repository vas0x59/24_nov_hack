import rospy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

rospy.init_node("turtle_mover")

length = 5
vel = Twist()

pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

def scan_cb(msg):
    global length
    if msg.ranges[0] == float('inf'):
        pass
    else:
        length = msg.ranges[0]
        rospy.loginfo(length)

rospy.Subscriber("/scan", LaserScan, scan_cb)

def mozg():
    global vel
    if length > 0.3:
        p = 7
    d = 0
    prev_vel = vel
        vel.linear.x = 0.1 * (length - 0.3) * p # - (prev_vel.linear.x) * d
        pub.publish(vel)
        rospy.loginfo("Edu")
    elif length < 0.3:
    vel.linear.x = -0.1
    pub.publish(vel)
    """
    else:
        rospy.loginfo("Stop")
        pub.publish(Twist())
    """

while not rospy.is_shutdown():
    mozg()
    rospy.sleep(0.3)