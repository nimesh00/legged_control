import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import numpy as np

vel_x = 0.0
vel_y = 0.0
w_z = 0.0

def joy_callback(data):
    global vel_x, vel_y, w_z
    vel_x = 0.5 * data.axes[1]
    vel_y = 0.5 * data.axes[0]
    w_z = data.axes[2]
    # print("Joy Callback. Vel: ", vel_x, vel_y, w_z)

def talker():
    global vel_x, vel_y, w_z
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('/joy', Joy, joy_callback)
    rospy.init_node('send_base_cmd_vel', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        vel = Twist()
        vel.linear.x = vel_x + 0.5
        vel.linear.y = vel_y
        vel.linear.z = 0.0
        vel.angular.x = 0.0
        vel.angular.y = 0.0
        vel.angular.z = w_z
        
        # rospy.loginfo(vel)
        pub.publish(vel)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass