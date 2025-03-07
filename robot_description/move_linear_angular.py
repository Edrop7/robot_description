#!/usr/bin/env python

# python API for ROS files
import rospy

#geometry_msgs.msg.Twist is a message providing velocity commands (linear & angular)
from geometry_msgs.msg import Twist

# nav_msgs.msg.Odometry is a message represending position and velocity of Robot
from nav_msgs.msg import Odometry

# the position can be extracted from msg.pose.pose.position
# this function takes the x and y
def callback(msg):
    print('X: %s, Y: %s' % (msg.pose.pose.position.x, msg.pose.pose.position.y))

# algorithm run 
def main():
    sub = rospy.Subscriber('/odom', Odometry, callback) # using Odometry, callback is run whenever data is received
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) # using Twist, buffer of 10 messages between
    rospy.init_node('exercise_node', anonymous=True) # the node for this exercise, anonymous = True ensures unique names given
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        msg = Twist() # define message to have linear and angular velocities
        msg.linear.x = 0.3 # move forward 0.3 m/s
        msg.angular.z = 0.3 # turn counterclockwise on z-axis 0.3 rad/s (left)
        pub.publish(msg) # send over /cmd_vel
        rate.sleep() # wait to maintain 10 Hz

# code below ensures that CTRL + C closes main() completely
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass