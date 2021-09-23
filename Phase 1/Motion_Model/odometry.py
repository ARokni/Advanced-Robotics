#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
import numpy as np
from numpy import linalg as LA

kp = 0.5
THRESHOLD = 0.002
THRESHOLD2 = 0.009
TRIAL_NUMBER = 50




x = 0.0
y = 0.0 
theta = 0.0

PI = 3.1415926535897

def new_odometry(msg):
    global x
    global y
    global theta
 
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
 
    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
 


def move():
    # Starts a new node
    rospy.init_node('vector_controller', anonymous=True)
    velocity_publisher = rospy.Publisher('/vector/cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber("/odom", Odometry, new_odometry)
    r = rospy.Rate(5) #5 Hz

    vel_msg = Twist()

    #Receiveing the user's input
    print("Let's move your robot")
    distance = float(input("Type your distance:"))
    isForward = float(input("Foward?: "))#True or False
    

    #Checking if the movement is forward or backwards
    if(isForward):
        direction = 1
    else:
        direction = -1
    #Since we are moving just in x-axis
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0
    states = []
    state_angle = []
    while not rospy.is_shutdown():
        t0 = rospy.Time.now().to_sec()
        #Loop to move the turtle in an specified distance
        for i in range(TRIAL_NUMBER):
            current_distance = 0
            initial_pose = np.array([x, y])
            initial_angle = theta
            while(abs(current_distance - distance)>THRESHOLD):
                if (abs(current_distance - distance)>THRESHOLD2):
                    odomet_speed = direction*abs(kp*(distance))
                else:
                    odomet_speed = direction*abs(kp*(distance - current_distance))
                print("SPD: ", odomet_speed,i)
                vel_msg.linear.x = odomet_speed
                #Publish the velocity
                velocity_publisher.publish(vel_msg)
                #Takes actual time to velocity calculus
                t1=rospy.Time.now().to_sec()
                #Calculates distancePoseStamped
                current_pose = np.array([x, y])
                current_distance = LA.norm(initial_pose - current_pose)
            #After the loop, stops the robot
            vel_msg.linear.x = 0
        #Force the robot to stop
            velocity_publisher.publish(vel_msg)
            print("Time:",t1-t0)
            states.append(abs(LA.norm(np.array([x,y]) - initial_pose )-distance))
            state_angle.append(abs(theta - initial_angle))
            r.sleep()  
        #np.save('/home/YourDir/Desktop/MyCourses/Master/Term2_Spring99/Advanced Robotics/Final Project/Phase1/Motion Model/Data/Odom_Trans', np.asarray(states))
        #np.save('/home/YourDir/Desktop/MyCourses/Master/Term2_Spring99/Advanced Robotics/Final Project/Phase1/Motion Model/Data/Odom_Trans_anglura', np.asarray(state_angle))
        rospy.signal_shutdown("reason")
       
 




if __name__ == '__main__':
    try:
        #Testing our function
        move()
    except rospy.ROSInterruptException: pass