#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
import numpy as np
from numpy import linalg as LA


x = 0.0
y = 0.0 
theta = 0.0

PI = 3.1415926535897

kp = 0.5
THRESHOLD = 0.99*PI/180
THRESHOLD2 = 1*PI/180
TRIAL_NUMBER = 50





def new_odometry(msg):
    global x
    global y
    global theta
 
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
 
    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta_tmp) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    if(theta_tmp<0):
        theta = -1*theta_tmp
    else:
        theta = theta_tmp
 


def move():
    # Starts a new node
    rospy.init_node('vector_controller', anonymous=True)
    velocity_publisher = rospy.Publisher('/vector/cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber("/odom", Odometry, new_odometry)
    r = rospy.Rate(0.5) #5 Hz

    vel_msg = Twist()

    #Receiveing the user's input
    print("Let's move your robot")
    angle = float(input("Type your Angular distance:"))
    isForward = float(input("Foward?: "))#True or False
    
    relative_angle = angle*2*PI/360
    
    

    #Checking if the movement is forward or backwards
    if(isForward):
        direction = 1
    else:
        direction = -1
    #Since we are moving just in x-axis
    vel_msg.linear.x=0
    vel_msg.linear.y=0
    vel_msg.linear.z=0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0

    states = []
    states_dist = []
    while not rospy.is_shutdown():
        t0 = rospy.Time.now().to_sec()
        #Loop to move the turtle in an specified distance
        for i in range(TRIAL_NUMBER):
            covered_angle = 0
            initial_pose = np.array([x, y])
            initial_angle = theta
            while(abs(covered_angle - relative_angle)>THRESHOLD):
                if(abs(covered_angle - relative_angle)>THRESHOLD2):
                    odomet_ang_spd = direction*abs(0.6*(relative_angle))
                else:
                    odomet_ang_spd = direction*abs(kp*(covered_angle - relative_angle))
                print("SPD: ", odomet_ang_spd,covered_angle,theta,i)
                vel_msg.angular.z = odomet_ang_spd
                #Publish the velocity
                velocity_publisher.publish(vel_msg)
                #Takes actual time to velocity calculus
                t1=rospy.Time.now().to_sec()
                #Calculates distancePoseStamped
                current_angle = theta
                covered_angle = abs(initial_angle - current_angle)
                
            #After the loop, stops the robot
            vel_msg.angular.z = 0
            velocity_publisher.publish(vel_msg)
            
            print("Time:", t1 -t0)
            states.append(abs(theta - initial_angle))
            states_dist.append(abs(LA.norm(np.array([x,y]) - initial_pose )))
            r.sleep()  
            
       # np.save('/home/amiredge/Desktop/MyCourses/Master/Term2_Spring99/Advanced Robotics/Final Project/Phase1/Motion Model/Data/Odom_Angular', np.asarray(states))
        #np.save('/home/amiredge/Desktop/MyCourses/Master/Term2_Spring99/Advanced Robotics/Final Project/Phase1/Motion Model/Data/Odom_Angular_dist', np.asarray(states_dist))
        rospy.signal_shutdown("reason")
       
 




if __name__ == '__main__':
    try:
        #Testing our function
        move()
    except rospy.ROSInterruptException: pass