#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Range
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist
import pandas as pd
import math
import numpy as np
import matplotlib.pyplot as plt
import xml.etree.ElementTree as ET
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
from numpy import linalg as LA
from itertools import repeat
import random
import scipy.stats as stats
import copy

path = '/home/amiredge/catkin_ws/src/anki_description/world'
root = ET.parse(path + '/map.world').getroot()
x = 0.0
y = 0.0
theta = 0.0
sensor_range = 0

class World_Map:

    def __init__(self):
        self.map = self.readMap()
        self.x_min, self.x_max, self.y_min, self.y_max = self.map_Width_Height()
        #self.plt_map = self.plot_map()
        return None

    def split(self, e):
        return [float(v) for v in e.text.split(' ')]

    def rotate(self, xy, radians, origin):
        x = xy[0]
        y = xy[1]
        offset_x, offset_y = origin
        adjusted_x = (x - offset_x)
        adjusted_y = (y - offset_y)
        cos_rad = math.cos(radians)
        sin_rad = math.sin(radians)
        qx = offset_x + cos_rad * adjusted_x + sin_rad * adjusted_y
        qy = offset_y + -sin_rad * adjusted_x + cos_rad * adjusted_y
        return qx, qy

    def readMap(self):
        obstacle_in_map = []
        obs_poses = [self.split(e) for e in root.findall('.//link/collision/../pose')]
        obs_sizes = [self.split(e) for e in root.findall('.//link/pose/../collision//size')]
        # bias = [self.split(root.findall('.//model/pose')[1])]
        bias_indx = 0
        for i in range(3):
            sum_ = np.sum([self.split(root.findall('.//model/pose')[i])])
            if sum_ != 0.0:
                bias_indx = i
        bias = [self.split(root.findall('.//model/pose')[bias_indx])]
        obs_poses = np.add(obs_poses, bias)
        for i in range(len(obs_poses)):
            rect = np.zeros((4, 2))
            w = obs_sizes[i][0]
            h = obs_sizes[i][1]
            x = obs_poses[i][0]
            y = obs_poses[i][1]
            angle = obs_poses[i][5]
            left_down = self.rotate([x - w / 2, y - h / 2], angle, (x, y))
            left_up = self.rotate([x - w / 2, y + h / 2], angle, (x, y))
            right_down = self.rotate([x + w / 2, y - h / 2], angle, (x, y))
            right_up = self.rotate([x + w / 2, y + h / 2], angle, (x, y))
            rect[0, :] = np.array(left_down)
            rect[1, :] = np.array(left_up)
            rect[2, :] = np.array(right_down)
            rect[3, :] = np.array(right_up)
            obstacle_in_map.append(rect)
        return obstacle_in_map

    def map_Width_Height(self, ):
        _map = self.map
        obs_map = np.array(_map)
        max_min_arr = np.zeros((len(_map), 4))
        for j in range(len(obs_map)):
            arr = obs_map[j]
            max_min_arr[j, 0] = min(arr[:, 0])  # min of x
            max_min_arr[j, 1] = max(arr[:, 0])  # max of x
            max_min_arr[j, 2] = min(arr[:, 1])  # min of y
            max_min_arr[j, 3] = max(arr[:, 1])  # max of y

        x_min_sorted = (max_min_arr[:, 0])
        x_max_sorted = (max_min_arr[:, 1])
        y_min_sorted = (max_min_arr[:, 2])
        y_max_sorted = (max_min_arr[:, 3])
        x_min_sorted.sort(), x_max_sorted.sort(), y_min_sorted.sort(), y_max_sorted.sort()
        x_min = x_min_sorted[1]
        x_max = x_max_sorted[-2]
        y_min = y_min_sorted[1]
        y_max = y_max_sorted[-2]

        return x_min, x_max, y_min, y_max

    def plot_map(self):
        _map = self.map
        x_min, x_max, y_min, y_max = (-0.75, 0.75, -0.75, 1.75)
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.set(xlim=(x_min, x_max), ylim=(y_min, y_max))
        for i in range(len(_map)):
            obstacle_map = _map[i]
            x1 = [obstacle_map[0][0], obstacle_map[1][0]]
            y1 = [obstacle_map[0][1], obstacle_map[1][1]]

            x2 = [obstacle_map[0][0], obstacle_map[2][0]]
            y2 = [obstacle_map[0][1], obstacle_map[2][1]]

            x3 = [obstacle_map[1][0], obstacle_map[3][0]]
            y3 = [obstacle_map[1][1], obstacle_map[3][1]]

            x4 = [obstacle_map[2][0], obstacle_map[3][0]]
            y4 = [obstacle_map[2][1], obstacle_map[3][1]]
            plt.plot(x1, y1)
            plt.plot(x2, y2)
            plt.plot(x3, y3)
            plt.plot(x4, y4)
        return plt, ax

    
    def plot_point_on_map(self, points):
        plt, ax = self.plot_map()
        for point in points:
          ax.arrow(point[0], point[1], 0.005*np.cos(point[2]), 0.005*np.sin(point[2]), head_width=0.03, head_length=0.03, fc='b', ec='b', fill = False)
        ax.scatter(x,y, s= 200, color = 'r')
        plt.show()

    def check_point_out_of_obstacle(self, xy_point):
        point = Point([xy_point[0], xy_point[1]])
        _map = self.map
        for i in range(len(_map)):
            rect = _map[i]
            v1 = rect[0, :]
            v2 = rect[1, :]
            v3 = rect[2, :]
            v4 = rect[3, :]
            polygon = Polygon([[v1[0], v1[1]], [v2[0], v2[1]], [v4[0], v4[1]], [v3[0], v3[1]]])
            if (point.within(polygon)):
                return False
        return True

    def check_poin_inside_map(self, xy_point):
        point = Point([xy_point[0], xy_point[1]])
        polygon = Polygon(
            [[self.x_min, self.y_min], [self.x_min, self.y_max], [self.x_max, self.y_max], [self.x_max, self.y_min]])
        if (point.within(polygon)):
            return True
        else:
            return False

    def check_point_feasibility(self, xy_point):
        if (self.check_point_out_of_obstacle(xy_point) and self.check_poin_inside_map(xy_point)):
            return True
        else:
            return False

    def calc_min_dist_to_obstacle(self, point):
        point = Point([point[0], point[1]])
        _map = self.map
        distances = []
        for i in range(len(_map)):
            rect = _map[i]
            v1 = rect[0, :]
            v2 = rect[1, :]
            v3 = rect[2, :]
            v4 = rect[3, :]
            polygon = Polygon([[v1[0], v1[1]], [v2[0], v2[1]], [v4[0], v4[1]], [v3[0], v3[1]]])
            distances.append(polygon.exterior.distance(point))
        min_distance = min(distances)
        return min_distance

class Motion_Model:
    def __init__(self, alpha1, alpha2, alpha3, alpha4, _map):
        self.__alpha1 = alpha1
        self.__alpha2 = alpha2
        self.__alpha3 = alpha3
        self.__alpha4 = alpha4
        self.__map = _map

    def random_particle(self):
        _map = self.__map
        LIMIT_TIME = 1000
        count = 0
        while (True):
            x_min, x_max, y_min, y_max = _map.map_Width_Height()
            _x = np.random.uniform(x_min, x_max)
            _y = np.random.uniform(y_min, y_max)
            _theta = np.random.uniform(-np.pi, np.pi)
            if (_map.check_point_feasibility([_x, _y])):
                return [_x, _y, _theta]
            elif (count > LIMIT_TIME):
                print("ERROR_SAMPLE_GENERATION")
                return [0, 0, 0]

    def __delta_cal(self, x_o_prev, y_o_prev, theta_o_prev, x_o_curr, y_o_curr, theta_o_curr):
        delta_rot1 = np.arctan2(y_o_curr - y_o_prev, x_o_curr - x_o_prev) - theta_o_prev
        delta_trans = np.sqrt((x_o_curr - x_o_prev) ** 2 + (y_o_curr - y_o_prev) ** 2)
        delta_rot2 = theta_o_curr - theta_o_prev - delta_rot1

        return delta_rot1, delta_trans, delta_rot2

    def __sample(self, b):
        _rand = np.random.uniform(-1, 1, 12)
        return (b / 6.0) * np.sum(_rand)

    def __pred_delta(self, _delta_rot1, _delta_trans, _delta_rot2):
        delta_rot1_pred = _delta_rot1 - self.__sample(self.__alpha1 * _delta_rot1 + self.__alpha2 * _delta_trans)
        delta_trans_pred = _delta_trans - self.__sample(
            self.__alpha3 * _delta_trans + self.__alpha4 * (_delta_rot1 + _delta_rot2))
        delta_rot2_pred = _delta_rot2 - self.__sample(self.__alpha1 * _delta_rot2 + self.__alpha2 * _delta_trans)
        return delta_rot1_pred, delta_trans_pred, delta_rot2_pred

    def __compatible_with_map(self, X_tmp):
        _map = self.__map
        if (_map.check_point_feasibility(X_tmp)):
            return X_tmp
        else:
            return self.random_particle()

    def __sample_motion_model_odometry(self, u, X_prev):  # Fekr konam x_prev az makane ghablie particle miad
        odom_prev, odom_curr = u
        x_o_prev, y_o_prev, theta_o_prev = odom_prev
        x_o_curr, y_o_curr, theta_o_curr = odom_curr
        x_prev, y_prev, theta_prev = X_prev

        delta_rot1, delta_trans, delta_rot2 = self.__delta_cal(x_o_prev, y_o_prev, theta_o_prev, x_o_curr, y_o_curr,
                                                               theta_o_curr)
        delta_rot1_pred, delta_trans_pred, delta_rot2_pred = self.__pred_delta(_delta_rot1=delta_rot1,
                                                                               _delta_trans=delta_trans,
                                                                               _delta_rot2=delta_rot2)
        x_curr = x_prev + delta_trans_pred * np.cos(theta_prev + delta_rot1_pred)
        y_curr = y_prev + delta_trans_pred * np.sin(theta_prev + delta_rot1_pred)
        theta_curr = theta_prev + delta_rot1_pred + delta_rot2_pred
        X_tmp = [x_curr, y_curr, theta_curr]
        #X_curr = self.__compatible_with_map(X_tmp)
        X_curr = X_tmp
        return X_curr

    def motion_model(self, u, particles):
        for particle in range(len(particles)):
            particles[particle] = self.__sample_motion_model_odometry(u, particles[particle])
        return particles

class Measurement_model:
    def __init__(self):
        self.sigma = 0.03
        self.z_hit = 0.87
        self.z_rand = 0.12
        self.z_max = 0.45
        return None

    def calc_true_position_of_obstacle_new(self, particle,range_measurement, map_obj):
        x_particle = particle[0]
        y_particle = particle[1]
        head_particle = particle[2]
        x_z_k_star = x_particle + range_measurement * np.cos(head_particle)
        y_z_k_star = y_particle + range_measurement * np.sin(head_particle)
        point = [x_z_k_star, y_z_k_star]
        min_distance = map_obj.calc_min_dist_to_obstacle(point)
        return min_distance

    def calc_true_position_of_obstacle(self, particle, map_obj):
        x_particle = particle[0]
        y_particle = particle[1]
        head_particle = particle[2]
        z_max = self.z_max * 100
        for measure in range(0, int(z_max) + 1):
            distance = measure / 100.0
            x_z_k_star = x_particle + distance * np.cos(head_particle)
            y_z_k_star = y_particle + distance * np.sin(head_particle)
            point = [x_z_k_star, y_z_k_star]
            if not map_obj.check_point_feasibility(point):
                return distance
        return distance

    def random_noise(self, range_measurement):
        if 0 <= range_measurement < self.z_max:
            return 1 / self.z_max
        else:
            return 0

    def range_finder_alg(self, range_measurement, main_obj, map_obj):
        weight_of_particles = []
        particles = main_obj.particles
        range_measurement += 0.05
        if np.abs(self.z_max - range_measurement) > 1e-3:
            for particle in range(len(particles)):
                z_k_star = self.calc_true_position_of_obstacle_new(particles[particle],range_measurement, map_obj)
                p_hit = stats.norm.pdf(z_k_star, 0, self.sigma)
                p_zt_given_xt = p_hit
                weight_of_particles.append(p_zt_given_xt)
            main_obj.weights = weight_of_particles
            return True
        elif main_obj.start_of_algorithm:
            weight_ = np.ones(len(particles))
            main_obj.weights = weight_
            main_obj.start_of_algorithm = False
            return False
        else:
            return False


class Main_file:

    def __init__(self):
        self.num_of_particles = 1500
        self.start_of_algorithm = True
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.prev_theta = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.weights = []
        self.particles = []
        self.kidnapping = False
        self.random_sampling = False
        self.w_avg = 0
        self.w_slow = 0
        self.w_fast = 0
        self.alpha_slow = 0.05
        self.alpha_fast = 0.2
        return None

    def generate_random_particle(self, map_obj, initialize):
        num_of_particles = self.num_of_particles
        count = 0
        particles = []
        while count < num_of_particles:
            x_min, x_max, y_min, y_max = map_obj.map_Width_Height()
            _x = np.random.uniform(x_min, x_max)
            _y = np.random.uniform(y_min, y_max)
            _theta = np.random.uniform(-np.pi/6.0 +theta, np.pi/6.0 + theta)
            if map_obj.check_point_feasibility([_x, _y]):
                rand_point = [_x, _y, _theta]
                if initialize:
                    return rand_point
                else:
                    particles.append(rand_point)
                    count = count + 1
        return particles

def new_odometry(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta_tmp) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    theta = theta_tmp
def read_position():
    sub = rospy.Subscriber("/odom", Odometry, new_odometry)

def initialize_robot_pose(initialize_pose, quaternion):
    rospy.init_node('vector_controller', anonymous=True)
    state_msg = ModelState()
    state_msg.model_name = 'vector'
    state_msg.pose.position.x = initialize_pose[0]
    state_msg.pose.position.y = initialize_pose[1]
    state_msg.pose.position.z = 0
    state_msg.pose.orientation.x = quaternion[0]
    state_msg.pose.orientation.y = quaternion[1]
    state_msg.pose.orientation.z = quaternion[2]
    state_msg.pose.orientation.w = quaternion[3]
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state(state_msg)

    except rospy.ServiceException, e:
        print("Service call failed: %s" % e)

def callback_range_sensor(msg):
    global sensor_range
    sensor_range = msg.range

def read_range_measurements():
    sub = rospy.Subscriber("/vector/laser", Range, callback_range_sensor)

class Move:
    def __init__(self):
        self.__kp = 0.5
        self.__threshold = 0.02
        self.__threshold2 = 0.09
        self.__direction_translational = 1
        self.__distance = 0.1
        # self.__distance = 0.05
        self.__kp_angular = 0.5
        self.__threshold_angular = 0.30 * np.pi / 180.0
        self.__threshold2_angular = 0.31 * np.pi / 180.0
        self.__obstacle_threshold = 0.14
        self.__limit_turning = 1
        self.__angle = 30

    def not_obstacle(self):
        read_range_measurements()
        
        if (sensor_range > self.__obstacle_threshold):
            return True
        else:
            return False

    def __calc_control_odometry_rotational(self, covered_angle, relative_angle, direction):
        if (abs(covered_angle - relative_angle) > self.__threshold2_angular):
            odomet_ang_spd = direction * abs(0.6 * (relative_angle))
        else:
            odomet_ang_spd = direction * abs(self.__kp_angular * (covered_angle - relative_angle))
        return odomet_ang_spd

    def __turn(self, velocity_publisher, move_dir, move_angle):
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        relative_angle = np.copy(move_angle * 2 * np.pi / 360)
        direction = move_dir
        cnt = 0
        while (cnt < self.__limit_turning):
            covered_angle = 0
            initial_angle = theta
            while (abs(covered_angle - relative_angle) > self.__threshold_angular):
                vel_msg.angular.z = self.__calc_control_odometry_rotational(covered_angle, relative_angle, direction)
                # Publish the velocity
                velocity_publisher.publish(vel_msg)
                # Takes actual time to velocity calculus
                # Calculates distancePoseStamped
                current_angle = theta
                if(current_angle - initial_angle<-np.pi/2):
                    current_angle = current_angle + 2*np.pi
                elif(current_angle - initial_angle>np.pi/2):
                    current_angle = current_angle - 2*np.pi
                covered_angle = abs(initial_angle - current_angle)
            # After the loop, stops the robot
            vel_msg.angular.z = 0
            velocity_publisher.publish(vel_msg)
            r = rospy.Rate(1)  # 5 Hz
            r.sleep()
            cnt += 1
            if (self.not_obstacle()):
                break

    def __calc_control_odometry_translational(self, current_distance, distance, direction):
        if (abs(current_distance - distance) > self.__threshold2):
            odomet_speed = direction * abs(self.__kp * (distance))
        else:
            odomet_speed = direction * abs(self.__kp * (distance - current_distance))
        return odomet_speed

    def __go_forward(self, velocity_publisher):
        vel_msg = Twist()
        distance = self.__distance
        direction = self.__direction_translational
        # Checking if the movement is forward or backwards
        # Since we are moving just in x-axis
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        # Loop to move the turtle in an specified distance
        current_distance = 0
        initial_pose = np.array([x, y])
        while (abs(current_distance - distance) > self.__threshold):
            vel_msg.linear.x = self.__calc_control_odometry_translational(current_distance, distance, direction)
            # Publish the velocity
            velocity_publisher.publish(vel_msg)
            # Calculates distancePoseStamped
            current_pose = np.array([x, y])
            current_distance = LA.norm(initial_pose - current_pose)
        # After the loop, stops the robot
        vel_msg.linear.x = 0
        # Force the robot to stop
        velocity_publisher.publish(vel_msg)

    def move_robot(self, move_dir, move_angle, translation_rotation):
        velocity_publisher = rospy.Publisher('/vector/cmd_vel', Twist, queue_size=10)
        sub = rospy.Subscriber("/odom", Odometry, new_odometry)
        rotation = 1
        translation = 0
        if(translation_rotation == rotation):
            self.__turn(velocity_publisher, move_dir, move_angle)
        no_obstacle = self.not_obstacle()
        if (no_obstacle and (translation_rotation == translation)):
            self.__go_forward(velocity_publisher)
            
            
    def move_in_map(self, check_right, check_left):
         left = 1
         right = -1
         _translation = 0
         _rotation =1
         _move_angle = 27
        
         if(not self.not_obstacle()):
                 self.move_robot( move_dir = left, move_angle = _move_angle, translation_rotation = _rotation)
                 _update_weight = True
         elif(not check_right):
                 self.move_robot( move_dir = right, move_angle = _move_angle/2.0, translation_rotation = _rotation)
                 check_right =  self.not_obstacle()
                 _update_weight = False
         elif(not check_left):
                 self.move_robot( move_dir = left, move_angle = 2*_move_angle/2.0, translation_rotation = _rotation)
                 check_left = self.not_obstacle()
                 check_right = check_left
                # _update_weight = True
                 _update_weight = False
                    
         else:
                  self.move_robot( move_dir = right, move_angle = _move_angle/2.0, translation_rotation = _rotation)
                  self.move_robot( move_dir = left, move_angle = _move_angle, translation_rotation = _translation)
                  check_right = False; check_left = False
                  _update_weight = True
         return check_right, check_left, _update_weight
    

def randon_resampling(main_obj, map_obj):
    percetage_random = 0.05
    weights = copy.copy(main_obj.weights)
    weights /= np.sum(weights)
    temp_weight = copy.copy(main_obj.weights)
    remove_limit = np.ceil(percetage_random * len(temp_weight))
    min_indexes = []
    for k in range(int(remove_limit)):
        indx_min = temp_weight.index(min(temp_weight))
        temp_weight[indx_min] = 1000
        min_indexes.append(indx_min)
    lower_bound_prob = []
    upper_bound_prob = []
    index = 0
    selected_indexes = np.zeros(len(main_obj.particles), 'i')
    lower_cumsum = 0
    upper_cumsum = 0
    # apply roulette wheel algorithm bounds
    for i in range(len(main_obj.particles)):
        lower_bound_prob.append(lower_cumsum)
        lower_cumsum = lower_cumsum + weights[i]
        upper_cumsum = upper_cumsum + weights[i]
        upper_bound_prob.append(upper_cumsum)
    for i in range(len(main_obj.particles)):
        random_num = np.random.uniform(0, 1, 1)
        for weight_idx in range(len(weights)):
            if (random_num >= float(lower_bound_prob[weight_idx])) & (random_num < float(upper_bound_prob[weight_idx])):
                index = weight_idx
        selected_indexes[i] = index
    main_obj.particles = [main_obj.particles[i] for i in selected_indexes]
    for idx in range(int(remove_limit)):
        main_obj.particles[idx] = main_obj.generate_random_particle(map_obj,True)
    main_obj.weights = [main_obj.weights[i] for i in selected_indexes]

def resampling(main_obj):
    weights = copy.copy(main_obj.weights)
    weights /= np.sum(weights)
    lower_bound_prob = []
    upper_bound_prob = []
    index = 0
    selected_indexes = np.zeros(len(main_obj.particles), 'i')
    lower_cumsum = 0
    upper_cumsum = 0
    # apply roulette wheel algorithm bounds
    for i in range(len(main_obj.particles)):
        lower_bound_prob.append(lower_cumsum)
        lower_cumsum = lower_cumsum + weights[i]
        upper_cumsum = upper_cumsum + weights[i]
        upper_bound_prob.append(upper_cumsum)
    for i in range(len(main_obj.particles)):
        random_num = np.random.uniform(0, 1, 1)
        for weight_idx in range(len(weights)):
            if (random_num >= float(lower_bound_prob[weight_idx])) & (random_num < float(upper_bound_prob[weight_idx])):
                index = weight_idx
        selected_indexes[i] = index
    main_obj.particles = [main_obj.particles[i] for i in selected_indexes]
    main_obj.weights = [main_obj.weights[i] for i in selected_indexes]


def kidnap_resampling(main_obj, map_obj):
    weights = copy.copy(main_obj.weights)
    weights /= np.sum(weights)
    lower_bound_prob = []
    upper_bound_prob = []
    index = 0
    selected_indexes = np.zeros(len(main_obj.particles), 'i')
    lower_cumsum = 0
    upper_cumsum = 0
    main_obj.w_avg = np.mean(np.array(main_obj.weights))
    main_obj.w_slow = main_obj.w_slow + main_obj.alpha_slow * (main_obj.w_avg - main_obj.w_slow)
    main_obj.w_fast = main_obj.w_fast + main_obj.alpha_fast * (main_obj.w_avg - main_obj.w_fast)
    print("main_obj.w_avg ,main_obj.w_slow, main_obj.w_fast: ",main_obj.w_avg
          ,main_obj.w_slow,main_obj.w_fast)
    bound_cond_prob = max(0.0, 1.0 - (main_obj.w_fast / main_obj.w_slow))
    if bound_cond_prob != 0:
        for i in range(len(main_obj.particles)):
            bernoulli_num = np.random.binomial(1, bound_cond_prob, 1)
            print(bernoulli_num)
            # generate a random particle with probability bound_cond_prob
            if bernoulli_num == 1:
                main_obj.particles.append(main_obj.generate_random_particle(map_obj,True))
    else:
        # apply roulette wheel algorithm bounds
        for i in range(len(main_obj.particles)):
            lower_bound_prob.append(lower_cumsum)
            lower_cumsum = lower_cumsum + weights[i]
            upper_cumsum = upper_cumsum + weights[i]
            upper_bound_prob.append(upper_cumsum)
        for i in range(len(main_obj.particles)):
            random_num = np.random.uniform(0, 1, 1)
            for weight_idx in range(len(weights)):
                if (random_num >= float(lower_bound_prob[weight_idx])) & (random_num < float(upper_bound_prob[weight_idx])):
                    index = weight_idx

            selected_indexes[i] = index
        main_obj.particles = [main_obj.particles[i] for i in selected_indexes]
        main_obj.weights = [main_obj.weights[i] for i in selected_indexes]
    

def localized_particles(particles):
    cnt_20 = 0; cnt_15 =0; cnt_10 =0; cnt_5 =0
    for particle in particles:  
        if (np.sqrt((particle[0] - x) ** 2 + (particle[1] - y) ** 2) < 0.2):
            cnt_20 +=1
        if (np.sqrt((particle[0] - x) ** 2 + (particle[1] - y) ** 2) < 0.15):
            cnt_15 += 1
        if (np.sqrt((particle[0] - x) ** 2 + (particle[1] - y) ** 2) < 0.1):
            cnt_10 +=1
        if (np.sqrt((particle[0] - x) ** 2 + (particle[1] - y) ** 2) < 0.05):
             cnt_5 +=1
    print("Particles in 20 Cm From Robot ", cnt_20)
    print("Particles in 15 Cm From Robot ", cnt_15)
    print("Particles in 10 Cm From Robot ", cnt_10)
    print("Particles in 5  Cm From Robot ", cnt_5)
def generate_initial_pose():
    map_obj_ = World_Map()
    main_obj_ = Main_file()
    initial_pose = main_obj_.generate_random_particle(map_obj_, True)
    min_distance = map_obj_.calc_min_dist_to_obstacle(initial_pose)
    if min_distance > 0.15  :
        return initial_pose
    else:
        return generate_initial_pose()
    
    

def main():
    print("Hello World!")
    map_obj = World_Map()
    main_obj = Main_file()
    measurement_obj = Measurement_model()
    motion_obj = Motion_Model(0.001, 0.0032, 0.000009, 0.000009, map_obj)
    move = Move()

    # generate random particles for starting
   
    
    initialize_pose = generate_initial_pose()
    quaternion = quaternion_from_euler(0, 0, initialize_pose[2])
    print("initial pose: ", initialize_pose)
    initialize_robot_pose(initialize_pose, quaternion)
    print("step1: generate random particle")
    particles = main_obj.generate_random_particle(map_obj, False)
    main_obj.particles = particles
    read_position()
    r = rospy.Rate(1)  # 5 Hz
    r.sleep()
    main_obj.prev_x = x
    main_obj.prev_y = y
    main_obj.prev_theta = theta
    cnt = 0
    show_interval = 10
    check_left = False;   check_right = False; _update = True
    while not rospy.is_shutdown():
        try:
            # apply move function
            r = rospy.Rate(1)  # 5 Hz
            r.sleep()
            read_range_measurements()
            print("step2: move the robot")
            check_right, check_left, _update = move.move_in_map(check_right, check_left)  
            if(_update):
                main_obj.current_x = x
                main_obj.current_y = y
                main_obj.current_theta = theta
                # u_t controller command
                obj_curr_theta = np.copy(theta)
                if(main_obj.current_theta - main_obj.prev_theta<-np.pi/2):
                    obj_curr_theta = np.copy(2*np.pi + main_obj.current_theta)
                elif(main_obj.current_theta - main_obj.prev_theta>np.pi/2):
                    obj_curr_theta = np.copy(-1*2*np.pi + main_obj.current_theta)
                x_bar_t_1 = [main_obj.prev_x, main_obj.prev_y, main_obj.prev_theta]
                x_bar_t = [main_obj.current_x, main_obj.current_y, obj_curr_theta]
                u_t = [x_bar_t_1, x_bar_t]
                # apply motion model
                print("step3: apply motion model")
                main_obj.particles = motion_obj.motion_model(u_t, main_obj.particles)
                # read range measurement
                print("step4: apply sensor model")
                read_range_measurements()
                is_update = measurement_obj.range_finder_alg(sensor_range, main_obj, map_obj)
               # print("weights in body after beam:",main_obj.weights)
                # apply resampling.....
                print("step5: apply resampling")
                if(is_update):
                        randon_resampling(main_obj, map_obj)
                localized_particles(main_obj.particles)
                main_obj.prev_x = main_obj.current_x
                main_obj.prev_y = main_obj.current_y
                main_obj.prev_theta = main_obj.current_theta
                if show_interval % 1 == 0:
                    map_obj.plot_point_on_map(main_obj.particles)
                show_interval += 1
        except rospy.ROSInterruptException:
            pass

if __name__ == "__main__":
    main()