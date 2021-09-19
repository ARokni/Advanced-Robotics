#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Range
import pandas as pd

sensor_range = 0
count = 0
measurement_list = []
import os

os.chdir(r'/home/sanaz/catkin_ws/src/advance_robotic_tutorial/src')
file = open('sample_range_30cm_x.txt', 'a')

def save(msg):
    global sensor_range
    global count
    global measurement_list
    global file
    count = count + 1
    with open('sample_range_30cm_x.txt', 'a'):
        file.write(str(msg.range)+'\n')
        print(count)
    if count > 999:
        # with open('sample_range_30cm_y.txt', 'a'):
            print(count)
            file.close()


rospy.init_node("range_save")
sub = rospy.Subscriber("/vector/laser", Range, save)
r = rospy.Rate(5)

while not rospy.is_shutdown():
    if count == 10:
        print(count)
        # print(measurement_list)
        # df = pd.DataFrame({'Data': measurement_list})
        # print(df)
        # df.to_excel('pandas_simple.xlsx', sheet_name='Sheet1')
        # writer.save()

    r.sleep()
