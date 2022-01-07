#!/usr/bin/env python
#################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#################################################################################

# Authors: Gilbert #

from os import O_NOATIME
import rospy
import numpy as np
import math
from math import pi
from geometry_msgs.msg import Twist, Point, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from respawnGoal import Respawn

import time

class Env():
    def __init__(self, action_size):
        self.goal_x = 0
        self.goal_y = 0
        self.heading1 = 0
        self.heading2 = 0
        self.heading3 = 0
        self.action_size = action_size
        self.initGoal = True
        self.get_goalbox = False
        self.position1 = Pose()
        self.position2 = Pose()
        self.position3 = Pose()
        self._position1 = Pose()
        self._position2 = Pose()
        self._position3 = Pose()
        self.pub_cmd_vel1 = rospy.Publisher('tb3_0/cmd_vel', Twist, queue_size=5)
        self.sub_odom1 = rospy.Subscriber('tb3_0/odom', Odometry, self.getOdometry)
        self.pub_cmd_vel2 = rospy.Publisher('tb3_1/cmd_vel', Twist, queue_size=5)
        self.sub_odom2 = rospy.Subscriber('tb3_1/odom', Odometry, self.getOdometry)
        self.pub_cmd_vel3 = rospy.Publisher('tb3_2/cmd_vel', Twist, queue_size=5)
        self.sub_odom3 = rospy.Subscriber('tb3_2/odom', Odometry, self.getOdometry)
        self.reset_proxy = rospy.ServiceProxy('gazebo/reset_simulation', Empty)
        self.unpause_proxy = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
        self.pause_proxy = rospy.ServiceProxy('gazebo/pause_physics', Empty)
        self.respawn_goal = Respawn()
        self.cmd_vel1 = 0.15
        self.cmd_vel2 = 0.15
        self.cmd_vel3 = 0.15
        self.ang_vel1 = 0
        self.ang_vel2 = 0
        self.ang_vel3 = 0
        self.goal_y1 = 0.15
        self.goal_y2 = -0.15
        self.goal_y3 = 0.15
        self.yaw1 = 0.0
        self.yaw2 = 0.0
        self.yaw3 = 0.0

    def getGoalDistace(self, scan_topic):
        if scan_topic == "tb3_0/scan":
            goal_distance = round(math.hypot(self.goal_x - self.position1.x, self.goal_y - self.position1.y), 2)
        elif scan_topic == "tb3_1/scan":
            goal_distance = round(math.hypot(self.goal_x - self.position2.x, self.goal_y - self.position2.y), 2)
        elif scan_topic == "tb3_2/scan":
            goal_distance = round(math.hypot(self.goal_x - self.position3.x, self.goal_y - self.position3.y), 2)
        else:
            print("$%#$%GJDKAJFGLK")

        return goal_distance

    def getOdometry(self, odom):
        if odom.header.frame_id == "tb3_0/odom":
            self.position1 = odom.pose.pose.position
            self._position1 = odom.pose.pose.position
        elif odom.header.frame_id == "tb3_1/odom":
            self.position2 = odom.pose.pose.position
            self._position2 = odom.pose.pose.position
        elif odom.header.frame_id == "tb3_2/odom":
            self.position3 = odom.pose.pose.position
            self._position3 = odom.pose.pose.position
        else:
            print("rotlqkf")

        if odom.header.frame_id == "tb3_0/odom":
            orientation = odom.pose.pose.orientation
            orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
            _, _, yaw = euler_from_quaternion(orientation_list)
            self.yaw1 = yaw # by jm
            # print("1: ", self.yaw1)
        elif odom.header.frame_id == "tb3_1/odom":
            orientation = odom.pose.pose.orientation
            orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
            _, _, yaw = euler_from_quaternion(orientation_list)
            self.yaw2 = yaw # by jm
            # print("2: ", self.yaw2)
        elif odom.header.frame_id == "tb3_2/odom":
            orientation = odom.pose.pose.orientation
            orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
            _, _, yaw = euler_from_quaternion(orientation_list)
            self.yaw3 = yaw # by jm
            # print("3: ", self.yaw3)
        else:
            print("##########################")
        
        # print(self.yaw1, self.yaw2, self.yaw3)

        if odom.header.frame_id == "tb3_0/odom":
            goal_angle1 = math.atan2(self.goal_y - self.position1.y, self.goal_x - self.position1.x)
        elif odom.header.frame_id == "tb3_1/odom":
            goal_angle2 = math.atan2(self.goal_y - self.position2.y, self.goal_x - self.position2.x)
        elif odom.header.frame_id == "tb3_2/odom":
            goal_angle3 = math.atan2(self.goal_y - self.position3.y, self.goal_x - self.position3.x)
        else:
            print("##########################")

        
        
        if odom.header.frame_id == "tb3_0/odom":
            heading = goal_angle1 - self.yaw1
            # print(heading, goal_angle, yaw)

            if heading > pi:
                heading -= 2 * pi

            elif heading < -pi:
                heading += 2 * pi
            self.heading1 = round(heading, 2)
        elif odom.header.frame_id == "tb3_1/odom":
            heading = goal_angle2 - self.yaw2
            # print(heading, goal_angle, yaw)

            if heading > pi:
                heading -= 2 * pi

            elif heading < -pi:
                heading += 2 * pi
            self.heading2 = round(heading, 2)
        elif odom.header.frame_id == "tb3_2/odom":
            heading = goal_angle3 - self.yaw3
            # print(heading, goal_angle, yaw)

            if heading > pi:
                heading -= 2 * pi

            elif heading < -pi:
                heading += 2 * pi
            self.heading3 = round(heading, 2)
        else:
            print("tlqkflqjflkjdlkfjqlwdjkf")

    def getState(self, scan, scan_topic):
        scan_range = []
        if scan_topic == "tb3_0/scan":
            heading = self.heading1
        elif scan_topic == "tb3_1/scan":
            heading = self.heading2
        elif scan_topic == "tb3_2/scan":
            heading = self.heading3
        else:
            print("@@@@@@@@@@@@@")

        min_range = 0.13 # 0.13
        done = False

        # print(scan_topic, scan.ranges)
        for i in range(len(scan.ranges)):
            if scan.ranges[i] == float('Inf'):
                scan_range.append(3.5)
            elif np.isnan(scan.ranges[i]):
                scan_range.append(0)
            else:
                scan_range.append(scan.ranges[i])

        obstacle_min_range = round(min(scan_range), 2)
        obstacle_angle = np.argmin(scan_range)

        distance_between1 = math.sqrt((self.position1.x - self.position2.x) ** 2 + (self.position1.y - self.position2.y) ** 2)
        distance_between2 = math.sqrt((self.position1.x - self.position3.x) ** 2 + (self.position1.y - self.position3.y) ** 2)
        distance_between3 = math.sqrt((self.position3.x - self.position2.x) ** 2 + (self.position3.y - self.position2.y) ** 2)
        distance_between = min(distance_between1, distance_between2, distance_between3)
        # print("distance: ", distance_between)

        # print(self.position1.y, self.position2.y, self.position3.y)
        # print("1")
        # print(self.position1.x - self.position2.x, self.position1.y - self.position2.y)
        # print(self.position1.x - self.position3.x, self.position1.y - self.position3.y)
        # print("2")
        # print(self.position2.x - self.position1.x, self.position2.y - self.position1.y)
        # print(self.position2.x - self.position3.x, self.position2.y - self.position3.y)
        # print("3")
        # print(self.position3.x - self.position1.x, self.position3.y - self.position1.y)
        # print(self.position3.x - self.position2.x, self.position3.y - self.position2.y)
        state_rl = []
        if scan_topic == "tb3_0/scan":
            x12 = round(self.position1.x - self.position2.x, 2)
            if x12 > 1.0:
                x12 = 1.0
            elif x12 < -1.0:
                x12 = -1.0
            else:
                pass
            y12 = round(self.position1.y - self.position2.y, 2)
            h12 = round(self.heading1 - self.heading2, 2)
            x13 = round(self.position1.x - self.position3.x, 2)
            if x13 > 1.0:
                x13 = 1.0
            elif x13 < -1.0:
                x13 = -1.0
            else:
                pass
            y13 = round(self.position1.y - self.position3.y, 2)
            h13 = round(self.heading1 - self.heading3, 2)
            px = self.position1.x
            py = round(self.position1.y, 2)
            ph = round(self.heading1, 2)
            state_rl = [py, ph, x12, y12, h12, x13, y13, h13]
            # state_rl = [py, ph, x12, y12, x13, y13] # 22-01-07
            print(state_rl)
        elif scan_topic == "tb3_1/scan":
            x21 = round(self.position2.x - self.position1.x, 2)
            y21 = round(self.position2.y - self.position1.y, 2)
            h21 = round(self.heading2 - self.heading1, 2)
            x23 = round(self.position2.x - self.position3.x, 2)
            y23 = round(self.position2.y - self.position3.y, 2)
            h23 = round(self.heading2 - self.heading3, 2)
            px = self.position2.x
            py = round(self.position2.y, 2)
            ph = round(self.heading2, 2)
            state_rl = [py, ph, x21, y21, h21, x23, y23, h23]
            # state_rl = [py, ph, x21, y21, x23, y23]
        elif scan_topic == "tb3_2/scan":
            x31 = round(self.position3.x - self.position1.x, 2)
            y31 = round(self.position3.y - self.position1.y, 2)
            h31 = round(self.heading3 - self.heading1, 2)
            x32 = round(self.position3.x - self.position2.x, 2)
            y32 = round(self.position3.y - self.position2.y, 2)
            h32 = round(self.heading3 - self.heading2, 2)
            px = self.position3.x
            py = round(self.position3.y, 2)
            ph = round(self.heading3, 2)
            state_rl = [py, ph, x31, y31, h31, x32, y32, h32]
            # state_rl = [py, ph, x31, y31, x32, y32]
        else:
            print("&&&&&&&&&&&&&&&&")

        exit1 = self.position1.y<-0.25 or self.position1.y>0.25
        exit2 = self.position2.y<-0.25 or self.position2.y>0.25
        exit3 = self.position3.y<-0.25 or self.position3.y>0.25
        exit_ = exit1 or exit2 or exit3

        goal_done = self.position1.x > 5.0 # 22-01-05 

        if (min_range > min(scan_range) > 0) or distance_between < 0.21 or exit_ or goal_done:
            done = True
        # print("scan_range: ", min(scan_range))
        if scan_topic == "tb3_0/scan":
            current_distance = round(math.hypot(self.goal_x - self.position1.x, self.goal_y - self.position1.y),2)
        elif scan_topic == "tb3_1/scan":
            current_distance = round(math.hypot(self.goal_x - self.position2.x, self.goal_y - self.position2.y),2)
        elif scan_topic == "tb3_2/scan":
            current_distance = round(math.hypot(self.goal_x - self.position3.x, self.goal_y - self.position3.y),2)
        else:
            print("&&&&&&&&&&&&&&&&")

        if current_distance < 0.2:
        # if current_distance < 0.2 or goal_done: # 22-01-05 
            print("tq!!!!!!!!goal!!!!!!!!!")
            self.get_goalbox = True
        # print(len(scan_range))
        # return scan_range + [heading, current_distance, obstacle_min_range, obstacle_angle], done
        return state_rl , done, px # [0, 0] + [round(heading, 2), round(current_distance, 2)]

    def setReward(self, state, done, action, scan_topic):
        yaw_reward = []
        # obstacle_min_range = state[-2]
        current_distance = state[-1] # -1 # -3
        heading = state[-2] # -2 # -4

        for i in range(5):
            angle = -pi / 4 + heading + (pi / 8 * i) + pi / 2
            tr = 1 - 4 * math.fabs(0.5 - math.modf(0.25 + 0.5 * angle % (2 * math.pi) / math.pi)[0])
            # print(tr)
            yaw_reward.append(tr)

        # print(scan_topic)

        if scan_topic == "tb3_0/scan":
            # print('1')
            # distance_rate = 2 ** (current_distance / self.goal_distance1)
            # reward = self.position1.x * 10
            reward_y = self.position1.y - self.goal_y1
        elif scan_topic == "tb3_1/scan":
            # print('2')
            # distance_rate = 2 ** (current_distance / self.goal_distance2)
            # reward = self.position2.x * 10
            reward_y = self.position2.y - self.goal_y2
        elif scan_topic == "tb3_2/scan":
            # print('2')
            # distance_rate = 2 ** (current_distance / self.goal_distance3)
            # reward = self.position3.x * 10 
            reward_y = self.position3.y - self.goal_y3
        else:
            print("*********************")

        # if obstacle_min_range < 0.5:
        #     ob_reward = -5
        # else:
        #     ob_reward = 0

        # reward = ((round(yaw_reward[action] * 5, 2)) * distance_rate) + ob_reward
        # reward = ((round(yaw_reward[action] * 5, 2)) * distance_rate) # + ob_reward
        # reward = distance_rate
        # print(reward)

        reward = -1 # - abs(reward_y) # 22-01-05

        # if scan_topic == "tb3_0/scan":
        #     # self.reward1 = self.position1.x
        #     # reward = self.reward1 * 10 - abs(self.position1.y - self.goal_y1) * 100
        #     reward -= abs(self.position1.y - self.goal_y1) * 100
        # elif scan_topic == "tb3_1/scan":
        #     # self.reward2 = self.position2.x
        #     # reward = self.reward2 * 10 - abs(self.position2.y - self.goal_y2) * 100
        #     reward -= abs(self.position2.y - self.goal_y2) * 100
        # elif scan_topic == "tb3_2/scan":
        #     # self.reward3 = self.position3.x
        #     # reward = self.reward3 * 10 - abs(self.position3.y - self.goal_y3) * 100
        #     reward -= abs(self.position3.y - self.goal_y3) * 100
        # else:
        #     print("&&&&&&&&&&&&&&&&")

        if done:
            rospy.loginfo("Collision!!")
            # if self.position1.x > 5.0:
            #     print("goooooood")
            #     reward = 200
            # else:
            #     print("baaaaaaaaaaaad")
            reward = -20 # -200
            if scan_topic == "tb3_0/scan":
                self.pub_cmd_vel1.publish(Twist())
            elif scan_topic == "tb3_1/scan":
                self.pub_cmd_vel2.publish(Twist())
            elif scan_topic == "tb3_2/scan":
                self.pub_cmd_vel3.publish(Twist())
            else:
                print("77777777777777777777")

        if self.get_goalbox:
            rospy.loginfo("Goal!!")
            reward = 200

            if scan_topic == "tb3_0/scan":
                self.pub_cmd_vel1.publish(Twist())
            elif scan_topic == "tb3_1/scan":
                self.pub_cmd_vel2.publish(Twist())
            elif scan_topic == "tb3_2/scan":
                self.pub_cmd_vel3.publish(Twist())
            else:
                print("888888888888")

            self.goal_x, self.goal_y = self.respawn_goal.getPosition(True, delete=True)

            if scan_topic == "tb3_0/scan":
                self.goal_distance1 = self.getGoalDistace(scan_topic)
            elif scan_topic == "tb3_1/scan":
                self.goal_distance2 = self.getGoalDistace(scan_topic)
            elif scan_topic == "tb3_2/scan":
                self.goal_distance3 = self.getGoalDistace(scan_topic)
            else:
                print("(((((((((((((((((((((((")

            self.get_goalbox = False
            # done = True # 22-01-05
        # print(reward)
        return reward

    def pid(self, goal_y): # deleted current_y 
        # error = goal_y - current_y 
        # print(error, goal_y, current_y)
        p = 1.0
        i = 0.0
        d = 0.0
        y_g_pos = goal_y # let's check and just see the results
        y_c_pos = self.position1.y
        p_error = y_g_pos - y_c_pos
        y_error = 0 - self.yaw1 # and i think we can skip the dimension change, ok
        ang_error = math.atan2(p_error,0.3)
        error = 1.5 * y_error + ang_error # is it right? thanks
        # error = 1.5 * y_error + p_error
        result = p * error
        # print(result, error, p_error, y_error, ang_error)
        # print(result, error, p_error, y_error)

        if result < -0.001:
            if result < -0.2:
                if self.yaw1 < -pi/2:
                    result = 0.0
                else:
                    result = -0.6
        if result > 0.001:
            if result > 0.2:
                if self.yaw1 > pi/2:
                    result = 0.0
                else:
                    result = 0.6
        # if abs(p_error) > 0.05:
        #     if self.yaw < -pi/3:
        #         result = 0.0
        #     else:
        #         result = p_error * 2.0
        # else:
        #     result *= result
        return result # check this out together, ok

    def pid_2(self, goal_y): # deleted current_y 
        p = 1.0
        y_g_pos = goal_y # let's check and just see the results
        y_c_pos = self.position2.y
        p_error = y_g_pos - y_c_pos
        y_error = 0 - self.yaw2 # and i think we can skip the dimension change, ok
        ang_error = math.atan2(p_error,0.3)
        error = 1.5 * y_error + ang_error # is it right? thanks
        result = p * error
        return result # check this out together, ok

    def pid_3(self, goal_y): # deleted current_y 
        p = 1.0
        y_g_pos = goal_y # let's check and just see the results
        y_c_pos = self.position3.y
        p_error = y_g_pos - y_c_pos
        y_error = 0 - self.yaw3 # and i think we can skip the dimension change, ok
        ang_error = math.atan2(p_error,0.3)
        error = 1.5 * y_error + ang_error # is it right? thanks
        result = p * error
        return result # check this out together, ok

    def set_vel_cmd(self, vel_cmd, scan_topic):
        if scan_topic == "tb3_0/scan":
            self.cmd_vel1 = vel_cmd
        elif scan_topic == "tb3_1/scan":
            self.cmd_vel2 = vel_cmd
        elif scan_topic == "tb3_2/scan":
            self.cmd_vel3 = vel_cmd
        else:
            print("(((((((((((((((((((((((")

    def set_ang_vel(self, ang_vel, scan_topic):
        if scan_topic == "tb3_0/scan":
            self.ang_vel1 = ang_vel
        elif scan_topic == "tb3_1/scan":
            self.ang_vel2 = ang_vel
        elif scan_topic == "tb3_2/scan":
            self.ang_vel3 = ang_vel
        else:
            print("(((((((((((((((((((((((")

    def set_goal_y(self, goal_y, scan_topic):
        if scan_topic == "tb3_0/scan":
            self.goal_y1 = goal_y
        elif scan_topic == "tb3_1/scan":
            self.goal_y2 = goal_y
        elif scan_topic == "tb3_2/scan":
            self.goal_y3 = goal_y
        else:
            print("(((((((((((((((((((((((")

    # def straight(self, scan_topic):
    #     self.set_vel_cmd(0.1, scan_topic)
    #     self.set_ang_vel(0.0, scan_topic)
    #     # self.set_goal_y(0.15, scan_topic)

    # def accelerate(self, scan_topic):
    #     self.set_vel_cmd(0.15, scan_topic)
    #     # self.set_goal_y(0.15, scan_topic)

    # def decelerate(self, scan_topic):
    #     self.set_vel_cmd(0.05, scan_topic)
    #     # self.set_goal_y(0.15, scan_topic)

    # def stop(self, scan_topic):
    #     self.set_vel_cmd(0.0, scan_topic)
    #     # self.set_goal_y(0.15, scan_topic)

    # def turn_left(self, scan_topic): # if turn left, current y is -0.15 and target y is 0.15
    #     # self.set_vel_cmd(0.15, scan_topic)
    #     self.set_goal_y(0.15, scan_topic) # -0.15
    #     # self.set_ang_vel(-0.5, scan_topic) #
    #     # print(self.ang_vel2)

    # def turn_right(self, scan_topic): # we need to change here!
    #     # self.set_vel_cmd(0.15, scan_topic)
    #     self.set_goal_y(-0.15, scan_topic) # 0.15
    #     # self.set_ang_vel(0.5, scan_topic)

    ###############################################################################################

    # def straight_h(self, scan_topic):
    #     self.set_vel_cmd(0.15, scan_topic)
    #     # self.set_ang_vel(0.0, scan_topic)

    # def straight_m(self, scan_topic):
    #     self.set_vel_cmd(0.1, scan_topic)
    #     # self.set_ang_vel(0.0, scan_topic)

    # def straight_l(self, scan_topic):
    #     self.set_vel_cmd(0.05, scan_topic)
    #     # self.set_ang_vel(0.0, scan_topic)
    #     # self.set_goal_y(0.15, scan_topic)

    def turn_left_f(self, scan_topic):
        self.set_vel_cmd(0.20, scan_topic)
        # self.set_ang_vel(0.0, scan_topic)
        self.set_goal_y(0.15, scan_topic)

    def turn_left_h(self, scan_topic):
        self.set_vel_cmd(0.15, scan_topic)
        # self.set_ang_vel(0.0, scan_topic)
        self.set_goal_y(0.15, scan_topic)

    def turn_left_m(self, scan_topic):
        self.set_vel_cmd(0.1, scan_topic)
        # self.set_ang_vel(0.0, scan_topic)
        self.set_goal_y(0.15, scan_topic)

    def turn_left_l(self, scan_topic):
        self.set_vel_cmd(0.05, scan_topic)
        # self.set_ang_vel(0.0, scan_topic)
        self.set_goal_y(0.15, scan_topic)

    def turn_right_f(self, scan_topic):
        self.set_vel_cmd(0.20, scan_topic)
        # self.set_ang_vel(0.0, scan_topic)
        self.set_goal_y(-0.15, scan_topic)

    def turn_right_h(self, scan_topic):
        self.set_vel_cmd(0.15, scan_topic)
        # self.set_ang_vel(0.0, scan_topic)
        self.set_goal_y(-0.15, scan_topic)

    def turn_right_m(self, scan_topic):
        self.set_vel_cmd(0.1, scan_topic)
        # self.set_ang_vel(0.0, scan_topic)
        self.set_goal_y(-0.15, scan_topic)

    def turn_right_l(self, scan_topic):
        self.set_vel_cmd(0.05, scan_topic)
        # self.set_ang_vel(0.0, scan_topic)
        self.set_goal_y(-0.15, scan_topic)

    def stop(self, scan_topic):
        self.set_vel_cmd(0.0, scan_topic)
        # self.set_goal_y(0.15, scan_topic)

    # def back_l(self, scan_topic):
    #     self.set_vel_cmd(-0.05, scan_topic)
    #     # self.set_ang_vel(0.0, scan_topic)
    #     # self.set_goal_y(0.15, scan_topic)

    # def back_left_l(self, scan_topic):
    #     self.set_vel_cmd(-0.05, scan_topic)
    #     # self.set_ang_vel(0.0, scan_topic)
    #     self.set_goal_y(0.15, scan_topic)

    # def back_right_l(self, scan_topic):
    #     self.set_vel_cmd(-0.05, scan_topic)
    #     # self.set_ang_vel(0.0, scan_topic)
    #     self.set_goal_y(-0.15, scan_topic)

    ##############################################################################

    # def lane_change_left(self, scan_topic):
    #     # print("tlqkfjldjkfdjfkdjf")
    #     # self.set_vel_cmd(0.15, scan_topic)
    #     if scan_topic == "tb3_0/scan":
    #         if self.goal_y1 == -0.15:
    #             while self.position1.y < 0:
                   
    #                 self.turn_left(scan_topic)
    #             while self.position1.y < 0.15:
    #                 self.turn_right(scan_topic)
    #         else:
    #             pass
    #     elif scan_topic == "tb3_1/scan":
    #         print("tlqkf", self.goal_y2, self.position2.y)
    #         if self.goal_y2 == -0.15:
    #             print("tlqkf")
    #             while self.position2.y < 0:
    #                 print("tlqkf444")
    #                 self.turn_left(scan_topic)
    #                 time.sleep(1)
    #             while self.position2.y < 0.15:
    #                 self.turn_right(scan_topic)
    #         else:
    #             print("tlqkf2")
    #             pass
    #     elif scan_topic == "tb3_2/scan":
    #         if self.goal_y3 == -0.15:
    #             while self.position3.y < 0:
    #                 self.turn_left(scan_topic)
    #             while self.position3.y < 0.15:
    #                 self.turn_right(scan_topic)
    #         else:
    #             pass
    #     else:
    #         print("(((((((((((((((((((((((")
        
    # def lane_change_right(self, scan_topic):

    #     if scan_topic == "tb3_0/scan":
    #         if self.goal_y1 == 0.15:
    #             while self.position1.y > 0:
    #                 self.turn_right(scan_topic)
    #             while self.position1.y > -0.15:
    #                 self.turn_left(scan_topic)
    #         else:
    #             pass
    #     elif scan_topic == "tb3_1/scan":
    #         if self.goal_y2 == 0.15:
    #             while self.position2.y > 0:
    #                 self.turn_right(scan_topic)
    #             while self.position2.y > -0.15:
    #                 self.turn_left(scan_topic)
    #         else:
    #             pass
    #     elif scan_topic == "tb3_2/scan":
    #         if self.goal_y3 == 0.15:
    #             while self.position3.y > 0:
    #                 self.turn_right(scan_topic)
    #             while self.position3.y > -0.15:
    #                 self.turn_left(scan_topic)
    #         else:
    #             pass
    #     else:
    #         print("(((((((((((((((((((((((")

    # def lane_change_left(self, scan_topic):
    #     self.set_vel_cmd(0.15, scan_topic)
    #     self.set_goal_y(0.15, scan_topic)
    #     self.set_ang_vel(scan_topic)

    # def lane_change_right(self, scan_topic):
    #     self.set_vel_cmd(0.15, scan_topic)
    #     self.set_goal_y(-0.15, scan_topic)
    #     self.set_ang_vel(scan_topic)

    def step(self, action, scan_topic):
        max_angular_vel = 1.0 # 1.5
        # ang_vel = ((self.action_size - 1)/2 - action) * max_angular_vel * 0.5
        # actions = [self.straight, self.accelerate, self.decelerate, self.stop, self.turn_left, self.turn_right]
        actions = [self.turn_left_f, self.turn_left_h, self.turn_left_m, self.turn_left_l, self.turn_right_f, self.turn_right_h, self.turn_right_m, self.turn_right_l, self.stop]
        # actions = [self.turn_left_h, self.turn_left_m, self.turn_left_l, self.turn_right_h, self.turn_right_m, self.turn_right_l, self.stop, self.back_l, self.back_left_l, self.back_right_l]
        actions[action](scan_topic)
        # print(action)

        # if scan_topic == "tb3_0/scan":
        #     ang_vel = self.ang_vel1 + self.pid(self.goal_y1) # i think it's not
        #     # print(self.pid(0.15, self.position1.y))
        #     # print("1", ang_vel)
        # elif scan_topic == "tb3_1/scan":
        #     ang_vel = self.ang_vel2 + self.pid(self.goal_y2)
        #     # print("2", ang_vel)
        # elif scan_topic == "tb3_2/scan":
        #     ang_vel = self.ang_vel3 + self.pid(self.goal_y3) 
        #     # print("3", ang_vel)
        # else:
        #     print("tlqkf")
        
        # ang_vel = min(ang_vel, max_angular_vel)
        # self.set_ang_vel(scan_topic)

        # print(action)

        vel_cmd1 = Twist()
        vel_cmd2 = Twist()
        vel_cmd3 = Twist()
        # vel_cmd.linear.x = 0.15
        # vel_cmd.angular.z = ang_vel

        if scan_topic == "tb3_0/scan":
            vel_cmd1.linear.x = self.cmd_vel1 
            # vel_cmd.angular.z = self.ang_vel1
            vel_cmd1.angular.z = self.pid(self.goal_y1) # here
            # print(vel_cmd1.angular.z)
        elif scan_topic == "tb3_1/scan":
            vel_cmd2.linear.x = self.cmd_vel2
            vel_cmd2.angular.z = self.pid_2(-0.15) # self.ang_vel2
            # vel_cmd.angular.z = self.pid(self.goal_y2)
        elif scan_topic == "tb3_2/scan":
            vel_cmd3.linear.x = self.cmd_vel3
            vel_cmd3.angular.z = self.pid_3(0.15) # self.ang_vel3 
            # vel_cmd.angular.z = self.pid(self.goal_y3)
        else:
            print("tlqkf")

        # print(self.goal_y1, self.goal_y2, self.goal_y3)

        if scan_topic == "tb3_0/scan":
            self.pub_cmd_vel1.publish(vel_cmd1)
        elif scan_topic == "tb3_1/scan":
            self.pub_cmd_vel2.publish(vel_cmd2)
        elif scan_topic == "tb3_2/scan":
            self.pub_cmd_vel3.publish(vel_cmd3)
        else:
            print("tlqkf")

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message(scan_topic, LaserScan, timeout=5)
            except:
                pass

        time.sleep(0.1)

        state, done, px = self.getState(data, scan_topic)
        reward = self.setReward(state, done, action, scan_topic)

        # print(self.position1.x, self.position1.y)

        return np.asarray(state), reward, done, px

    def reset(self):
        rospy.wait_for_service('gazebo/reset_simulation')
        try:
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print("gazebo/reset_simulation service call failed")

        data1 = None
        while data1 is None:
            try:
                data1 = rospy.wait_for_message("tb3_0/scan", LaserScan, timeout=5)
            except:
                pass
        data2 = None
        while data2 is None:
            try:
                data2 = rospy.wait_for_message("tb3_1/scan", LaserScan, timeout=5)
            except:
                pass
        data3 = None
        while data3 is None:
            try:
                data3 = rospy.wait_for_message("tb3_2/scan", LaserScan, timeout=5)
            except:
                pass

        if self.initGoal:
            self.goal_x, self.goal_y = self.respawn_goal.getPosition()
            self.initGoal = False

        # if scan_topic == "tb3_0/scan":
        self.goal_distance1 = self.getGoalDistace("tb3_0/scan")
        # elif scan_topic == "tb3_1/scan":
        self.goal_distance2 = self.getGoalDistace("tb3_1/scan")
        self.goal_distance3 = self.getGoalDistace("tb3_2/scan")
        # else:
            # print("))))))))))))")

        state1, done1, px1 = self.getState(data1, "tb3_0/scan")
        state2, done2, px2 = self.getState(data2, "tb3_1/scan")
        state3, done3, px3 = self.getState(data3, "tb3_2/scan")

        return np.asarray(state1), np.asarray(state2), np.asarray(state3), px1, px2, px3