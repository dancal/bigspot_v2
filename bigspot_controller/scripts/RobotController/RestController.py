#!/usr/bin/env python3
#Author: lnotspotl

import numpy as np
import tf
import rospy
import time
import threading as th  

from sensor_msgs.msg import Range
from sensor_msgs.msg import Joy

from RoboticsUtilities.Transformations import rotxyz
from . PIDController import PID_controller
from . StateCommand import State, Command, BehaviorState

from std_msgs.msg import String


class RestController(object):
    def __init__(self, body, legs):
        self.body = body
        self.legs = legs
                
        self.slow_init          = False
        self.delta_x            = self.body[0] * 0.5
        self.delta_y            = self.body[1] * 0.5 + self.legs[1]

        self.x_shift_front      = 0.08
        self.x_shift_back       = -0.16
        self._default_height    = 0.82
        self._before_height     = self._default_height

        self._wakeup_secs       = 2.0
        self._slow_init         = False
        self._slow_init_ing     = False

        self.state              = State(self.default_height, BehaviorState.REST)
        self.command            = Command(self.default_height)

        # TODO: tune kp, ki and kd
        #                                     kp     ki    kd
        # pid: {p: 100.0, i: 0.01, d: 10.0}
        self.pid_controller = PID_controller(0.75, 2.29, 0.0)
        self.use_imu        = True
        self.use_button     = True
        self.pid_controller.reset()
        
        self.publisher_lcd_use_imu  = rospy.Publisher("bigspot_lcd/use_imu", String, queue_size = 10)

    @property
    def default_height(self):
        return self._default_height

    def slowInitStart(self, before_height):
        if self._slow_init_ing:
            return
        
        self._slow_init_ing = True
        self._slow_init      = True
        self._before_height  = before_height
        S = th.Timer(self._wakeup_secs, self.slowInitDone)
        S.start()  
 
    def slowInitDone(self):
        self._slow_init      = False
        self._slow_init_ing = False

    def updateStateCommand(self, msg, state, command, motion):

        # local body position
        state.body_local_position[0] = msg.axes[7] * 0.08
        state.body_local_position[1] = msg.axes[6] * 0.2
        state.body_local_position[2] = msg.axes[1] * 0.2

        # local body orientation
        state.body_local_orientation[0] = msg.axes[0] * 0.4
        state.body_local_orientation[1] = msg.axes[4] * 0.5
        state.body_local_orientation[2] = msg.axes[3] * 0.4

        if self.use_button:
            if msg.buttons[7]:
                self.use_imu = not self.use_imu
                self.use_button = False
                self.publisher_lcd_use_imu.publish(f"{self.use_imu}")
                rospy.loginfo(f"Rest Controller - Use roll/pitch compensation: {self.use_imu}")

        if not self.use_button:
            if not (msg.buttons[7]):
                self.use_button = True

    def step(self, state, command):

        if self._slow_init:
            self.state.ticks += 0.1
            robo_height = self._before_height + (self.default_height * self.state.ticks)
            # print("REST = ", robo_height, self._before_height, self.state.ticks, self._slow_init, self._slow_init_ing)

            if robo_height >= self.default_height:
                robo_height = self.default_height
            else:  
                time.sleep(0.15)
        else:
            state.ticks = 1
            robo_height = self._default_height
        

        temp    = self.default_stance
        temp[2] = [-robo_height] * 4

        # roll and pitch compensation
        # if self.use_imu == True, the robot tries to keep its body horizontal
        # using a PID controller
        if self.use_imu:
            compensation = self.pid_controller.run(state.imu_roll, state.imu_pitch)
            
            roll_compensation = -compensation[0]
            pitch_compensation = -compensation[1]

            rot = rotxyz(roll_compensation,pitch_compensation,0)
            temp = np.matmul(rot,temp)

        return temp

    def run(self, state, command):
        state.foot_locations = self.step(state, command)
        return state.foot_locations

    @property
    def default_stance(self):
        # FR, FL, RR, RL
        return np.array([[self.delta_x + self.x_shift_front,self.delta_x + self.x_shift_front,-self.delta_x + self.x_shift_back,-self.delta_x + self.x_shift_back],
                         [-self.delta_y                    ,self.delta_y                     ,-self.delta_y                    ,self.delta_y                    ],
                         [0                                ,0                                ,0                                ,0                                ]])

