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
#from . PIDController import PID_controller
from . StateCommand import State, Command, BehaviorState

from std_msgs.msg import String


class LieController(object):
    def __init__(self, body, legs):
        self.body               = body
        self.legs               = legs

        self.delta_x            = self.body[0] * 0.5
        self.delta_y            = self.body[1] * 0.5 + self.legs[1]

        self.x_shift_front      = 0.
        self.x_shift_back       = -0.
        
        self._default_height    = 0.25
        self._before_height     = self._default_height
        self._slow_init         = False
        self._slow_init_ing     = False
        self._wakeup_secs       = 2.0

        self.state              = State(self.default_height, BehaviorState.LIE)
        self.command            = Command(self.default_height)
        
    @property
    def default_height(self):
        return self._default_height

    def slowInitStart(self, before_height):
        if self._slow_init_ing:
            return
        
        self._slow_init_ing  = True
        self._slow_init      = True
        self._before_height  = before_height
        S = th.Timer(self._wakeup_secs, self.slowInitDone)
        S.start()  

    def slowInitDone(self):
        self._slow_init     = False
        self._slow_init_ing = False

    def updateStateCommand(self, msg, state, command, motion):

        state.body_local_position[0] = msg.axes[7] * 0.18

        self.FR_X = msg.axes[1]
        self.FR_Y = msg.axes[0]

        self.FL_X = msg.axes[4]
        self.FL_Y = msg.axes[3]

    def step(self, state, command):
        if self._slow_init:
            self.state.ticks += 0.1
            robo_height = self._before_height - (self._before_height * self.state.ticks)
            #print("LIE = ", robo_height, self._before_height, self.state.ticks, self._slow_init, self._slow_init_ing)

            if robo_height <= self.default_height:
                robo_height = self.default_height
            else:  
                time.sleep(0.15)
        else:
            state.ticks = 1
            robo_height = self._default_height
        

        temp    = self.default_stance
        temp[2] = [-robo_height] * 4

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
