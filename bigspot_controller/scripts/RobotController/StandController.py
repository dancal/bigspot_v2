#!/usr/bin/env python3
#Author: lnotspotl

from cgi import print_arguments
import rospy
import numpy as np
import time
import threading

from RoboticsUtilities.Transformations import rotxyz
#from . PIDController import PID_controller
from . StateCommand import State, Command, BehaviorState

class StandController(object):
    def __init__(self, body, legs):
        self.body               = body
        self.legs               = legs

        self.max_reach          = 0.4
        self.delta_x            = self.body[0] * 0.5
        self.delta_y            = self.body[1] * 0.5 + self.legs[1]

        self.x_shift_front      = 0.
        self.x_shift_back       = -0.
        
        self._default_height    = 0.82
        self._before_height     = self._default_height
        self._slow_init         = False
        self._slow_init_ing     = False
        self._wakeup_secs       = 2.0

        self.state              = State(self.default_height, BehaviorState.STAND)
        self.command            = Command(self.default_height)

        self.FR_X = 0.
        self.FR_Y = 0.
        self.FL_X = 0.
        self.FL_Y = 0.

    def updateStateCommand(self,msg,state,command,motion):

        #state.body_local_position[0] = msg.axes[7] * 0.18
        state.body_local_position[0] = msg.axes[7] * 0.18

        self.FR_X = msg.axes[1]
        self.FR_Y = msg.axes[0]

        self.FL_X = msg.axes[4]
        self.FL_Y = msg.axes[3]

    def run(self,state,command):

        temp = self.default_stance
        temp[2] = [command.robot_height] * 4
        
        temp[1][0] += self.FR_Y * self.max_reach
        temp[0][0] += self.FR_X * self.max_reach

        temp[1][1] += self.FL_Y * self.max_reach
        temp[0][1] += self.FL_X * self.max_reach

        #temp[2] = [command.robot_height]

        state.foot_locations = temp
        return state.foot_locations
    
    @property
    def default_height(self):
        return self._default_height
    
    @property
    def default_stance(self):
        # FR, FL, RR, RL
        return np.array([[self.delta_x + self.x_shift_front,self.delta_x + self.x_shift_front,-self.delta_x + self.x_shift_back,-self.delta_x + self.x_shift_back],
                         [-self.delta_y                    ,self.delta_y                     ,-self.delta_y                    ,self.delta_y                    ],
                         [0                                ,0                                ,0                                ,0                                ]])
