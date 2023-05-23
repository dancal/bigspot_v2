#!/usr/bin/evn python3
#Author: lnotspotl

import numpy as np
import tf
import rospy

from sensor_msgs.msg import Range
from sensor_msgs.msg import Joy

from . StateCommand import State, Command, BehaviorState, Motion
from . RestController import RestController
from . TrotGaitController import TrotGaitController
from . CrawlGaitController import CrawlGaitController
from . StandController import StandController
from . LieController import LieController

from RobotSensors.RgbSensorController import RgbSensorController

## gazebo
# roslaunch bigspot run_gazebo_only.launch
# roslaunch bigspot_joystick ramped_keyboard.launch
# roslaunch bigspot_description controller_gazebo.launch

# <limit>
# effort: 힘 (N)
# lower, upper: 최소, 최대 각도(radian)
# velocity: 속도(rad/s)

class Robot(object):
    def __init__(self, body, legs, imu):
        self.body                   = body
        self.legs                   = legs

        self.standController        = StandController(self.body, self.legs)
        self.lieController          = LieController(self.body, self.legs)
        self.restController         = RestController(self.body, self.legs)
        self.trotGaitController     = TrotGaitController(self.body, self.legs, stance_time = 0.18, swing_time = 0.24, time_step = 0.02, use_imu = imu)
        self.crawlGaitController    = CrawlGaitController(self.body, self.legs, stance_time = 0.55, swing_time = 0.45, time_step = 0.02)

        self.motion                 = Motion()

        #self.currentController      = self.lieController
        self.currentController      = self.restController
        self.state                  = self.currentController.state
        self.command                = self.currentController.command
        self.state.foot_locations   = self.currentController.default_stance
        self.state.behavior_state   = BehaviorState.REST

    def change_controller(self):
        before_robo_height          = self.currentController.default_height
        before_behavior_state       = self.currentController.state.behavior_state

        if self.motion.lie_event:
            # LIE
            if before_behavior_state == BehaviorState.REST:
                
                self.currentController      = self.lieController
                self.state                  = self.currentController.state
                self.command                = self.currentController.command
                self.state.foot_locations   = self.currentController.default_stance
                self.state.behavior_state   = BehaviorState.LIE
                self.state.ticks            = 0
                self.currentController.slowInitStart(before_robo_height)

            self.motion.lie_event  = False

        elif self.motion.rest_event:
            
            #if before_behavior_state == BehaviorState.REST:
            #    return
            
            # REST
            self.currentController      = self.restController
            self.state                  = self.currentController.state
            self.command                = self.currentController.command
            self.state.foot_locations   = self.currentController.default_stance
            self.state.behavior_state   = BehaviorState.REST
            self.state.ticks            = 0

            if before_behavior_state == BehaviorState.LIE:
                self.currentController.slowInitStart(before_robo_height)

            self.currentController.pid_controller.reset()
            
            self.motion.rest_event     = False

        elif self.motion.trot_event:
            # TROT
            if before_behavior_state == BehaviorState.REST:
                self.currentController      = self.trotGaitController
                self.state                  = self.currentController.state
                self.command                = self.currentController.command
                self.state.foot_locations   = self.currentController.default_stance                
                self.state.behavior_state   = BehaviorState.TROT
                self.state.ticks            = 0

                self.currentController.pid_controller.reset()

            self.motion.trot_event = False

        elif self.motion.crawl_event:
            # CRAWL
            if before_behavior_state == BehaviorState.REST:
                self.currentController      = self.crawlGaitController
                self.state                  = self.currentController.state
                self.command                = self.currentController.command
                self.state.foot_locations   = self.currentController.default_stance     
                self.state.behavior_state   = BehaviorState.CRAWL
                self.currentController.first_cycle = True
                self.state.ticks = 0

            self.motion.crawl_event = False

        elif self.motion.stand_event:
            # STAND
            if before_behavior_state == BehaviorState.REST:

                self.currentController      = self.standController
                self.state                  = self.currentController.state
                self.command                = self.currentController.command
                self.state.foot_locations   = self.currentController.default_stance    
                self.state.behavior_state   = BehaviorState.STAND
                self.state.ticks            = 0
                self.state.body_local_position[2] = 0.08 * self.currentController.default_height
                
            self.motion.stand_event = False


    def joystick_command(self,msg):

        if msg.buttons[0]: # rest - ok
            self.motion.trot_event     = False
            self.motion.crawl_event    = False
            self.motion.stand_event    = False
            self.motion.rest_event     = True
            self.motion.lie_event      = False

        elif msg.buttons[1]: # trot - ok
            self.motion.trot_event     = True
            self.motion.crawl_event    = False
            self.motion.stand_event    = False
            self.motion.rest_event     = False
            self.motion.lie_event      = False

        elif msg.buttons[7]: # stand
            self.motion.trot_event     = False
            self.motion.crawl_event    = False
            self.motion.stand_event    = True
            self.motion.rest_event     = False
            self.motion.lie_event      = False

        elif msg.buttons[3]: # lie - ok
            self.motion.trot_event     = False
            self.motion.crawl_event    = False
            self.motion.stand_event    = False
            self.motion.rest_event     = False
            self.motion.lie_event      = True

        elif msg.buttons[4]: # crawl - ok
            self.motion.trot_event     = False
            self.motion.crawl_event    = True
            self.motion.stand_event    = False
            self.motion.rest_event     = False
            self.motion.lie_event      = False
            
        self.currentController.updateStateCommand(msg, self.state, self.command, self.motion)

    def imu_orientation(self,msg):
        q = msg.orientation
        rpy_angles = tf.transformations.euler_from_quaternion([q.x,q.y,q.z,q.w])
        self.state.imu_roll = rpy_angles[0]
        self.state.imu_pitch = rpy_angles[1]
        #print("imu_orientation = ", rpy_angles)

    def run(self):
        return self.currentController.run(self.state, self.command)

    #@property
    #def default_stance(self):
    #    # FR, FL, RR, RL
    #    return np.array([[self.delta_x + self.x_shift_front,self.delta_x + self.x_shift_front,-self.delta_x + self.x_shift_back,-self.delta_x + self.x_shift_back],
    #                     [-self.delta_y                    ,self.delta_y                     ,-self.delta_y                    ,self.delta_y                    ],
    #                     [0                                ,0                                ,0                                ,0                                ]])
