#! /usr/bin/env python3
# -*- coding: utf-8 -*-
#
# This file presents an interface for interacting with the Playstation 4 Controller
# in Python. Simply plug your PS4 controller into your computer using USB and run this
# script!
#
# NOTE: I assume in this script that the only joystick plugged in is the PS4 controller.
#       if this is not the case, you will need to change the class accordingly.
#
# Copyright © 2015 Clay L. McLeod <clay.l.mcleod@gmail.com>
#
# Distributed under terms of the MIT license.

import rospy
import os
import pprint
import time
from math import fabs
from numpy import array_equal

import pygame

from sensor_msgs.msg import Joy
from std_msgs.msg import String
from std_msgs.msg import Float32

class PS4Controller(object):
    """Class representing the PS4 controller. Pretty straightforward functionality."""

    controller = None
    axis_data = None
    button_data = None
    hat_data = None

    def init(self):
        """Initialize the joystick components"""
        
        pygame.init()
        pygame.joystick.init()
        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()

        rospy.init_node("Joystick_ramped")
        #rospy.Subscriber("/joy", Joy, self.callback)

        self.publisher_joy              = rospy.Publisher("bigspot_joy/joy_ramped", Joy, queue_size = 10)
        #self.publisher_lcd_joy_speed    = rospy.Publisher("bigspot_lcd/joy_speed", String, queue_size = 3)
        #self.publisher_rgb              = rospy.Publisher('bigspot_rgb/rgb_dist', Float32, queue_size = 1)

        # ultrasonic
        #rospy.Subscriber("bigspot_ultrasonic/sonic_dist", Joy, self.callback_sonic)
        
        self.rate = rospy.Rate(60)

        self.backup_joy = Joy()

        # target
        self.target_joy = Joy()
        self.target_joy.axes = [0.,0.,1.,0.,0.,1.,0.,0.]
        self.target_joy.buttons = [0,0,0,0,0,0,0,0,0,0,0]

        # last
        self.last_joy = Joy()
        self.last_joy.axes = [0.,0.,1.,0.,0.,1.,0.,0.]
        self.last_joy.buttons = [0,0,0,0,0,0,0,0,0,0,0]
        self.last_send_time = rospy.Time.now()

        self.use_button = True

        self.speed_index = 1
        self.available_speeds = [1.0, 2.0, 3.0, 4.0, 5.0]

        rospy.loginfo(f"PS4 Joystick start")

    def callback(self, msg):
        if self.use_button:
            if msg.buttons[4]:
                self.speed_index -= 1
                if self.speed_index < 0:
                    self.speed_index = len(self.available_speeds) - 1
                rospy.loginfo(f"Joystick speed:{self.available_speeds[self.speed_index]}")
                self.use_button = False
            elif msg.buttons[5]:
                self.speed_index += 1
                if self.speed_index >= len(self.available_speeds):
                    self.speed_index = 0
                rospy.loginfo(f"Joystick speed:{self.available_speeds[self.speed_index]}")
                self.use_button = False

            #self.publisher_lcd_joy_speed.publish(f"{self.available_speeds[self.speed_index]}")

        if not self.use_button:
            if not(msg.buttons[4] or msg.buttons[5]):
                self.use_button = True

        self.target_joy.axes    = msg.axes
        self.target_joy.buttons = msg.buttons
        self.backup_joy         = self.target_joy

    def ramped_vel(self,v_prev,v_target,t_prev,t_now):
        # This function was originally not written by me:
        # https://github.com/osrf/rosbook/blob/master/teleop_bot/keys_to_twist_with_ramps.py
        step = (t_now - t_prev).to_sec()
        sign = self.available_speeds[self.speed_index] if \
                (v_target > v_prev) else -self.available_speeds[self.speed_index]
        error = fabs(v_target - v_prev)

        # if we can get there within this timestep -> we're done.
        if error < self.available_speeds[self.speed_index]*step:
            return v_target
        else:
            return v_prev + sign * step # take a step toward the target

    def publish_joy(self):
        t_now = rospy.Time.now()

        # determine changes in state
        buttons_change = array_equal(self.last_joy.buttons, self.target_joy.buttons)
        axes_change = array_equal(self.last_joy.axes, self.target_joy.axes)

        # if the desired value is the same as the last value, there's no
        # need to publish the same message again
        if not(buttons_change and axes_change):
            # new message
            joy = Joy()
            if not axes_change:
                # do ramped_vel for every single axis
                for i in range(len(self.target_joy.axes)): 
                    if self.target_joy.axes[i] == self.last_joy.axes[i]:
                        joy.axes.append(self.last_joy.axes[i])
                    else:
                        joy.axes.append(self.ramped_vel(self.last_joy.axes[i],self.target_joy.axes[i],self.last_send_time,t_now))
            else:
                joy.axes = self.last_joy.axes

            joy.buttons = self.target_joy.buttons
            self.last_joy = joy
            self.publisher_joy.publish(self.last_joy)

    def listen(self):
        """Listen for events to happen"""
        
        if not self.axis_data:
            self.axis_data = {}

        if not self.button_data:
            self.button_data = {}
            for i in range(self.controller.get_numbuttons()):
                self.button_data[i] = False

        if not self.hat_data:
            self.hat_data = {}
            for i in range(self.controller.get_numhats()):
                self.hat_data[i] = (0, 0)

        self.axis_data = [0.,0.,1.,0.,0.,1.,0.,0.]
        self.button_data = [0,0,0,0,0,0,0,0,0,0,0]

        while not rospy.is_shutdown():
            for event in pygame.event.get():
                if event.type == pygame.JOYAXISMOTION:
                    self.axis_data[event.axis] = round(event.value,3)
                elif event.type == pygame.JOYBUTTONDOWN:
                    self.button_data[event.button] = True
                elif event.type == pygame.JOYBUTTONUP:
                    self.button_data[event.button] = False
                elif event.type == pygame.JOYHATMOTION:
                    self.hat_data[event.hat] = event.value

                # Insert your code on what you would like to happen for each event here!
                # In the current setup, I have the state simply printing out to the screen.
                joy                 = Joy()
                joy.header.stamp    = rospy.Time.now()
                joy.axes            = self.axis_data
                joy.buttons         = self.button_data
                self.callback(joy)
                self.publish_joy()
                #self.publisher_joy.publish(joy)
                #os.system('clear')
                self.rate.sleep()
                print(joy)
                #pprint.pprint(self.button_data)
                #pprint.pprint(self.axis_data)
                #pprint.pprint(self.hat_data)


if __name__ == "__main__":
    ps4 = PS4Controller()
    ps4.init()
    ps4.listen()