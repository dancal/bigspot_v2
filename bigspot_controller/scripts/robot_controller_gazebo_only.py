#!/usr/bin/env python3
#Author: lnotspotl

import rospy

from sensor_msgs.msg import Joy,Imu
from sensor_msgs.msg import Range

from RobotController import RobotController
from InverseKinematics import robot_IK
from std_msgs.msg import Float64

USE_IMU = True
RATE = 60

rospy.init_node("Robot_Controller")
 
# Robot geometry
#body                = [0.99, 0.25]
#legs                = [0.0, 0.04, 0.183, 0.2] 
body                = [0.88, 0.28]
legs                = [0.12, 0.11, 0.42, 0.42] 
#body                 = [0.366, 0.094]
#legs                 = [0.,0.08505, 0.2, 0.2] 

bigspot_robot       = RobotController.Robot(body, legs, USE_IMU)
inverseKinematics   = robot_IK.InverseKinematics(body, legs)

command_topics = ["/bigspot_controller/FRS_Joint/command",
                  "/bigspot_controller/FRL_Joint/command",
                  "/bigspot_controller/FRF_Joint/command",
                  "/bigspot_controller/FLS_Joint/command",
                  "/bigspot_controller/FLL_Joint/command",
                  "/bigspot_controller/FLF_Joint/command",
                  "/bigspot_controller/RRS_Joint/command",
                  "/bigspot_controller/RRL_Joint/command",
                  "/bigspot_controller/RRF_Joint/command",
                  "/bigspot_controller/RLS_Joint/command",
                  "/bigspot_controller/RLL_Joint/command",
                  "/bigspot_controller/RLF_Joint/command"]
                
publishers = []
for i in range(len(command_topics)):
    publishers.append(rospy.Publisher(command_topics[i], Float64, queue_size = 0))

#if USE_IMU:

rospy.Subscriber("bigspot_imu/base_link_orientation", Imu, bigspot_robot.imu_orientation)
rospy.Subscriber("bigspot_joy/joy_ramped", Joy, bigspot_robot.joystick_command)
#rospy.Subscriber("bigspot_ultrasonic/sonic_dist", Range, bigspot_robot.ultrasonic_command)

rate = rospy.Rate(RATE)

del body
del legs
del command_topics
del USE_IMU
del RATE

print("loop")
while not rospy.is_shutdown():
    leg_positions = bigspot_robot.run()
    bigspot_robot.change_controller()

    dx      = bigspot_robot.state.body_local_position[0]
    dy      = bigspot_robot.state.body_local_position[1]
    dz      = bigspot_robot.state.body_local_position[2]
    
    roll    = bigspot_robot.state.body_local_orientation[0]
    pitch   = bigspot_robot.state.body_local_orientation[1]
    yaw     = bigspot_robot.state.body_local_orientation[2]

    try:
        joint_angles = inverseKinematics.inverse_kinematics(leg_positions, dx, dy, dz, roll, pitch, yaw)
        for i in range(len(joint_angles)):
            #FR,                              ,FL,                              ,RR                               ,RL
            publishers[i].publish(joint_angles[i])
    except Exception as e:
        print(e)
        pass

    rate.sleep()
