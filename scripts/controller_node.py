#! /usr/bin/env python
import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
from pressure_controller_ros.msg import CommandAction, CommandGoal
from geometry_msgs.msg import Pose as PoseMsg
from geometry_msgs.msg import Quaternion, Point
from apriltag_ros.msg import AprilTagDetectionArray

from tf.transformations import euler_from_quaternion, quaternion_from_euler

import time
import copy
import sys
import os
import numpy as np

#import sorotraj


class Controller:
    def __init__(self, pressure_server = "dynamixel"):
        self.pressure_server_name = pressure_server
        self.DEBUG = rospy.get_param(rospy.get_name()+"/DEBUG",False)
        self.config = rospy.get_param(rospy.get_name()+"/controller",None)
        self.type = self.config.get('type',None)
        self.params = self.config.get('parameters',None)
        self.controller_rate = float(rospy.get_param(rospy.get_name()+"/controller_rate",30))

        self.num_channels = rospy.get_param('/dynamixel/num_channels',[])
        self.num_channels_total = sum(self.num_channels)

        for key in self.params:
            self.params[key] = np.array(self.params[key])

        # Connect to the pressure controller command server
        self.command_client = actionlib.SimpleActionClient(self.pressure_server_name, CommandAction)
        self.command_client.wait_for_server()
        self.set_data_stream(True)

        # Connect a callback function to send single desrired arrangement commands 
        arrange_topic = rospy.get_param(rospy.get_name()+"/arrange_topic",'/desired_arrange')
        


        # Connect a callback function to the tag detection topic
        tag_topic = rospy.get_param(rospy.get_name()+"/tag_topic",None)
        rospy.Subscriber(tag_topic, AprilTagDetectionArray, self.tag_callback)

        # Set up some parameters       
        self.is_shutdown=False
        self.is_init = False

   


    def send_setpoint(self, pressures, transition_time=None):

        if transition_time is None:
            transition_time = 1/self.controller_rate

        if isinstance(pressures, np.ndarray):
            pressures = pressures.tolist()

        psend=[transition_time]+pressures
        
        self.send_command('set', psend)


    def set_data_stream(self, value):
        if value:
            self.send_command('on', [])
        else:
            self.send_command('off', [])



    def send_command(self,cmd, args, wait_for_ack=False):
        goal = CommandGoal(command=cmd, args=args, wait_for_ack = wait_for_ack)
        self.command_client.send_goal(goal)
        self.command_client.wait_for_result()


    def tag_callback(self, msg): 
        #This function should be called each time the tag_topic gets something
        #It's result should be sending one arrangement value (in degrees)
        if self.is_shutdown:
            return

        detections = msg.detections

        # If no detection, do nothing
        if len(detections)==0:            
            return

        # If there's a detection, get the data

        time = msg.header.stamp.to_sec()
        detection = detections[0]
        #Get tag id
        tag_id = detection.id
        #From the tag id, get the arrangement
        arrange = [45] #This is where a function needs to be called that extracts the arrangemetn value
        
        
        # Send arrangement value
        if arrange is not None:            
            print(arrange)
            self.send_setpoint(arrange)
        elif not self.is_init:
            rest=self.params.get('rest',None) #TODO: Figure out what this does
            if rest is not None:
                self.send_setpoint(rest,1.0)
            self.is_init=True


    def shutdown(self):
        self.is_shutdown=True
        print('Setting all pressures to zero')
        self.send_setpoint([0]*self.num_channels_total, 2)
        time.sleep(2.2)
        print('Turning off data stream')
        self.set_data_stream(False)
        time.sleep(0.5)
        
        #self.command_client.cancel_all_goals()



if __name__ == '__main__':
    try:
        rospy.init_node('controller_node', disable_signals=False)
        node = Controller()
        rospy.on_shutdown(node.shutdown)
        rospy.spin()
    except:
        raise

    # except KeyboardInterrupt:
    #     print("Keyboard Interrup Triggered")
    #     node.shutdown()

    # except rospy.ROSInterruptException:
    #     print("ROS is shutting down")
    #     node.shutdown()
        