#! /usr/bin/env python
import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
from pressure_controller_ros.msg import CommandAction, CommandGoal
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Pose as PoseMsg
from geometry_msgs.msg import Quaternion, Point

from tf.transformations import euler_from_quaternion, quaternion_from_euler

import time
import sys
import copy
import os
import numpy as np
import pandas as pd

#import sorotraj


class Controller:
    def __init__(self, pressure_server = "dynamixel"):
        self.pressure_server_name = pressure_server
        self.DEBUG = rospy.get_param(rospy.get_name()+"/DEBUG",False)              
        self.controller_rate = float(rospy.get_param(rospy.get_name()+"/controller_rate",30))

        self.num_channels = rospy.get_param('/dynamixel/num_channels',[])
        self.num_channels_total = sum(self.num_channels)

        self.config = rospy.get_param(rospy.get_name()+"/runs",None)
        self.speeds = self.config.get('speeds',[])    
        self.num_reps = self.config.get('num_reps',1)
        self.save_path = self.config.get('save_path',None)

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
        self.init_pose = None
        self.init_time = None

        self.is_shutdown=False
        self.is_init = False

        self.curr_arrange = None

        self.data = []
        self.curr_pose = {}
        self.detected = False
        self.save_data = False
        self.speed = 200


   
    
    def run_speed(self):          
        self.detected = True      
        if self.detected:
            print("Running Speed Test")
            for speed in self.speeds:
                self.send_command("speed",[float(speed)])
                self.speed = speed
                print("Speed: " + str(speed))
                for i in range(self.num_reps):
                    self.save_data = True
                    time.sleep(5)
                    self.send_command("set",[0,88])                    
                    time.sleep(5)
                    self.send_command("set",[0,0])
                    self.save_data = False                    
                    time.sleep(5)                    

            #Save data
            df = pd.DataFrame.from_dict(self.data, orient='columns')            
            file_name = self.save_path
            df.to_csv(file_name)            
            self.detected = False
    
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
        self.detected = True
        
        pose = detection.pose.pose.pose

        pos_raw = pose.position
        ori_raw = pose.orientation

        pos = np.array([pos_raw.x, pos_raw.y, pos_raw.z])
        ori = euler_from_quaternion([ori_raw.x, ori_raw.y, ori_raw.z, ori_raw.w])
        ori = np.array(ori)
        pose={'pos_x':pos[0], 'pos_y':pos[1],'pos_z':pos[2],'ori_x':ori[0],'ori_y':ori[1],'ori_z':ori[2]}
        if self.init_pose is None:
            self.init_pose = copy.deepcopy(pose)
            self.init_time = copy.deepcopy(time)

            self.last_pose = copy.deepcopy(pose)
            self.last_time = copy.deepcopy(time)
            return
        # Calculate relative time and pose
        time_rel = time - self.init_time

        pose_rel = {}
        for key in pose:
            pose_rel[key] = pose[key] - self.init_pose[key]
        self.curr_pose = pose_rel
        if self.save_data:
            self.curr_pose['time'] = time_rel
            self.curr_pose['speed'] = self.speed
            self.data.append(self.curr_pose)

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
        node.run_speed()
        print("Done")
        rospy.spin()
        
    except:
        raise

    # except KeyboardInterrupt:
    #     print("Keyboard Interrup Triggered")
    #     node.shutdown()

    # except rospy.ROSInterruptException:
    #     print("ROS is shutting down")
    #     node.shutdown()
        