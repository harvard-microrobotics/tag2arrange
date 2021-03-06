#! /usr/bin/env python
import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
from pressure_controller_ros.msg import CommandAction, CommandGoal
from apriltag_ros.msg import AprilTagDetectionArray

import time
import sys
import os
import numpy as np

import sorotraj
from pressure_controller_ros.msg import *
from pressure_controller_ros.live_traj_new import trajSender as pneu_traj_sender

import rospkg
from simple_ur_move.cartesian_trajectory_handler import CartesianTrajectoryHandler
import simple_ur_move.utils as utils

class Controller:
    def __init__(self, pressure_server = "dynamixel"):
        self.pressure_server_name = pressure_server
        self.DEBUG = rospy.get_param(rospy.get_name()+"/DEBUG",False)
        self.config = rospy.get_param(rospy.get_name()+"/items",None)
        self.items = self.config.get('items',None)        
        self.controller_rate = float(rospy.get_param(rospy.get_name()+"/controller_rate",30))

        self.num_channels = rospy.get_param('/pressure_control/num_channels',[])
        self.num_channels_total = sum(self.num_channels)

        #Load Finger Trajectory information 
        #self.traj_profile = rospy.get_param(rospy.get_name()+'/traj_profile')
        self.pre_grasp = rospy.get_param(rospy.get_name()+'/pre_grasp',None)
        self.start_grasp = rospy.get_param(rospy.get_name()+'/start_grasp',None)
        self.end_grasp = rospy.get_param(rospy.get_name()+'/end_grasp',None)  
        self.speed_factor = rospy.get_param(rospy.get_name()+'/speed_factor',1.0)
        self.num_reps = rospy.get_param(rospy.get_name()+'/num_reps',1)

        # Connect to the motor controller command server

        self.command_client = actionlib.SimpleActionClient(self.pressure_server_name, CommandAction)
        self.command_client.wait_for_server()
        self.set_data_stream(True)

        # Connect to the pressure controller command server
        self.p_ctrl = rospy.get_param(rospy.get_name()+"/pctrl",False)
        if self.p_ctrl:            
            self.command_client_p = actionlib.SimpleActionClient("pressure_control", CommandAction)
            self.command_client_p.wait_for_server()
            self.set_data_stream(True)

        self.arm_on = rospy.get_param(rospy.get_name()+"/arm",False)
        #self.arm_traj = rospy.get_param(rospy.get_name()+'/arm_traj_profile')
        if self.arm_on:
            self.traj_handler = CartesianTrajectoryHandler(
                name="",
                controller="pose_based_cartesian_traj_controller",
                debug=False)
            self.start_pick = rospy.get_param(rospy.get_name()+'/start_pick',None)
            self.end_place= rospy.get_param(rospy.get_name()+'/end_place',None)
            self.finish_pnp = rospy.get_param(rospy.get_name()+'/finish_pnp',None)
            
        # Connect a callback function to send single desrired arrangement commands 
        arrange_topic = rospy.get_param(rospy.get_name()+"/arrange_topic",'/desired_arrange')
        


        # Connect a callback function to the tag detection topic
        tag_topic = rospy.get_param(rospy.get_name()+"/tag_topic",None)
        rospy.Subscriber(tag_topic, AprilTagDetectionArray, self.tag_callback)

        # Set up some parameters       
        self.is_shutdown=False
        self.is_init = False
        self.curr_arrange = None
   
    def get_arrange(self,tag_id):
        #Check tag id is in items
        #If it is in items, extract the arrangement value and return        
        return self.items[str(tag_id)]['arrange']
    def get_height(self,tag_id):
        #Check tag id is in items
        #If it is in items, extract the height value and return        
        return self.items[str(tag_id)]['height']
    def send_setpoint(self, pressures, transition_time=None, p_ctrl=False):

        if transition_time is None:
            transition_time = 1/self.controller_rate

        if isinstance(pressures, np.ndarray):
            pressures = pressures.tolist()

        psend=[transition_time]+pressures
        
        
        if p_ctrl:
            self.send_command('set', psend, p_ctrl=p_ctrl)
        else:
            self.send_command('set', psend)


    def set_data_stream(self, value):
        if value:
            self.send_command('on', [])
        else:
            self.send_command('off', [])



    def send_command(self,cmd, args, wait_for_ack=False, p_ctrl=False):
        '''
            Sends the command to the command client (pressure controller or arrangement)
        '''
        goal = CommandGoal(command=cmd, args=args, wait_for_ack = wait_for_ack)                
        if p_ctrl and self.p_ctrl: 
            print(goal)           
            self.command_client_p.send_goal(goal)
            self.command_client_p.wait_for_result()
        else:
            print(goal)
            self.command_client.send_goal(goal)
            self.command_client.wait_for_result()


    def tag_callback(self, msg): 
        '''
            Callback function used by ROS each time the tag topic recieves something
            This function is the main controller of the system. 
            It reads the tag. Sends the arrangement, pressure and arm trajectory.
        '''
        
        if self.is_shutdown:
            return

        detections = msg.detections

        # If no detection, do nothing
        if len(detections)==0:            
            return

        # If there's a detection, get the data

        time_ = msg.header.stamp.to_sec()
        detection = detections[0]
        #Get tag id
        tag_id = detection.id
        #From the tag id, get the arrangement        
        arrange = self.get_arrange(tag_id[0]) #[0] is because tag_id comes as a tuple
        arrange = [arrange] #For formatting, should probably change later
        
        if self.arm_on:
            self.height = self.get_height(tag_id[0])
            #self.height = 0.05
        
        # Send arrangement value
        if arrange is not None:       
            if arrange != self.curr_arrange:     
                raw_input("New Item Detected. Do you want to change arrangements?")

                self.send_setpoint(arrange)
                self.curr_arrange = arrange
                if self.p_ctrl:
                    time.sleep(0.5)
                    self.run_traj(self.pre_grasp)
                    if self.arm_on:
                        print("Move to item")
                        #Run trajectory to get in position  
                        self.run_armtraj(self.start_pick,modify=True,step=0) 
                    print("Grasp Item")                     
                    self.run_traj(self.start_grasp)
                    
                    if self.arm_on:
                        print("Move the item to box")
                        #After grasping, 
                        self.run_armtraj(self.end_place,modify=True,step=1)
                    print("Release item")    
                    self.run_traj(self.end_grasp)
                    if self.arm_on:
                        print("Move back to start")
                        #After grasping, 
                        self.run_armtraj(self.finish_pnp,modify=True,step=2)

                    
        elif not self.is_init:
            rest=self.params.get('rest',None) #TODO: Figure out what this does
            if rest is not None:
                self.send_setpoint(rest,1.0)
            self.is_init=True

    def run_traj(self,traj):
        '''
            Run pressure trajectory for soft robotic fingers using sorotraj
        '''
        # Make a sotortaj traj builder
        builder = sorotraj.TrajBuilder() # Make a traj-builder object

        # Load a trajectory from a file and make some modifications on the fly
        file_to_use = traj
        builder.load_traj_def(file_to_use)
        traj_def = builder.get_definition()        
        builder.set_definition(traj_def) # Reset the definition (this also rebuilds the trajectory)

        # Alternatively, you can forgo the "load from file" step and set the trajectory
        # definition directly if you have already loaded/created it.
        # traf_def = {SOME DICTIONARY} 
        # builder.set_definition(traj_def) 

        # Get the trajectory in "sorotraj" format
        traj = builder.get_trajectory()
        traj_use = traj['setpoints']
        hand_sender = pneu_traj_sender(self.speed_factor) # Make a traj-sender object
        traj_ros = hand_sender.build_traj(traj_use) # Convert the trajectory to a ROS trajectory
        hand_sender.execute_traj(traj_ros, blocking=False) # Send the trajectory
        hand_sender.traj_client.wait_for_result() # wait until the trajectory is finished
    def run_armtraj(self,traj,modify=False,step=0):
        '''
            Run motion trajectory for robotic arm
        '''
        
        # OR Set trajectory config directly
        #config={TRAJECTORY CONFIG DICT}
        #traj_handler.set_config(config)
        if modify:
            config = utils.load_yaml(traj)
            traj  = config.get('trajectory')
            if step == 0:
                times = [2]
            elif step == 1:
                times = [0,3]
            elif step == 2:
                times = [0]
            else:
                time = []    
            for time in times:        
                traj[time]['position'][2] = self.height + 0.05
            self.traj_handler.set_config(config)
        else:         
            self.traj_handler.load_config(filename=traj)
        self.traj_handler.set_initialize_time(3.0)
        self.traj_handler.run_trajectory(blocking=True)
        self.traj_handler.shutdown()
        
    def shutdown(self):
        self.is_shutdown=True
        print('Setting all pressures to zero')
        self.send_setpoint([0], 2)
        if self.p_ctrl:
            self.send_setpoint([0]*self.num_channels_total, 2,p_ctrl=True)
        time.sleep(2.2)
        print('Turning off data stream')
        self.set_data_stream(False)
        time.sleep(0.5)
        if self.arm_on:
            self.traj_handler.shutdown()
            
        
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
        