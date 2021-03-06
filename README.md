# Tag2Arrange

Using April tags to control finger arrangement, pressure controllers and robot arm trajectories

## Installation
1. Install [ROS](https://www.ros.org/)
2. Set up your AprilTag workspace
    1. Make a [new catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
    2. Clone the [AprilTag ROS package](https://github.com/AprilRobotics/apriltag_ros)
    3. Clone the [USB cam package](http://wiki.ros.org/usb_cam)
    4. Install dependencies: `rosdep install --from-paths src --ignore-src -y`
    5. Run `catkin_make_isolated`
3. Set up your pressure control workspace
    1. Make another new catkin workspace
    2. Set up my [pressure controller package](https://ctrl-p.cbteeple.com/latest/ros-driver)
    3. Clone this package into the src folder
    4. Clone my [video_recorder](https://github.com/cbteeple/ros_video_recorder) package into the src folder (_optional_)
    5. Clone my [rosbag_recorder](https://github.com/cbteeple/rosbag-recorder) package into the src folder (_optional_)
    6. Run `catkin_make`
4. Set up the [robot arm control package](https://github.com/cbteeple/simple_ur_move) in same workspace as the pressure controller
5. [Set up your `~/.bashrc` file](https://docs.cbteeple.com/robot/ros#setting-up-ros-on-linux) to source both workspaces

## Setup AprilTags
1. Set up AprilTag config files to specify your tag properties.
2. Make a new verison of `usb_cam-test.launch` to use your camera settings
3. [Calibrate your camera](https://github.com/NVlabs/Deep_Object_Pose/blob/master/doc/camera_tutorial.md)
## Setup Items
1. Set up item directory in the `config/items` directory.
To create this directory from item geometry:
    1. Create a spreadsheet listing the item's tag id, length, width, height and object shape
    2. Import a trained classification model as a pmml file
    3. Run the `create_dir.py` script which will output a yaml file
    
## Usage

1. Start running the camera (publishes data to the `usb_cam/image_raw` topic)
    - `roslaunch usb_cam usb_cam-test.launch`
2. Rectify the image using the calibration matrix ([tutorial](http://wiki.ros.org/image_proc))
    - `ROS_NAMESPACE=my_camera rosrun image_proc image_proc`
3. Begin detecting apriltags
    - If using rectified camera image: `roslaunch apriltag_ros continuous_detection.launch camera_name:=usb_cam`
    - If you just want to test without calibrating: `roslaunch apriltag_ros continuous_detection.launch camera_name:=usb_cam image_topic:=image_raw`
4. Start the pressure controller
    - `roslaunch pressure_controller_ros bringup_HID.launch profile:=anthro8 hw_profile:=hid`
5. Start Dynamixel motor
    - `roslaunch pressure_controller_configs bringup_dynamixel.launch profile:=dynamixel_single hw_profile:=dynamixel_default`
6. Start the robot arm with the teach pendant program
    - `roslaunch ur_user_calibration bringup_armando.launch`
7. Start the controller
    - `roslaunch tag2arrange run_controller.launch pctrl:=True arm:=True`
