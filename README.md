RobotController
===============

ROS enabled Gazebo model plugin and interface for controlling general robot models

To install:
* cd into the folder and run rosmake
* cd into the build folder and run sudo make install
* Add /usr/local/lib/gazebo_plugins to your GAZEBO_PLUGIN_PATH
(eg. export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH: /usr/local/lib/gazebo_plugins)

To use:
* insert the plugin into the urdf of your model by adding the tags:
    <gazebo>
     <plugin filename="libControllerPlugin.so" name="robot_plugin" />
    </gazebo>

Interface:
RobotController supports PID joint position control and feedforward effort control
to send a command send a RobotCommand.msg to /<your_robot_name>/robot_command
to recive the state of the robot subscribe to /<your_robot_name>/robot_state
note that the joint order in the command and robot_state is the order in which they are loaded to the plugin

the modules JointController.py and robot_state.py ease the interface with the plugin


