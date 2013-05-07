#
# This message has been carefully constructed to be less
# than 1500 in size when serialized, to accommodate transfer
# UDP.
#
# testing everything a robot needs
Header header

# repeating data from osrf_msgs/JointCommands as joint_states
float32[] position
float32[] velocity
float32[] effort
float32[] kp_position
float32[] ki_position
float32[] kd_position
float32[] kp_velocity
float32[] i_effort_min
float32[] i_effort_max

#sensor_msgs/Imu imu 
geometry_msgs/Quaternion orientation
geometry_msgs/Vector3 angular_velocity
geometry_msgs/Vector3 linear_acceleration

#robotcon_msgs/ForceTorqueSensors force_torque_sensors
geometry_msgs/Wrench[] ankles
geometry_msgs/Wrench[] wrists
