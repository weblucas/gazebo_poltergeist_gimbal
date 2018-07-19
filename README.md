Only pitch is tested

*demo:
roslaunch gazebo_poltergeist_gimbal top_gimbal.launch
rostopic pub /firefly/command/gimbal_actuators sensor_msgs/Joy " {'header': {'stamp': 'now', 'frame_id': '', 'seq': 0}, 'axes': [0.0, 45.0, 0.0]}"
