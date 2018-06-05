# pan_tilt
ROS node to control an array of pan tilt devices.
## Executable
pan_tilt_node
## Subscribe topics
- pan_tilt_node/index0_position of type pan_tilt.msg
- pan_tilt_node/index1_position of type pan_tilt.msg
## publishes topics 
- servo of type servo_array.msg

### Description
The node takes demands for Pan/Tilt position of a device and request the servo positions.
It can handle a number of pan tilt devices. For example index 0 may be for a head/camera and index 1 could be for a LIDAR device.

