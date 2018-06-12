# pan_tilt
ROS node to control an array of pan tilt devices.
## Executable
pan_tilt_node
## Subscribe topics
- pan_tilt_node/index0_position of type pan_tilt.msg
- pan_tilt_node/index1_position of type pan_tilt.msg
## publishes topics 
- servo of type servo_array.msg
## Fixed server parameters
- /servo/index0/pan_servo, default = 0, servo index for the pan servo of device 0
- /servo/index0/tilt_servo, default = 1, servo index for the tilt servo of device 0
- /servo/index1/pan_servo, default = 2, servo index for the pan servo of device 1
- /servo/index1/tilt_servo", default = 3, servo index for the tilt servo of device 1
- /servo/index0/pan_max", default = 180, maximum range for the pan servo of device 0
- /servo/index0/pan_min", default = 0, minimum range for the pan servo of device 0
- /servo/index0/tilt_max", default = 180, maximum range for the tilt servo of device 0
- /servo/index0/tilt_min", default = 0, minimum range for the tilt servo of device 0
- /servo/index1/pan_max", default = 180, maximum range for the pan servo of device 1
- /servo/index1/pan_min", default = 0, minimum range for the pan servo of device 1
- /servo/index1/tilt_max", default = 180, maximum range for the tilt servo of device 1
- /servo/index1/tilt_min", default = 0, minimum range for the tilt servo of device 1
## Cached server parameters
- /servo/index0/pan_trim, trim value for pan servo device 0
- /servo/index0/tilt_trim, trim value for tilt servo device 0
- /servo/index1/pan_trim, trim value for pan servo device 1
- /servo/index1/tilt_trim, trim value for tilt servo device 1
### Description
The node takes demands for Pan/Tilt position of a device and request the servo positions.
It can handle a number of pan tilt devices. For example index 0 may be for a head/camera and index 1 could be for a LIDAR device.

