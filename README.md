# pan_tilt

ROS node to control the position of two pan tilt devices. The node takes demands for Pan/Tilt position of a device and request the servo positions.

## Running the Node

Once you have the node built you can test it by launching the pan_tilt_test.launch file. This launch file also starts the serial_node as in my system the the actual control of the pan/tilt servos is done on a slave Arduino board.

## Node Information
Topics:

* `pan_tilt_node/index0_position`:  
  Subscribes `servo_msgs/pan_tilt` with the pan and tilt demands for the first pan/tilt device.

* `pan_tilt_node/index1_position`:  
  Subscribes `servo_msgs/pan_tilt` with the pan and tilt demands for the second pan/tilt device.

* `servo`:  
  Publishes `servo_msgs/servo_array` with the index of the servo (0-3) and the demand angle for the servo.
 

Parameters:

* `/servo/index0/pan_servo`: Servo index for the pan servo of the first device. Default value = 0.

* `/servo/index0/tilt_servo`: Servo index for the tilt servo of the first device. Default = 1.

* `/servo/index1/pan_servo`: Servo index for the pan servo of the second device. Default value = 2.

* `/servo/index1/tilt_servo`: Servo index for the tilt servo of the second device. Default = 3.

* `/servo/index0/pan_max`: Maximum range for the pan servo of the first device. Default = 180.

* `/servo/index0/pan_min`: Minimum range for the pan servo of the first device. Default = 0.

* `/servo/index0/tilt_max`: Maximum range for the tilt servo of the first device. Default = 180.

* `/servo/index0/tilt_min`: Minimum range for the tilt servo of first device. Default = 0.

* `/servo/index1/pan_max`: Maximum range for the pan servo of second device. Default = 180.

* `/servo/index1/pan_min`: Minimum range for the pan servo of second device. Default = 0.

* `/servo/index1/tilt_max`: Maximum range for the tilt servo of second device. Default = 180.

* `/servo/index1/tilt_min`: Minimum range for the tilt servo of second device. Default = 0.

## Trimming The Servos with dynamic reconfiguration

Due to the mechanical fittings of your pan/tilt device it may be off by a number of degrees. You can dynamically adjust the trim using rqt_reconfigure. Start this application with `rosrun rqt_reconfigure rqt_reconfigure`. It will bring up a user interface like the one shown below. Trim paramaters can be dynamically adjusted via the interface. 

![rqt_reconfigure](pan_tilt_trim.png)

Once you are happy with the trim values you can edit the default values in the cfg/pan_tilt.cfg file.
