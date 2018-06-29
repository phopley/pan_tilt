# pan_tilt

ROS node to control the position of two pan tilt devices. The node takes demands for Pan/Tilt position of a device and request the servo positions.

## Running the Node

You can test the pan_tilt_node by launching the pan_tilt_test.launch file. This launch file also starts the serial_node as in my system the the actual control of the pan/tilt servos is done on a slave Arduino board.

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

* `/servo/index0/pan_trim`: Trim value for pan servo of the first device. This is a cached parameter. Default = 0.

* `/servo/index0/tilt_trim`: Trim value for tilt servo of the first device. This is a cached parameter. Default = 0.

* `/servo/index1/pan_trim`: Trim value for pan servo of the second device. This is a cached parameter. Default = 0.

* `/servo/index1/tilt_trim`: Trim value for tilt servo of the second device. This is a cached parameter. Default = 0.

## Trimming The Servos

Due to the mechanical fittings of your pan/tilt device it may be off by a number of degrees. You can trim the servos with the following procedure:

Set the position of both servos for the first device to the mid position, say 90 degrees for the pan and 45 degrees for the tilt.

`$ rostopic pub -1 /pan_tilt_node/index0_position servo_msgs/pan_tilt {90,45}`

Estimate the angle that the pan servo is off by and set the /servo/index0/pan_trim value with the following command giving the correction angle. In this example I'm setting it to 5.

`$ rosparam set /servo/index0/pan_trim 5`

Now republish the topic with the 90,45 settings.

`$ rostopic pub -1 /pan_tilt_node/index0_position servo_msgs/pan_tilt {90,45}`

The pan servo will move by the new trim value. Repeat the process until the pan servo is trimmed to where you want it. You can then repeat the process for the tilt servo with the /servo/index0/tilt_trim parameter.

For the second pan/tilt device use the topic `pan_tilt_node/index1_position` and the parameters `/servo/index1/pan_trim` and `/servo/index1/tilt_trim`.

Once you are happy with the trim values you can edit the config.yaml to include these new trim values. Then the next time the nodes are started these trim values will be used.