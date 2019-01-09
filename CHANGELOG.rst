^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pan_tilt
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2019-01-09)
------------------
* Changes to match ROS standards for coordinate frames and to use to /sensor_msgs/JointState type (input radians)
* Layout of config file changed
* Don't publish a position message from a dynamic reconfig call unless a position message has been received
* Changes to config file for tilt min and max
* pan trim for index0 servo in pan_tilt.cfg set to 0

0.1.5 (2018-08-15)
------------------
* tilt_max for index0 servo in config file set to 90
* pan trim for index0 servo in pan_tilt.cfg set to -6

0.1.4 (2018-07-31)
------------------
* Invalid email address in package.xml file causes error when node launched

0.1.3 (2018-07-27)
------------------
* servo_array message now latched

0.1.2 (2018-07-14)
------------------
* Trim now controlled by dynamic reconfiguration server

0.1.1 (2018-06-29)
------------------
* Updates to standardize documentation

0.1.0 (2018-06-14)
------------------
* First formal release of the package
