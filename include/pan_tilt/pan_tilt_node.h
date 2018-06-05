#ifndef PAN_TILT_NODE_H_
#define PAN_TILT_NODE_H_
/* Pan/Tilt Processing Node. Takes demand for Pan/Tilt position and request the servo positions.
 * It can handle a number of pan tilt device. For example index 0 may be for a head/camera and
 * index 1 could be for a LIDAR device.
 */
#include <ros/ros.h>
#include <servo_msgs/pan_tilt.h>

#define NUMBER_OF_PAN_TILT_DEVICES 2

class PanTiltNode
{
public:
    PanTiltNode();

private:
    void panTilt0CB(const servo_msgs::pan_tilt& pan_tilt);    
    void panTilt1CB(const servo_msgs::pan_tilt& pan_tilt);    
    void movePanTilt(const servo_msgs::pan_tilt& pan_tilt, int pan_trim, int tilt_trim, int index);
	int checkMaxMin(int current_value, int max, int min);

    ros::NodeHandle n_;
    ros::Subscriber pan_tilt_sub_[NUMBER_OF_PAN_TILT_DEVICES];
    ros::Publisher servo_array_pub_;

    // Configuration parameters
    int pan_servo_[NUMBER_OF_PAN_TILT_DEVICES];     // Maps a servo to a pan device
	int tilt_servo_[NUMBER_OF_PAN_TILT_DEVICES];    // Maps a servo to a tilt device
    int pan_max_[NUMBER_OF_PAN_TILT_DEVICES];       // Maximum range for the pan servo    
    int pan_min_[NUMBER_OF_PAN_TILT_DEVICES];       // Minimum range for the pan servo
    int tilt_max_[NUMBER_OF_PAN_TILT_DEVICES];      // Maximum range for the tilt servo 
    int tilt_min_[NUMBER_OF_PAN_TILT_DEVICES];      // Minimum range for the tilt servo
};

#endif // PAN_TILT_NODE_H_
