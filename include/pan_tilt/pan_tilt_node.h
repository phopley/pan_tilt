#ifndef PAN_TILT_NODE_H_
#define PAN_TILT_NODE_H_
/* Pan/Tilt Processing Node. Takes demand for Pan/Tilt position and request the servo positions.
 * It can handle a number of pan tilt device. For example index 0 may be for a head/camera and
 * index 1 could be for a LIDAR device.
 */
#include <ros/ros.h>
#include <servo_msgs/pan_tilt.h>
#include <dynamic_reconfigure/server.h>
#include <pan_tilt/PanTiltConfig.h>

#define NUMBER_OF_PAN_TILT_DEVICES 2

class PanTiltNode
{
public:
    PanTiltNode();
    
    // This callback is for when the dynamic configuration parameters change
    void reconfCallback(pan_tilt::PanTiltConfig &config, uint32_t level);

private:
    void panTilt0CB(const servo_msgs::pan_tilt& pan_tilt);    
    void panTilt1CB(const servo_msgs::pan_tilt& pan_tilt);    
    void movePanTilt(const servo_msgs::pan_tilt& pan_tilt, int pan_trim, int tilt_trim, int index);
	int checkMaxMin(int current_value, int max, int min);

    ros::NodeHandle n_;
    ros::Subscriber pan_tilt_sub_[NUMBER_OF_PAN_TILT_DEVICES];
    ros::Publisher servo_array_pub_;
    
    // Used for dynamic reconfiguration
    int index0_pan_trim_;
    int index0_tilt_trim_;
    int index1_pan_trim_;
    int index1_tilt_trim_;
    bool first_index0_msg_received_;
    bool first_index1_msg_received_;
    
    // Used to store last position (not including trim value)
    servo_msgs::pan_tilt index0_pan_tilt_;
    servo_msgs::pan_tilt index1_pan_tilt_;

    // Configuration parameters
    int pan_servo_[NUMBER_OF_PAN_TILT_DEVICES];     // Maps a servo to a pan device
	int tilt_servo_[NUMBER_OF_PAN_TILT_DEVICES];    // Maps a servo to a tilt device
    int pan_max_[NUMBER_OF_PAN_TILT_DEVICES];       // Maximum range for the pan servo    
    int pan_min_[NUMBER_OF_PAN_TILT_DEVICES];       // Minimum range for the pan servo
    int tilt_max_[NUMBER_OF_PAN_TILT_DEVICES];      // Maximum range for the tilt servo 
    int tilt_min_[NUMBER_OF_PAN_TILT_DEVICES];      // Minimum range for the tilt servo
};

#endif // PAN_TILT_NODE_H_
