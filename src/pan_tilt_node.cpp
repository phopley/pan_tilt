/* Pan/Tilt Processing Node. Takes demand for Pan/Tilt position and request the servo positions.
 * It can handle a number of pan tilt device. For example index 0 may be for a head/camera and
 * index 1 could be for a LIDAR device.
 */
#include <pan_tilt/pan_tilt_node.h>
#include <servo_msgs/servo_array.h>

// Constructor 
PanTiltNode::PanTiltNode()
{
	// Set default values
	pan_servo_[0] = 0;
	tilt_servo_[0] = 1;
	pan_servo_[1] = 2;
	tilt_servo_[1] = 3;
    
	pan_max_[0] = 180;
	pan_min_[0] = 0;
	tilt_max_[0] = 180;
	tilt_min_[0] = 0;
	pan_max_[1] = 180;
	pan_min_[1] = 0;
	tilt_max_[1] = 180;
	tilt_min_[1] = 0;

	// Get any parameters from server, will not change after startup 
	n_.param("/servo/index0/pan_servo", pan_servo_[0], pan_servo_[0]);
	n_.param("/servo/index0/tilt_servo", tilt_servo_[0], tilt_servo_[0]);
	n_.param("/servo/index1/pan_servo", pan_servo_[1], pan_servo_[1]);
	n_.param("/servo/index1/tilt_servo", tilt_servo_[1], tilt_servo_[1]);
	n_.param("/servo/index0/pan_max", pan_max_[0], pan_max_[0]);
	n_.param("/servo/index0/pan_min", pan_min_[0], pan_min_[0]);
	n_.param("/servo/index0/tilt_max", tilt_max_[0], tilt_max_[0]);
	n_.param("/servo/index0/tilt_min", tilt_min_[0], tilt_min_[0]);
	n_.param("/servo/index1/pan_max", pan_max_[1], pan_max_[1]);
	n_.param("/servo/index1/pan_min", pan_min_[1], pan_min_[1]);
	n_.param("/servo/index1/tilt_max", tilt_max_[1], tilt_max_[1]);
	n_.param("/servo/index1/tilt_min", tilt_min_[1], tilt_min_[1]);

    first_index0_msg_received_ = false;
    first_index1_msg_received_ = false;

    pan_tilt_sub_[0] = n_.subscribe("pan_tilt_node/index0_position", 10, &PanTiltNode::panTilt0CB, this);
	pan_tilt_sub_[1] = n_.subscribe("pan_tilt_node/index1_position", 10, &PanTiltNode::panTilt1CB, this);

    // Published topic is latched
	servo_array_pub_ = n_.advertise<servo_msgs::servo_array>("servo", 10, true);
}

// This callback is for when the dynamic configuration parameters change
void PanTiltNode::reconfCallback(pan_tilt::PanTiltConfig &config, uint32_t level)
{
    index0_pan_trim_ = config.index0_pan_trim;
    index0_tilt_trim_ = config.index0_tilt_trim;
    index1_pan_trim_ = config.index1_pan_trim;
    index1_tilt_trim_ = config.index1_tilt_trim;

    // We don't want to send a message following a call here unless we have received
    // a position message. Otherwise the trim value will be taken for an actual position.
    if(first_index0_msg_received_ == true)
    {
        // Send new messages with new trim values
        movePanTilt(index0_pan_tilt_, index0_pan_trim_, index0_tilt_trim_, 0);        
    }

    if(first_index1_msg_received_ == true)
    {
        movePanTilt(index1_pan_tilt_, index1_pan_trim_, index1_tilt_trim_, 1);
    }
}
//---------------------------------------------------------------------------

// Callback to move the pan tilt device indexed 0
void PanTiltNode::panTilt0CB(const servo_msgs::pan_tilt& pan_tilt)
{
    // Store latest value in case we change thte trim
    index0_pan_tilt_ = pan_tilt;
    first_index0_msg_received_ = true;
    
    movePanTilt(pan_tilt, index0_pan_trim_, index0_tilt_trim_, 0);
}

// Callback to move the pan tilt device indexed 1
void PanTiltNode::panTilt1CB(const servo_msgs::pan_tilt& pan_tilt)
{    
    // Store latest value in case we change thte trim
    index1_pan_tilt_ = pan_tilt;
    first_index1_msg_received_ = true;
    
    movePanTilt(pan_tilt, index1_pan_trim_, index0_tilt_trim_, 1);
}

void PanTiltNode::movePanTilt(const servo_msgs::pan_tilt& pan_tilt, int pan_trim, int tilt_trim, int index)
{
	int pan;
	int tilt;
	servo_msgs::servo_array servo;

	pan = pan_trim + pan_tilt.pan;
	tilt = tilt_trim + pan_tilt.tilt;

	pan = checkMaxMin(pan, pan_max_[index], pan_min_[index]);
	tilt = checkMaxMin(tilt, tilt_max_[index], tilt_min_[index]);

	// Send message for pan position
	servo.index = (unsigned int)pan_servo_[index];
	servo.angle = (unsigned int)pan;
	servo_array_pub_.publish(servo);

	// Send message for tilt position
	servo.index = (unsigned int)tilt_servo_[index];
	servo.angle = (unsigned int)tilt;
	servo_array_pub_.publish(servo);    
}

int PanTiltNode::checkMaxMin(int current_value, int max, int min)
{
	int value = current_value;

	if (value > max)
	{
		value = max;
	}

	if (value < min)
	{
		value = min;
	}

	return (value);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pan_tilt_node");	
	
	PanTiltNode *pan_tiltnode = new PanTiltNode();
	
	dynamic_reconfigure::Server<pan_tilt::PanTiltConfig> server;
    dynamic_reconfigure::Server<pan_tilt::PanTiltConfig>::CallbackType f;
  	
  	f = boost::bind(&PanTiltNode::reconfCallback, pan_tiltnode, _1, _2);
    server.setCallback(f);
    	
    std::string node_name = ros::this_node::getName();
	ROS_INFO("%s started", node_name.c_str());
	ros::spin();
	return 0;
}
