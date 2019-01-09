/* Pan/Tilt Processing Node. Takes demand for Pan/Tilt position and request the servo positions.
 * It can handle a number of pan tilt device. For example index 0 may be for a head/camera and
 * index 1 could be for a second device.
 *
 * The position (rotation) values are signed radians and follow 
 * the right-hand rule (https://en.wikipedia/wiki/Right-hand_rule).
 */
#include <pan_tilt/pan_tilt_node.h>
#include <servo_msgs/servo_array.h>
#include <math.h>

// Constructor 
PanTiltNode::PanTiltNode()
{
    double max_radians;
    double min_radians;
    int temp;

	/* Get any parameters from server which will not change after startup. 
     * Defaults used if parameter is not in the parameter server
     */

    // Which servo is used for what
	n_.param("/servo/index0/pan/servo",  pan_servo_[0],  0);
	n_.param("/servo/index0/tilt/servo", tilt_servo_[0], 1);
	n_.param("/servo/index1/pan/servo",  pan_servo_[1],  2);
	n_.param("/servo/index1/tilt/servo", tilt_servo_[1], 3);

    // Check for any servos mounted the opposite rotation of the right hand rule
    n_.param("/servo/index0/pan/flip_rotation", pan_flip_rotation_[0], false);
    n_.param("/servo/index0/tilt/flip_rotation", tilt_flip_rotation_[0], false);
    n_.param("/servo/index1/pan/flip_rotation", pan_flip_rotation_[1], false);
    n_.param("/servo/index1/tilt/flip_rotation", tilt_flip_rotation_[1], false);

    /* Maximum and Minimum ranges. Values stored on parameter server in
     * radians and RH rule as per ROS standard. These need converting
     * to degrees and may need flipping.
     */
	n_.param("/servo/index0/pan/max", max_radians, M_PI/2.0);
	n_.param("/servo/index0/pan/min", min_radians, -(M_PI/2.0));
    pan_max_[0] = (int)signedRadianToServoDegrees(max_radians, pan_flip_rotation_[0]);
    pan_min_[0] = (int)signedRadianToServoDegrees(min_radians, pan_flip_rotation_[0]);
    if(true == pan_flip_rotation_[0])
    {
        temp = pan_max_[0];
        pan_max_[0] = pan_min_[0];
        pan_min_[0] = temp;
    }

	n_.param("/servo/index0/tilt/max", max_radians, M_PI/2.0);
    n_.param("/servo/index0/tilt/min", min_radians, -(M_PI/2.0));
    tilt_max_[0] = (int)signedRadianToServoDegrees(max_radians, tilt_flip_rotation_[0]);
    tilt_min_[0] = (int)signedRadianToServoDegrees(min_radians, tilt_flip_rotation_[0]);
    if(true == tilt_flip_rotation_[0])
    {
        temp = tilt_max_[0];
        tilt_max_[0] = tilt_min_[0];
        tilt_min_[0] = temp;
    }

	n_.param("/servo/index1/pan/max", max_radians, M_PI/2.0);
    n_.param("/servo/index1/pan/min", min_radians, -(M_PI/2.0));
    pan_max_[1] = (int)signedRadianToServoDegrees(max_radians, pan_flip_rotation_[1]);	
    pan_min_[1] = (int)signedRadianToServoDegrees(min_radians, pan_flip_rotation_[1]);
    if(true == pan_flip_rotation_[1])
    {
        temp = pan_max_[1];
        pan_max_[1] = pan_min_[1];
        pan_min_[1] = temp;
    }

	n_.param("/servo/index1/tilt/max", max_radians, M_PI/2.0);
    n_.param("/servo/index1/tilt/min", min_radians, -(M_PI/2.0));
    tilt_max_[1] = (int)signedRadianToServoDegrees(max_radians, tilt_flip_rotation_[1]);
    tilt_min_[1] = (int)signedRadianToServoDegrees(min_radians, tilt_flip_rotation_[1]);
    if(true == tilt_flip_rotation_[1])
    {
        temp = tilt_max_[1];
        tilt_max_[1] = tilt_min_[1];
        tilt_min_[1] = temp;
    }

    // Joint names
    n_.param<std::string>("/servo/index0/pan/joint_name", pan_joint_names_[0], "reserved_pan0");
    n_.param<std::string>("/servo/index0/tilt/joint_name", tilt_joint_names_[0], "reserved_tilt0");
    n_.param<std::string>("/servo/index1/pan/joint_name", pan_joint_names_[1], "reserved_pan1");
    n_.param<std::string>("/servo/index1/tilt/joint_name", tilt_joint_names_[1], "reserved_tilt1");

    first_index0_msg_received_ = false;
    first_index1_msg_received_ = false;

    // Published topic is latched
	servo_array_pub_ = n_.advertise<servo_msgs::servo_array>("/servo", 10, true);

    // Subscribe to topic
    joint_state_sub_ = n_.subscribe("/pan_tilt_node/joints", 10, &PanTiltNode::panTiltCB, this);
}
//---------------------------------------------------------------------------

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
        movePanTilt(index0_pan_, index0_tilt_, index0_pan_trim_, index0_tilt_trim_, 0);        
    }

    if(first_index1_msg_received_ == true)
    {
        movePanTilt(index1_pan_, index1_tilt_, index1_pan_trim_, index1_tilt_trim_, 1);
    }
}
//---------------------------------------------------------------------------

// Callback to move the joints
void PanTiltNode::panTiltCB(const sensor_msgs::JointState& joint)
{
    bool index0 = false;
    bool index1 = false;

    /* Search the list of joint names in the message. Although we expect pan/tilt
     * values for one device, a JointState message may contain data for one joint
     * or all four joints. The position (rotation) values are signed radians and
     * follow the right-hand rule. Values to be converted from signed radians to
     * degrees and for the servo orientation. Pan/tilt values are also stored in
     * case we change the trim.
     */
    for (unsigned int i = 0; i < joint.name.size(); i++)
    {         
        // Is it one of the pan or tilt joints
        if(pan_joint_names_[0] == joint.name[i])
        {
            // Index 0 pan
            index0_pan_ = (int)signedRadianToServoDegrees(joint.position[i], pan_flip_rotation_[0]);
            index0 = true;
        }
        else if(pan_joint_names_[1] == joint.name[i])
        {
            // Index 1 pan
            index1_pan_ = (int)signedRadianToServoDegrees(joint.position[i], pan_flip_rotation_[1]);
            index1 = true;            
        }
        else if(tilt_joint_names_[0] == joint.name[i])
        {
            // Index 0 tilt
            index0_tilt_ = (int)signedRadianToServoDegrees(joint.position[i], tilt_flip_rotation_[0]);
            index0 = true;                        
        }
        else if (tilt_joint_names_[1] == joint.name[i])
        {
            // Index 1 tilt
            index1_tilt_ = (int)signedRadianToServoDegrees(joint.position[i], tilt_flip_rotation_[1]);
            index1 = true;
        }
    }

    if(index0 == true)
    {
        first_index0_msg_received_ = true;
        movePanTilt(index0_pan_, index0_tilt_, index0_pan_trim_, index0_tilt_trim_, 0);        
    }

    if(index1 == true)
    {
        first_index1_msg_received_ = true; 
        movePanTilt(index1_pan_, index1_tilt_, index1_pan_trim_, index0_tilt_trim_, 1);
    }       
}
//---------------------------------------------------------------------------

// Converts a signed radian value to servo degrees. 0 radians is 90 degrees.
double PanTiltNode::signedRadianToServoDegrees(double rad, bool flip_rotation)
{
    double retVal;
    
    if(true == flip_rotation)
    {
        retVal = ((-rad/(2.0*M_PI))*360.0)+90.0;
    }        
    else
    {
        retVal = ((rad/(2.0*M_PI))*360.0)+90.0;
    }

    return retVal;
}
//---------------------------------------------------------------------------

void PanTiltNode::movePanTilt(int pan_value, int tilt_value, int pan_trim, int tilt_trim, int index)
{
	int pan;
	int tilt;
	servo_msgs::servo_array servo;

	pan = pan_trim + pan_value;
	tilt = tilt_trim + tilt_value;

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
//---------------------------------------------------------------------------

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
//---------------------------------------------------------------------------

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
//---------------------------------------------------------------------------

