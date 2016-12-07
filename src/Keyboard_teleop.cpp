#include <keyboard_teleop/Keyboard_teleop.h>
#include <string>

using namespace syd;

Keyboard_teleop::Keyboard_teleop(double _gridSize) : gridSize(_gridSize), initializeFlag(false), isOffBoard(false)
{
	state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, &Keyboard_teleop::state_callback, this);
	local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &Keyboard_teleop::local_position_callback, this);
	keyboard_sub = nh.subscribe<keyboard::Key>("keyboard/keydown", 10, &Keyboard_teleop::keyboard_callback, this);
	
	setpoint_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local",10);
	arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
	set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
}


bool Keyboard_teleop::setArming(int _mode)
{
	if(_mode == SET_ARM)
	{
		if(current_state.armed)
		{
			ROS_INFO("Already armed");
			return true;
		}
		else
		{
			arm_cmd.request.value = true;
			if(arming_client.call(arm_cmd) && arm_cmd.response.success)
			{
				ros::Duration(1).sleep();
				ros::spinOnce();
				if(current_state.armed)
				{
					ROS_INFO("Armed");
					initialize();
					return true;
				}
				else
				{
					ROS_INFO("Arming failed");
					return false;
				}
			}
			else
			{
				ROS_INFO("Arming failed");
				return false;
			}
		}
	}

	else if(_mode == SET_DISARM)
	{
		if(!current_state.armed)
		{
			ROS_INFO("Already diarmed");
			return true;
		}
		else
		{
			arm_cmd.request.value = false;
			if(arming_client.call(arm_cmd) && arm_cmd.response.success)
			{
				ros::Duration(1).sleep();
				if(!current_state.armed)
				{
					ROS_INFO("Disarmed");
					return true;
				}
				else
				{
					ROS_INFO("Disarming failed");
					return false;
				}
			}
			else
			{
				ROS_INFO("Disarming failed");
				return false;
			}
		}
	}
}

bool Keyboard_teleop::setMode(int _mode)
{
	if(_mode == SET_STABILIZED)
	{
		if(!current_state.armed)
		{
			ROS_INFO("Not armed");
			return false;
		}
		else if(current_state.mode == "STABILIZED")
		{
			ROS_INFO("Already stabilized mode");
			initialize();
			return true;
		}
		else
		{
			setMode_cmd.request.custom_mode = "STABILIZED";
			if(set_mode_client.call(setMode_cmd) && setMode_cmd.response.success)
			{
				ros::Duration(2).sleep();
				if(current_state.mode == "STABILIZED")
				{
					ROS_INFO("Changed to stabilized mode");
					isOffBoard = false;
					return true;
				}
				else
				{
					ROS_INFO("Stabilized mode change failed");
					return false;
				}
			}
			else
			{
				ROS_INFO("Stabilized mode change failed");
				return false;
			}
		}
	}
	else if(_mode == SET_OFFBOARD)
	{
		if(!current_state.armed)
		{
			ROS_INFO("Not armed");
			return false;
		}
		else if(current_state.mode == "OFFBOARD")
		{
			ROS_INFO("Already offboard mode");
			initialize();
			return true;
		}
		else
		{
			setMode_cmd.request.custom_mode = "OFFBOARD";
			if(set_mode_client.call(setMode_cmd) && setMode_cmd.response.success)
			{
//				ros::Duration(2).sleep();
				if(current_state.mode == "OFFBOARD")
				{
					ROS_INFO("Changed to offboard mode");
					isOffBoard = true;
					initialize();
					return true;
				}
				else
				{
					ROS_INFO("Offboard mode change failed");
					return false;
				}
			}
			else
			{
				ROS_INFO("Offboard mode change failed");
				return false;
			}
		}

	}

}

void Keyboard_teleop::update()
{
	ros::spinOnce();
	switch (key_in)
	{
		case KEY_W :
		if(initializeFlag)
		{
			set_pose.pose.position.x -= gridSize;
			ROS_INFO("Move forward");
		ROS_INFO_STREAM("setpoint : [x = " << set_pose.pose.position.x << " y = " << set_pose.pose.position.y << " z = " << set_pose.pose.position.z << "]");
		}
		break;

		case KEY_A :
		if(initializeFlag)
		{
			set_pose.pose.position.y -= gridSize;
			ROS_INFO("Move left");
		ROS_INFO_STREAM("setpoint : [x = " << set_pose.pose.position.x << " y = " << set_pose.pose.position.y << " z = " << set_pose.pose.position.z << "]");
		}
		break;

		case KEY_S :
		if(initializeFlag)
		{
			set_pose.pose.position.x += gridSize;
			ROS_INFO("Move backward");
		ROS_INFO_STREAM("setpoint : [x = " << set_pose.pose.position.x << " y = " << set_pose.pose.position.y << " z = " << set_pose.pose.position.z << "]");
		}
		break;

		case KEY_D :
		if(initializeFlag)
		{
			set_pose.pose.position.y += gridSize;
			ROS_INFO("Move right");
		ROS_INFO_STREAM("setpoint : [x = " << set_pose.pose.position.x << " y = " << set_pose.pose.position.y << " z = " << set_pose.pose.position.z << "]");
		}
		break;

		case KEY_O :
		if(initializeFlag)
		{
			set_pose.pose.position.z -= gridSize*0.2;
			ROS_INFO("Move upward");
		ROS_INFO_STREAM("setpoint : [x = " << set_pose.pose.position.x << " y = " << set_pose.pose.position.y << " z = " << set_pose.pose.position.z << "]");
		}
		break;

		case KEY_P :
		if(initializeFlag)
		{
			set_pose.pose.position.z += gridSize*0.2;
			ROS_INFO("Move downward");
		ROS_INFO_STREAM("setpoint : [x = " << set_pose.pose.position.x << " y = " << set_pose.pose.position.y << " z = " << set_pose.pose.position.z << "]");
		}
		break;

		case KEY_SPACE :
			setArming(SET_DISARM);
		break;

		case KEY_C :
		setArming(SET_DISARM);	
		break;

		case KEY_V :
		setArming(SET_ARM);
		initialize();
		break;

		case KEY_B :
		//setMode(SET_OFFBOARD);
		//initialize();
		break;

		case KEY_N :
		//setMode(SET_STABILIZED);
		break;

		default :
		;
//		ROS_INFO("Not assigned key");
	}
key_in = -100;
}

bool Keyboard_teleop::check_connection()
{
	ros::spinOnce();
	if(current_state.connected) return true;
	return false;
}

void Keyboard_teleop::state_callback(const mavros_msgs::State::ConstPtr& msg)
{
	current_state = *msg;
	if(current_state.mode == "OFFBOARD") isOffBoard = true;
	else isOffBoard = false;
}

void Keyboard_teleop::local_position_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	current_pose = *msg;
}

void Keyboard_teleop::keyboard_callback(const keyboard::Key::ConstPtr& msg)
{
	key_in = msg->code;
}

void Keyboard_teleop::initialize()
{
	set_pose = current_pose;
	set_pose.pose.position.z -= 0.2;
	ROS_INFO_STREAM("setpoint : [x = " << set_pose.pose.position.x << " y = " << set_pose.pose.position.y << " z = " << set_pose.pose.position.z << "]");
	initializeFlag = true;
	ROS_INFO("Initialized");
}

void Keyboard_teleop::publish()
{
	if(initializeFlag)
	{
		//set_pose.header.stamp = ros::Time::now();
		setpoint_pub.publish(set_pose);
	}
}
