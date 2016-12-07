#include <ros/ros.h>
#include <keyboard_teleop/Keyboard_teleop.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "keyboard_teleop");

	double gridSize = 0.2;

	syd::Keyboard_teleop *object = new syd::Keyboard_teleop(gridSize);

	ros::Rate rate(100);
	while(ros::ok())
	{
        ros::Duration(1.0).sleep();
		if(object->check_connection())
		{
			ROS_INFO("Pixhawk connected");
			break;
		}
		else
		{
			ROS_INFO("Pixhawk not connected");
		}
	}

	while(ros::ok())
	{	
		object->update();
		object->publish();
		rate.sleep();
	}
}
