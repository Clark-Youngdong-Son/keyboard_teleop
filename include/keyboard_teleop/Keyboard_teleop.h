#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <keyboard/Key.h>
#include <std_msgs/Bool.h>

#define KEY_W 119	//forward
#define KEY_A 97	//left
#define KEY_S 115	//backward
#define KEY_D 100	//right
#define KEY_O 111	//altitude down
#define KEY_P 112	//altitude up
#define KEY_SPACE 32	//all kill
#define KEY_C 99	//disarm
#define KEY_V 118	//arm
#define KEY_B 98	//Robot arm switch off
#define KEY_N 110	//Robot arm switch on
#define KEY_I 105	//initialize

#define ARMED -22
#define DISARMED -33

namespace syd {

	class Keyboard_teleop {

		public:
		Keyboard_teleop(double);
		bool check_connection();
		void update();
		void publish();
		
		private:
		ros::NodeHandle nh;	

		mavros_msgs::State current_state;
		mavros_msgs::CommandBool arm_cmd;
		geometry_msgs::PoseStamped current_pose, set_pose;
		
		ros::Subscriber state_sub, local_position_sub, keyboard_sub;
		ros::Publisher setpoint_pub, robotArm_pub;
		ros::ServiceClient arming_client, set_mode_client;

		void state_callback(const mavros_msgs::State::ConstPtr&);
		void local_position_callback(const geometry_msgs::PoseStamped::ConstPtr&);
		void keyboard_callback(const keyboard::Key::ConstPtr&);
		void initialize();
		bool initializeFlag;
		bool setArming(int);
        bool robotArmSwitch;
	
		int key_in;
		double gridSize;
	};

}
