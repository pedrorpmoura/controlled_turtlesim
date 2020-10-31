#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>


class TeleopKeyHandler {
	
	public:
		TeleopKeyHandler(void);
		void loop(void);
		
	private:
		ros::NodeHandle nh;
		ros::Subscriber cmd_vel_sub;
		ros::Publisher key_pressed_pub;
		ros::Publisher cmd_vel_pub;
};


TeleopKeyHandler::TeleopKeyHandler(void) {
	
	// setup subscriber
	cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>("/manual_driver/turtle1/cmd_vel", 100,
			[this](const geometry_msgs::Twist::ConstPtr& cmd_vel_msg) {
				ROS_INFO("KEY WAS PRESSED. Publishing msg received and publishing /keypressed signal.");
				
				std_msgs::Bool key_pressed_msg;
				key_pressed_msg.data = true;
				key_pressed_pub.publish(key_pressed_msg);

				cmd_vel_pub.publish(cmd_vel_msg);			
			});
	
	// setup publishers
	key_pressed_pub = nh.advertise<std_msgs::Bool>("/key_pressed", 100);

	cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 100);
}


void TeleopKeyHandler::loop(void) {
	ros::spin();
}


int main(int argc, char *argv[]) {
	
	ros::init(argc, argv, "teleop_key_handler");
	
	TeleopKeyHandler handler;
	handler.loop();

	return 0;
}

















