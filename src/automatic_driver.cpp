#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>


class AutomaticDriver {
	
	public:
		AutomaticDriver(void);
		geometry_msgs::Twist generateCmdVelMessage(void);
		void loop(void);
	
	private:
		bool auto_driving = false;

		ros::NodeHandle nh;
		ros::Subscriber drive_sub;
		ros::Publisher cmd_vel_pub;
};


AutomaticDriver::AutomaticDriver(void) {
	
	// setup subscriber
	drive_sub = nh.subscribe<std_msgs::Bool>("/drive", 100,
			[this](const std_msgs::Bool::ConstPtr &msg) {
				ROS_INFO("RECEIVED drive message");
				auto_driving = msg->data;
				if (auto_driving) {
					ROS_INFO("Automatic driving activated");
				} else {
					ROS_INFO("Automatic driving deactivated");
				}
			});

	// setup publisher
	cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 100);	
}


geometry_msgs::Twist AutomaticDriver::generateCmdVelMessage(void) {
	
	geometry_msgs::Twist msg;
	msg.linear.x = 2;
	msg.angular.z = 2;

	return msg;
}


void AutomaticDriver::loop(void) {
	
	ros::Rate rate(10);
	while (ros::ok()) {
	
		if (auto_driving) {
			geometry_msgs::Twist msg = generateCmdVelMessage();
			cmd_vel_pub.publish(msg);
		}
		
		rate.sleep();
		ros::spinOnce();
	}
}

int main(int argc, char *argv[]) {

	ros::init(argc, argv, "automatic_driver");
	
	AutomaticDriver driver;
	driver.loop();

	return 0;
}















