#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>

#include <math.h>
#include <tuple>

class AutomaticDriver {
	
	public:
		AutomaticDriver(void);
		void loop(void);
	
	private:
		bool auto_driving = false;
		std::tuple<float,float> current_pose;

		ros::NodeHandle nh;
		ros::Subscriber drive_sub;
		ros::Subscriber pose_sub;
		ros::Publisher cmd_vel_pub;
		
		geometry_msgs::Twist calculateNextMove(void);
		geometry_msgs::Twist avoidCollision(void);
};


AutomaticDriver::AutomaticDriver(void) {
	
	// setup subscribers
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

	pose_sub = nh.subscribe<turtlesim::Pose>("/turtle1/pose", 1,
			[this](const turtlesim::Pose::ConstPtr &msg) {
				current_pose = std::make_tuple(msg->x, msg->y);
			});

	// setup publisher
	cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 100);	
}


geometry_msgs::Twist AutomaticDriver::calculateNextMove(void) {
	
	geometry_msgs::Twist msg;

	msg.linear.x  = 1;
	
	float pi_2 = M_PI / 2;
	float angular_z = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / M_PI));
	msg.angular.z = angular_z - pi_2;

	return msg;
}


geometry_msgs::Twist AutomaticDriver::avoidCollision(void) {

	geometry_msgs::Twist msg;
	
	msg.linear.x = -1;

	return msg;
}


void AutomaticDriver::loop(void) {
	
	ros::Rate rate(10);
	int counter = 0;
	while (ros::ok()) {
	
		if (auto_driving && (counter % 10) == 0) {
			geometry_msgs::Twist msg;
			
			int x = std::get<0>(current_pose);
			int y = std::get<0>(current_pose);
			if (x <= 1.0 || x >= 10.0 || y <= 1.0 || y >= 10.0) {
				msg = avoidCollision();
			}
			else {
				msg = calculateNextMove();
			}

			cmd_vel_pub.publish(msg);
		}
		
		rate.sleep();
		ros::spinOnce();
		counter++;
	}
}

int main(int argc, char *argv[]) {

	ros::init(argc, argv, "automatic_driver");
	
	AutomaticDriver driver;
	driver.loop();

	return 0;
}















