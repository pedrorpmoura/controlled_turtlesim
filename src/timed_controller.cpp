#include <ros/ros.h>
#include <std_msgs/Bool.h>


class TimedController {
	
	public:
		TimedController(void);
		void loop(void);
			
	private:
		int timer_duration;	
		bool timer_running;
		
		ros::Timer timer;
		ros::NodeHandle nh;
		ros::Subscriber signal_sub;
		ros::Publisher start_stop_pub; // true to start, false to stop
};


TimedController::TimedController(void) {
	
	// get timer duration from launch file
	nh.getParam("/timed_controller/time", timer_duration);
	
	// setup timer
	timer = nh.createTimer(ros::Duration(timer_duration), 
			[this](const ros::TimerEvent&) {
				ROS_INFO("TIMER FINISHED. Publishing /start_stop = true.");
				std_msgs::Bool msg;
				msg.data = true;
				start_stop_pub.publish(msg);
				
				timer.stop();
			});
	
	// setup subscriber
	signal_sub = nh.subscribe<std_msgs::Bool>("/signal", 100,
			[this](const std_msgs::Bool::ConstPtr&) {
				ROS_INFO("RECEIVED SIGNAL. Publishing /start_stop = false.");
				std_msgs::Bool msg;
				msg.data = false;
				start_stop_pub.publish(msg);

				ROS_INFO("RESTARTING TIMER...");
				timer.stop();
				timer.start();
			});	

	// setup publisher
	start_stop_pub = nh.advertise<std_msgs::Bool>("/start_stop", 100);
}


void TimedController::loop(void) {
		
	ros::spin();	
}	


int main(int argc, char *argv[]) {
	
	ros::init(argc, argv, "timed_controller");

	TimedController controller;
	controller.loop();
	
	return 0;
}


