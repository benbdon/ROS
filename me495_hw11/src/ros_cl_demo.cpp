#include <ros/ros.h>
#include <std_msgs/String.h>
#include <me495_hw1/ME495Pub.h>
#include <me495_hw1/ME495Srv.h>
#include <cmath>
#include <string>
#include <iostream>


class ME495Demo
{

private:
	ros::NodeHandle n_;
	ros::Publisher pubdemo;
	ros::Timer pubtimer;
	ros::Subscriber subdemo;
	ros::ServiceServer srvdemo;
	ros::Time base_time;

public:
	ME495Demo() {
		pubdemo = n_.advertise<me495_hw1::ME495Pub>("demo_publish_topic", 1);
		pubtimer = n_.createTimer(ros::Duration(0.02), &ME495Demo::timercb, this);
		subdemo = n_.subscribe("demo_subscriber_topic", 10, &ME495Demo::subscribercb, this);
		srvdemo = n_.advertiseService("me495_math_server", &ME495Demo::srvcb, this);
		base_time = ros::Time::now();
		return;
	}

	void timercb(const ros::TimerEvent& e) {
		ros::Time time_now = ros::Time::now();
		float time = (time_now - base_time).toSec();
		float config = 10*sin(2*M_PI*time);
		me495_hw1::ME495Pub pub_msg;
		pub_msg.header.frame_id = "world";
		pub_msg.header.stamp = time_now;
		pub_msg.time = time;
		pub_msg.configuration = config;
		pubdemo.publish(pub_msg);
		return;
	}

	void subscribercb(const std_msgs::String &received) {
		std::string in = received.data;
		std::string out;
		for (std::string::reverse_iterator rev_it=in.rbegin(); rev_it!=in.rend(); ++rev_it)
			out.push_back(*rev_it);
		ROS_INFO("Manipulated String: %s",out.c_str());
	}

	bool srvcb(me495_hw1::ME495Srv::Request &req,
			   me495_hw1::ME495Srv::Response &resp) {
		uint32_t i = req.input;
		resp.output = i > 0 ? (int) log10 ((double) i) + 1 : 1;
		return true;
	}
};
	

int main(int argc, char** argv)
{
	ros::init(argc, argv, "me495_cl_demo");
	ME495Demo demo;
	ros::spin();
	return 0;
}

