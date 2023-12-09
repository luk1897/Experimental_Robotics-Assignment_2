#include "ros/ros.h"
#include "../include/action_handler.h"
#include <unistd.h>
#include "assignment_2/RobotVision.h"

int id;

namespace KCL_rosplan {
	ActionInterfaceExtended::ActionInterfaceExtended(ros::NodeHandle &nh) {
	// here the initialization
	}
	
	bool ActionInterfaceExtended::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
			// here the implementation of the action
			std::cout << "Searching for " << msg->parameters[2].value << " in " << msg->parameters[1].value << std::endl;
			
			sleep(5); // inviare la velocità angolare al server, fare switch case dell'id dei marker e il check id
			
			//ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
			ROS_INFO("ID: %d", id);
			sleep(10);
			return true;
		}
}

void vision_cb(const assignment_2::RobotVision::ConstPtr& msg){
	
	//ROS_INFO("Vision subscriber@[%d]", msg->id);
	
	id = msg->id;
}


int main(int argc, char **argv) {
	
	ros::init(argc, argv, "search_interface", ros::init_options::AnonymousName);
	
	ros::NodeHandle nh("~");
	
	ros::Subscriber sub = nh.subscribe("info_vision", 10, vision_cb);
	
	// inizializzare client al rotation server
	
	KCL_rosplan::ActionInterfaceExtended my_aci(nh);
	
	my_aci.runActionInterface();
	
	return 0;
}
