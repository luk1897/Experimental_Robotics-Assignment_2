#include "../include/action_handler.h"
#include <unistd.h>

namespace KCL_rosplan {
	ActionInterfaceExtended::ActionInterfaceExtended(ros::NodeHandle &nh) {
	// here the initialization
	}
	
	bool ActionInterfaceExtended::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
			// here the implementation of the action
			std::cout << "Going from " << msg->parameters[1].value << " to " << msg->parameters[2].value << std::endl;
			
			sleep(5); // qui bisogna mettere wp0 e inviare le coordinate al server
			
			ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
			
			return true;
		}
}


int main(int argc, char **argv) {
	
	ros::init(argc, argv, "go-home_interface", ros::init_options::AnonymousName);
	
	ros::NodeHandle nh("~");
	
	//inizializzare client to movebase
	
	KCL_rosplan::ActionInterfaceExtended my_aci(nh);
	
	my_aci.runActionInterface();
	
	return 0;
}
