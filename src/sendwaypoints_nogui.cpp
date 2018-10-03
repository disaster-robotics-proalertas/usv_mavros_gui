#include "waypointcontrol.h"
#include <limits>

int main(int argc, char *argv[])
{
   ros::init(argc, argv, "waypoint_editor");

	WaypointControl ctl;
	int s=-1;
	ctl.loadFromFile(argv[1]);
	ctl.count(s);
	ROS_INFO("Here is the count %d", s);
	ros::spin();
	return 0;
}
