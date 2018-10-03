#include "include/waypointcontrol.h"

int main(int argc, char *argv[])
{
	if(argc < 4)
	{
		std::cerr << "insufficient servo values" << std::endl;		
		return 1;		
	}
	if(argc > 9)
	{
		std::cerr << "There are too many servo values: max 8, using 3 for the USV" << argc << std::endl;		
		return 1;		
	}

	std::vector<unsigned short> v(8);
	for (int i = 1; i < argc; ++i)
	{
		std::cout << "Converting value " << argv[i] << std::endl;
		std::string commands(argv[i]);
		std::istringstream servorawinfo(commands);
		unsigned short val = 0;
		servorawinfo >> val;
		v[i-1] = val;
	}
	
   ros::init(argc, argv, "setServos");

	WaypointControl ctl;
	ctl.setActuators(v);
	ctl.setActuators(v);
	ros::spin();
	return 0;
}

