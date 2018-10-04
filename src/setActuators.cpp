#include "include/waypointcontrol.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "RCChannelsOverride");
    WaypointControl ctl;
    ctl.setMode(0);
    ctl.takeOverRC();
    ctl.giveBackRC();
    ros::spin();
    return 0;
}

