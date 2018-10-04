#include "include/waypointcontrol.h"
#include <chrono>
#include <thread>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "RCChannelsOverride");
    WaypointControl ctl;

    //set stream rate
    ctl.setStreamRate(0,100,true);

    // Setting parameter SYSID_MYCGS is required for override
    int integer = 0;
    double real = 0;
    ctl.getParam("SYSID_MYGCS", integer, real);
    ctl.setParam("SYSID_MYGCS",1, 1);
    //    ctl.getParam("SYSID_MYGCS", integer, real);
    while(integer!=1)
        ctl.getParam("SYSID_MYGCS", integer, real);

    // Changing Mode to MANUAL mode armed
    ctl.setMode(192); // manual armed
    ctl.takeOverRC();
    std::vector<unsigned short> vals(ctl.servos);
    ctl.setRCChannels(vals);
    ros::spinOnce();

    for(int i=0;i<100;++i)
    {
        ctl.setRCChannels(vals);
        vals[2]+=3;
        vals[0]+=3;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        ros::spinOnce();
    }
    for(int i=0;i<100;++i)
    {
        vals[2]-=3;
        vals[0]-=3;
        ctl.setRCChannels(vals);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        ros::spinOnce();
    }

    // Trying Twist
    double lx = 0.00;
    double ly = 0.00;
    for(int i=0;i<100;++i)
    {
        ctl.cmdVel(lx,ly,0, 0,0,0);
        lx+=0.005;
        ly+=0.005;
        ros::spinOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    for(int i=0;i<100;++i)
    {
        lx-=0.005;
        ly-=0.005;
        ctl.cmdVel(lx,ly,0, 0,0,0);
        ros::spinOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    // giving control back to RC
    ctl.giveBackRC();
    // Changing to MANUAL mode disarmed
    ctl.setMode(64); // manual disarmed
    ctl.setParam("SYSID_MYGCS", 0, 0.0f);
    ctl.getParam("SYSID_MYGCS", integer, real);
    ros::spinOnce();
    return 0;
}

