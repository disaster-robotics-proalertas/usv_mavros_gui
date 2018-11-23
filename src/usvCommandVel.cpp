#include "include/waypointcontrol.h"
#include <numeric>
#include <chrono>
#include <thread>
#include <stdexcept>
#include <cmath>

//unsigned short ros_modes [] ={0, 80, 208, 64, 192, 88, 216, 92, 220, 66, 194};
// const unsigned short mode_freqs[] = {1000, 1300, 1400, 1500, 1700, 1900};
//                                  manual "    acro  auto  guided RTL

int main(int argc, char *argv[])
{    
    // starting ROS
    ros::init(argc, argv, "RCChannelsOverride");

    /// processing input parameters
    if(argc == 1 || std::string(argv[1])==std::string("-h") || std::string(argv[1])==std::string("--help"))
    {
        ROS_INFO("Usage: rosrun usv_mavros_gui_nogui_cmdvel <time_sec> <lx> <ly> <lz> <ax> <ay> <az>");
        return 0;
    }
    if(argc < 8 || argc > 8)
    {
        ROS_ERROR("Error: cmd_vel test requires 7 input parameters.");
        ROS_INFO("Usage: rosrun usv_mavros_gui_nogui_cmdvel <time_sec>  <lx> <ly> <lz> <ax> <ay> <az>");
        return 1;
    }
    /// Parsing parameters
    // Set experiment duration
    int duration;
    std::vector<double> vel(6, 0.0);
    try {
            duration = std::stoi(argv[1], nullptr, 0);
    } catch (std::invalid_argument& ia) {
        ROS_ERROR("Set experiment duration in seconds properly in argv[1]");
        ROS_INFO("Usage: rosrun usv_mavros_gui_nogui_cmdvel <time_sec> <lx> <ly> <lz> <ax> <ay> <az>");
        return 1;
    }
    // set velocity values
    for(int i = 0; i<6; ++i)
    {
        try {
            vel[i] = std::stoi(argv[2+i], nullptr, 0);
        } catch (std::invalid_argument& ia) 
	{
            ROS_ERROR("Set velocity value properly in argv[%i]", i);
            ROS_INFO("Usage: rosrun usv_mavros_gui_nogui_cmdvel <time_sec> <lx> <ly> <lz> <ax> <ay> <az>");
            return 1;
        }
    }
    // Quit if experiment time is greater than 60s
    if(duration>60)
    {
        ROS_ERROR("The experiment duration is greater than 1 minute, NOT SAFE for the USV");
        ROS_ERROR("Exiting");
        return 0;
    }

    /// Config mavros
    WaypointControl ctl;
    //set stream rate
    //ctl.setStreamRate(0, 10, true);
    // Setting parameter SYSID_MYCGS is required for override
    int integer = 0;
    double real = 0;
    while(integer!=1)
    {
        ctl.setParam("SYSID_MYGCS",1, 1.0f);
        ctl.getParam("SYSID_MYGCS", integer, real);
        ros::spinOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    // Arming
    if(ctl.arming(true))
        ROS_INFO("Success: USV IS ARMED");
    else
        ROS_ERROR("FAILURE: USV IS NOT ARMED OR WAS ALREADY ARMED BEFORE");
    ros::spinOnce();
    if(ctl.setMode(216)) // 220 auto armed // 192 manual armed // GUIDED_ARMED = 216
        ROS_INFO("Set Mode Success");
    else
        ROS_ERROR("Set Mode FAILURE");
    ros::spinOnce();
    // Take over RC and config the unmanned system
    ctl.takeOverRC();
    ros::spinOnce();

    /// Experiment Loop
    // Initialize RC channel
    std::vector<unsigned short> vals(ctl.servos);
    for(int i=0; i<8;++i)
        vals[i]=1500;
    vals[5]=1700; // Channel which control usv operation mode: GUIDED is ~1700
    // experiment start time
    auto start = std::chrono::high_resolution_clock::now();
    while (true)
    {
        // Lap start time
        auto lapStart = std::chrono::high_resolution_clock::now();
        ctl.setRCChannels(vals);
	ctl.cmdVel(vel[0], vel[1], vel[2], vel[3], vel[4], vel[5]);
        ros::spinOnce();
        // get current time
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> diff = end-start;
        // Check for experiment end
        if(diff.count() >= duration)
            break;
        std::chrono::duration<double> lapDiff = end-lapStart;
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>((std::max(0,20-static_cast<int>(lapDiff.count()))))));
    }


    /// Mavros configuration to reset everything fpr the conventional RC
    // giving control back to RC
    vals[5]=1100;
    ctl.setRCChannels(vals); // set MANUAL again
    ros::spinOnce();
    ctl.giveBackRC();
    ros::spinOnce();
    // Changing to MANUAL mode disarmed
    ctl.setMode(192);
    integer = 1;
    real = 1.0f;
    while(integer!=0 && real!=0.0)
    {
        ctl.setParam("SYSID_MYGCS",0, 0.0);
        ctl.getParam("SYSID_MYGCS", integer, real);
        ros::spinOnce();
    }

    /// Experiment ended
    return 0;
}
