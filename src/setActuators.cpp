#include "include/waypointcontrol.h"
#include <chrono>
#include <thread>

unsigned short ros_modes [] ={0, 80, 208, 64, 192, 88, 216, 92, 220, 66, 194};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "RCChannelsOverride");
    WaypointControl ctl;

    //set stream rate
    ctl.setStreamRate(0,5,true);
    //    for(int m = 0; m<11; ++m)
    //    {
    // Setting parameter SYSID_MYCGS is required for override
    int integer = 0;
    double real = 0;
    while(integer!=1 && real!=1.0)
    {
        ctl.setParam("SYSID_MYGCS",1, 1);
        ctl.getParam("SYSID_MYGCS", integer, real);
    }
    //        while(integer!=1 && real!=1.0)
    //        {
    //            ctl.setParam("SYSID_THISMAV",1, 1);
    //            ctl.getParam("SYSID_THISMAV", integer, real);
    //        }
    //        ctl.getParam("SYSID", integer, real);
    //        ROS_INFO("SYSTEM_ID %d, %g", integer, real);

    //        ctl.getParam("SYSID", integer, real);
    //        ROS_INFO("COMPONENT_ID %d, %g", integer, real);



    // Disabling Fence
    // Changing Mode
    if(ctl.setMode(216)) // 220 auto armed // 192 manual armed // GUIDED_ARMED = 216
        ROS_INFO("Set Mode Success");
    else
        ROS_ERROR("Set Mode FAILURE");

    if(ctl.enableFence(0)) //  enable? (0=disable, 1=enable, 2=disable_floor_only)
        ROS_INFO("Fence disabled");
    else
        ROS_ERROR("Could not disable FENCE");

    // Enabling Guided Mode
    if(ctl.setGuidedMode(true)) //  enable? (0=disable, 1=enable, 2=disable_floor_only)
        ROS_INFO("Guided Mode enabled");
    else
        ROS_ERROR("Guided Mode could not be enabled");

    // Arming DOES NOT WORK
    if(ctl.arming(false))
        ROS_INFO("Success: USV IS DISARMED");
    else
        ROS_WARN("FAILURE: COULD NO DISARM,  PROBABLY ALREADY DISARMED");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    if(ctl.arming(true))
        ROS_INFO("Success: USV IS ARMED");
    else
        ROS_ERROR("FAILURE: USV IS NOT SARMED OR WAS ALREADY ARMED BEFORE");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    for(int i = 0; i< 100;++i)
    {
        ctl.setYawSpeed(3000, 0.5);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        ros::spinOnce();
    }

//    ctl.startEngine(1,0);

//    // DEPLOY VESSEL DOES WORK
//    ctl.deployVessel(-30.0612, -51.1747, 20);

//    // SetRCChannel
//    std::vector<unsigned short> vals(ctl.servos);
//    //        ctl.setRCChannels(vals);
//    ros::spinOnce();
//    ctl.takeOverRC();
//    ros::spinOnce();

//    for(int i=0;i<100;++i)
//    {
//        ctl.setRCChannels(vals);

//        vals[2]+=3;
//        vals[0]+=3;
//        std::this_thread::sleep_for(std::chrono::milliseconds(20));
//        ros::spinOnce();
//    }
//    for(int i=0;i<100;++i)
//    {
//        vals[2]-=3;
//        vals[0]-=3;
//        ctl.setRCChannels(vals);
//        std::this_thread::sleep_for(std::chrono::milliseconds(20));
//        ros::spinOnce();
//    }

//    // Manual Control
//    ctl.manualControl(0,0,0,0,0);
//    ros::spinOnce();
//    std::this_thread::sleep_for(std::chrono::milliseconds(100));
//    ctl.manualControl(0,0,0,0,0);
//    ros::spinOnce();
//    std::this_thread::sleep_for(std::chrono::milliseconds(100));
//    ctl.manualControl(1,1,1,1,0);
//    ros::spinOnce();
//    std::this_thread::sleep_for(std::chrono::milliseconds(100));
//    ctl.manualControl(1,1,1,1,0);
//    ros::spinOnce();
//    std::this_thread::sleep_for(std::chrono::milliseconds(100));
//    ctl.manualControl(500,500,500,500,0);
//    ros::spinOnce();
//    std::this_thread::sleep_for(std::chrono::milliseconds(100));
//    ctl.manualControl(500,500,500,500,0);
//    ros::spinOnce();
//    std::this_thread::sleep_for(std::chrono::milliseconds(100));
//    ctl.manualControl(1000,1000,1000,1000,0);
//    ros::spinOnce();
//    std::this_thread::sleep_for(std::chrono::milliseconds(100));
//    ctl.manualControl(1000,1000,1000,1000,0);
//    ros::spinOnce();
//    std::this_thread::sleep_for(std::chrono::milliseconds(100));
//    ctl.manualControl(2000,2000,2000,2000,0);
//    ros::spinOnce();
//    std::this_thread::sleep_for(std::chrono::milliseconds(100));
//    ctl.manualControl(2000,2000,2000,2000,0);
//    ros::spinOnce();
//    std::this_thread::sleep_for(std::chrono::milliseconds(100));
//    float x=0, y=0, z=0, r=0;
//    unsigned short buttons = 0;
//    for(int i=0;i<100;++i)
//    {
//        ctl.manualControl(x,y,z,r,buttons);
//        r+=3;
//        x+=3;
//        y+=3;
//        z+=3;
//        ros::spinOnce();
//        std::this_thread::sleep_for(std::chrono::milliseconds(100));
//    }
//    for(int i=0;i<100;++i)
//    {
//        r-=3;
//        x-=3;
//        y-=3;
//        z-=3;
//        ctl.manualControl(x,y,z,r,buttons);
//        ros::spinOnce();
//        std::this_thread::sleep_for(std::chrono::milliseconds(100));
//    }

//    // giving control back to RC
//    ctl.giveBackRC();


//    //   Twist
//    double lx = 0.00;
//    double ly = 0.00;
//    double lz = 0.00;
//    for(int i=0;i<100;++i)
//    {
//        ctl.cmdVel(lx,ly,lz, 0,0,0);
//        ctl.cmd_vel(lx,ly,lz, 0,0,0);
//        ctl.thrust(lx);
//        lx+=0.005;
//        ly+=0.005;
//        lz+=0.005;
//        ros::spinOnce();
//        std::this_thread::sleep_for(std::chrono::milliseconds(100));
//    }
//    for(int i=0;i<100;++i)
//    {
//        lx-=0.005;
//        ly-=0.005;
//        lz-=0.005;
//        ctl.thrust(lx);
//        ctl.cmdVel(lx,ly,lz, 0,0,0);
//        ctl.cmd_vel(lx,ly,lz, 0,0,0);
//        ros::spinOnce();
//        std::this_thread::sleep_for(std::chrono::milliseconds(100));
//    }

    // Changing to MANUAL mode disarmed
    ctl.setMode(0);
    ctl.setParam("SYSID_MYGCS", 0, 0.0f);
    ctl.getParam("SYSID_MYGCS", integer, real);
    ros::spinOnce();
    //    }
    return 0;
}
