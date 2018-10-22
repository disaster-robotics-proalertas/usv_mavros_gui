#ifndef WAYPOINTCONTROL_H
#define WAYPOINTCONTROL_H

#include <vector>
#include <string>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "std_msgs/String.h"

#include "mavros_msgs/WaypointList.h"
#include "mavros_msgs/WaypointClear.h"
#include "mavros_msgs/WaypointPush.h"
#include "mavros_msgs/WaypointPull.h"
#include "mavros_msgs/WaypointSetCurrent.h"
#include "mavros_msgs/CommandCode.h"
#include "mavros_msgs/CommandHome.h"
#include "mavros_msgs/RCIn.h"
#include "mavros_msgs/OverrideRCIn.h"
#include "mavros_msgs/ManualControl.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/ParamGet.h"
#include "mavros_msgs/ParamSet.h"
#include "mavros_msgs/StreamRate.h"
#include "mavros_msgs/Thrust.h"
#include "mavros_msgs/CommandTOL.h"
#include "mavros_msgs/HomePosition.h"
#include "mavros_msgs/SetMavFrame.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/CommandInt.h"
#include "mavros_msgs/CommandLong.h"




enum WaypointTraversal {acceptanceRadius, passThrough, fromFile};
enum USVOrientationAtWaypoint{matters, ignore};
enum RCManualOverride{CHAN_RELEASE = static_cast<unsigned short>(0u), CHAN_ARREST = static_cast<unsigned short>(65535u)};

class WaypointControl
{
public:
    WaypointControl();

    // movement control
    bool setYawSpeed(float angleDeg, float normalized_yaw_speed);
    bool cmdVel(double lx, double ly, double lz, double ax, double ay, double az);
    bool cmd_vel(double lx, double ly, double lz, double ax, double ay, double az);
    bool thrust(float t);
    bool startEngine(float start, float coldStart);
    bool enableFence(float fenceEnable);
    bool deployVessel(float latitude, float longitude, float altitude); // "takeoff"
    bool collectVessel(float latitude, float longitude, float altitude); // "land"

    // waypoint control
    bool clear();
    bool push(int start_index, std::vector<mavros_msgs::Waypoint>& wps);
    bool count(int& c);
    void list(std::vector<mavros_msgs::Waypoint>& wps);
    bool loadFromFile(const std::string filename, WaypointTraversal method = WaypointTraversal::fromFile, int radius = 0);
    bool loadFromString(const std::string data, WaypointTraversal method = WaypointTraversal::fromFile, int radius = 0);
    bool setHome(double latitude, double longitude);

    // set controller values: require usv base config first
    bool setRCChannels(std::vector<unsigned short> values=std::vector<unsigned short>(8,65535u));

    // usv base config
    bool arming(bool setArmed);
    bool setMode(unsigned char mode=0, std::string custom=std::string(""));
    bool getParam(std::string param_id, int &intVal, double &realVal);
    bool setParam(std::string param_id, int intVal, double realVal);
    bool takeOverRC();
    bool giveBackRC();
    bool overrideRCChannels(std::vector<RCManualOverride> values=std::vector<RCManualOverride>(8,RCManualOverride::CHAN_RELEASE));
    bool setStreamRate(unsigned char stream_id, unsigned short message_rate, bool onoff);
    bool manualControl(float x, float y, float z, float r, unsigned short buttons);
    bool setGuidedMode(bool val);

    virtual void waypointCallback(const mavros_msgs::WaypointList& wps);
    virtual void homePositionCallback(const mavros_msgs::HomePosition& h);

   std::vector<unsigned short> servos;

protected:
   ros::NodeHandle n;
   ros::ServiceClient client;
   ros::Publisher pub;
   ros::Subscriber subWps;
   ros::Subscriber subHome;

   std::vector<mavros_msgs::Waypoint>& waypoints;
   mavros_msgs::HomePosition home;
};

#endif // WAYPOINTCONTROL_H
