#ifndef WAYPOINTCONTROL_H
#define WAYPOINTCONTROL_H

#include "ros/ros.h"
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
#include <vector>
#include <string>


enum WaypointTraversal {acceptanceRadius, passThrough, fromFile};
enum USVOrientationAtWaypoint{matters, ignore};
enum RCManualOverride{CHAN_RELEASE = 0u, CHAN_ARREST = 65535u};

class WaypointControl
{
public:
    WaypointControl();
    bool clear();
    bool push(int start_index, std::vector<mavros_msgs::Waypoint>& wps);
    bool count(int& c);
    void list(std::vector<mavros_msgs::Waypoint>& wps);
    bool loadFromFile(const std::string filename, WaypointTraversal method = WaypointTraversal::fromFile, int radius = 0);
    bool loadFromString(const std::string data, WaypointTraversal method = WaypointTraversal::fromFile, int radius = 0);
    bool setHome(double latitude, double longitude);
    bool setMode(unsigned char mode=0, std::string custom=std::string());
    bool getParam(std::string param_id, int& intVal, double& realVal);
    bool overrideRCChannels(std::vector<RCManualOverride> values=std::vector<RCManualOverride>(8,RCManualOverride::CHAN_RELEASE));
    bool setRCChannels(std::vector<unsigned char> values=std::vector<unsigned char>(8,static_cast<unsigned char>(65535u)));
    bool takeOverRC();
    bool giveBackRC();

    void waypointCallback(const mavros_msgs::WaypointList& wps);


protected:
   ros::NodeHandle n;
   ros::ServiceClient client;
   ros::Publisher pub;
   ros::Subscriber sub;
   std::vector<mavros_msgs::Waypoint>& waypoints;
   std::vector<unsigned short> servos;

};

#endif // WAYPOINTCONTROL_H
