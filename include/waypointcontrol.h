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
#include <vector>
#include <string>


enum WaypointTraversal {acceptanceRadius, passThrough, fromFile};
enum USVOrientationAtWaypoint{matters, ignore};

class WaypointControl
{
public:
    WaypointControl();
    bool setActuators(std::vector<unsigned short> values);
    bool clear();
    bool push(int start_index, std::vector<mavros_msgs::Waypoint>& wps);
    bool count(int& c);
    void list(std::vector<mavros_msgs::Waypoint>& wps);
    bool loadFromFile(const std::string filename, WaypointTraversal method = WaypointTraversal::fromFile, int radius = 0);
    bool loadFromString(const std::string data, WaypointTraversal method = WaypointTraversal::fromFile, int radius = 0);
    bool setHome(double latitude, double longitude);
protected:
   ros::NodeHandle n;
   ros::ServiceClient client;
   ros::Publisher pub;
   ros::Subscriber sub;
   std::vector<mavros_msgs::Waypoint>& waypoints;
   std::vector<unsigned short> servos;

};

#endif // WAYPOINTCONTROL_H
