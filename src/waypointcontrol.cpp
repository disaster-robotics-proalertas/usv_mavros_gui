#include "include/waypointcontrol.h"
#include <iostream>
#include <fstream>
#include <sstream>

unsigned short servoStopped [] ={1550, 0, 1200, 0, 0, 0, 0, 0};

std::vector<mavros_msgs::Waypoint> globalWaypoints;


WaypointControl::WaypointControl() :
    n(),
    client(),
    sub(),
    pub(),
    waypoints(globalWaypoints),
    servos(servoStopped, servoStopped + sizeof(servoStopped) / sizeof(servoStopped[0]))
{
    sub = n.subscribe("/mavros/mission/waypoints", 1, &WaypointControl::waypointCallback, this);
}

bool WaypointControl::clear()
{
    // Clear waypoints
    client = n.serviceClient<mavros_msgs::WaypointClear>("/mavros/mission/clear");
    mavros_msgs::WaypointClear clearService;
    if (client.call(clearService))
    {
        if(clearService.response.success)
            ROS_INFO("Waypoint list cleared");
        else
            ROS_ERROR("Clear service was called, but returned false");
    }
    else
    {
        ROS_ERROR("Failed to call clear mission service");
        return false;
    }

    return clearService.response.success;
}

bool WaypointControl::push(int start_index, std::vector<mavros_msgs::Waypoint>& wps)
{
    ros::ServiceClient client = n.serviceClient<mavros_msgs::WaypointPush>("/mavros/mission/push");
    mavros_msgs::WaypointPush pushService;
    pushService.request.start_index=start_index;
    pushService.request.waypoints=wps;

    if (client.call(pushService))
    {
        if(pushService.response.success)
            ROS_INFO("Qtd Waypoints: %d", pushService.response.wp_transfered);
        else
            ROS_INFO("Service was contacted, but Waypoints failed to be transferred");
    }
    else
    {
        ROS_ERROR("Failed to call push mission service");
        return false;
    }
}


bool WaypointControl::count(int& c)
{
    // reading waypoints -- cannot read waypoints, just count them
    // returns -1 if error
    client = n.serviceClient<mavros_msgs::WaypointPull>("/mavros/mission/pull");
    mavros_msgs::WaypointPull pullService;
    if (client.call(pullService))
    {
        if(pullService.response.success)
        {
            ROS_INFO("Qtd Waypoints: %d", pullService.response.wp_received);
            c = pullService.response.wp_received;
        }
        else
        {
            ROS_ERROR("Pull service was called, but failed");
            c = -1;
        }
    }
    else
    {
        ROS_ERROR("Failed to call pull mission service");
        c=-1;
        return false;
    }

    return pullService.response.wp_received;
}

void WaypointControl::list(std::vector<mavros_msgs::Waypoint>& wps)
{
    wps = waypoints;
    for(int i=0;i<waypoints.size();++i)
    {
        ROS_INFO("Waypoint [%d]: lat %g, long %g", i, wps[i].x_lat, wps[i].y_long);
    }
}

bool WaypointControl::loadFromFile(const std::string filename, WaypointTraversal method, int radius)
{
    // open filename
    std::ifstream source;
    source.open(filename);

    // check if file is open
    if (source.is_open())
    {
        // clean waypoints vector
        waypoints.clear();

        // first line indicates which wypoint is current
        std::string firstLine;
        std::getline(source, firstLine);
        std::istringstream inFirst(firstLine);
        int current=-1;
        inFirst >> current;

        // initialize a waypoint message, each waypoint is stored in waypoints
        mavros_msgs::Waypoint wp;
        wp.frame = mavros_msgs::Waypoint::FRAME_GLOBAL;
        wp.command = mavros_msgs::CommandCode::NAV_WAYPOINT;
        wp.is_current = false;
        wp.autocontinue = true;
        // random altitude, just not 0
        wp.z_alt = 1;
        // Hold time in decimal seconds. (ignored by fixed wing, time to stay at waypoint for rotary wing)
        wp.param1 = 0;
        // Desired yaw angle at waypoint (rotary wing). NaN for unchanged.
        wp.param4 = std::numeric_limits<double>::quiet_NaN();
        for(std::string line; std::getline(source, line); )   //read stream line by line
        {
            //make a stream for the line
            std::istringstream in(line);
            // set lat and long from file
            in >> wp.x_lat >> wp.y_long;

            // Assuming params 2 and 3 are mutualy exclusive...
            switch(method){
            case (WaypointTraversal::passThrough):
            {
                wp.param2 = 0; // Acceptance radius in meters (if the sphere with this radius is hit, the waypoint counts as reached)
                wp.param3 = radius; // 0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.
                break;
            }
            case (WaypointTraversal::acceptanceRadius):
            {
                wp.param2 = radius; // Acceptance radius in meters (if the sphere with this radius is hit, the waypoint counts as reached)
                wp.param3 = 0; // 0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.
                break;
            }
            case (WaypointTraversal::fromFile):
            {
                in >> wp.param2 >> wp.param3;
                break;
            }
            default:
            {
                wp.param2 = radius; // Acceptance radius in meters (if the sphere with this radius is hit, the waypoint counts as reached)
                wp.param3 = 0; // 0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.
            }
            }

            // store waypoint
            waypoints.push_back(wp);
        }

        // if failed to determine current, set first
        if(current>waypoints.size()-1 || current < 0)
            // set first
            waypoints[0].is_current = true;
        else
            // set current
            waypoints[current].is_current = true;

        // Set last one as not auto continue
        if(waypoints.size()-1 >= 0)
            waypoints[waypoints.size()-1].autocontinue = false;
        else
        {
            ROS_ERROR("Waypoints file has no waypoints");
            return false;
        }
    }
    else
    {
        ROS_ERROR("Unable to open file");
        return false;
    }

    // Send waypoints to ROS
    if(clear() && push(0, waypoints))
        ROS_INFO("Waypoints stored");
    else
    {
        ROS_ERROR("Could not store waypoints from file");
        return false;
    }

    return true;
}

bool WaypointControl::loadFromString(const std::string data, WaypointTraversal method, int radius)
{
    // clean waypoints vector
    waypoints.clear();

    // first line indicates which wypoint is current
    std::istringstream waypointrawinfo(data);
    std::string firstLine;
    std::getline(waypointrawinfo, firstLine);

    int current = -1;
    waypointrawinfo >> current;

    // initialize a waypoint message, each waypoint is stored in waypoints
    mavros_msgs::Waypoint wp;
    wp.frame = mavros_msgs::Waypoint::FRAME_GLOBAL;
    wp.command = mavros_msgs::CommandCode::NAV_WAYPOINT;
    wp.is_current = false;
    wp.autocontinue = true;
    // random altitude, just not 0
    wp.z_alt = 1;
    // Hold time in decimal seconds. (ignored by fixed wing, time to stay at waypoint for rotary wing)
    wp.param1 = 0;
    // Desired yaw angle at waypoint (rotary wing). NaN for unchanged.
    wp.param4 = std::numeric_limits<double>::quiet_NaN();
    for(std::string line; std::getline(waypointrawinfo, line);)   //read stream line by line
    {
        //make a stream for the line
        std::istringstream in(line);
        // set lat and long from file
        in >> wp.x_lat >> wp.y_long;

        // Assuming params 2 and 3 are mutualy exclusive...
        switch(method){
        case (WaypointTraversal::passThrough):
        {
            wp.param2 = 0; // Acceptance radius in meters (if the sphere with this radius is hit, the waypoint counts as reached)
            wp.param3 = radius; // 0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.
            break;
        }
        case (WaypointTraversal::acceptanceRadius):
        {
            wp.param2 = radius; // Acceptance radius in meters (if the sphere with this radius is hit, the waypoint counts as reached)
            wp.param3 = 0; // 0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.
            break;
        }
        case (WaypointTraversal::fromFile):
        {
            in >> wp.param2 >> wp.param3;
            break;
        }
        default:
        {
            wp.param2 = radius; // Acceptance radius in meters (if the sphere with this radius is hit, the waypoint counts as reached)
            wp.param3 = 0; // 0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.
        }
        }

        // store waypoint
        waypoints.push_back(wp);
    }

    // if failed to determine current, set first
    if(current>waypoints.size()-1 || current < 0)
        // set first
        waypoints[0].is_current = true;
    else
        // set current
        waypoints[current].is_current = true;

    // Set last one as not auto continue
    if(waypoints.size()-1 >= 0)
        waypoints[waypoints.size()-1].autocontinue = false;
    else
    {
        ROS_ERROR("Waypoints file has no waypoints");
        return false;
    }

    // Send waypoints to ROS
    if(clear() && push(0, waypoints))
        ROS_INFO("Waypoints stored");
    else
    {
        ROS_ERROR("Could not store waypoints from file");
        return false;
    }

    return true;
}

bool WaypointControl::setHome(double latitude, double longitude)
{
    // Clear waypoints
    client = n.serviceClient<mavros_msgs::CommandHome>("/mavros/cmd/set_home");
    mavros_msgs::CommandHome setHomeService;
    setHomeService.request.current_gps = false;
    setHomeService.request.latitude = latitude;
    setHomeService.request.longitude = longitude;
    if (client.call(setHomeService))
    {
        if(setHomeService.response.success)
            ROS_INFO("New home is set");
        else
            ROS_ERROR("set home service was called, but returned false");
    }
    else
    {
        ROS_ERROR("Failed to call set home service");
        return false;
    }

    return setHomeService.response.success;
}

bool WaypointControl::overrideRCChannels(std::vector<RCManualOverride> values)
{
    // Check if number of channels is correct
    int numChannels=8;
    mavros_msgs::OverrideRCIn s;

    if(values.size() > 0 && values.size() < 8){
        ROS_WARN("RC Control config will occur only over %d channels", static_cast<int>(values.size()));
        numChannels=values.size();
    }
    else if(values.size() > 0 && values.size() <= 8){
        // Send override command
        for(int i = 0; i< numChannels; ++i)
            s.channels[i] = values[i];

        pub = n.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 1000);
        pub.publish(s);
    }
    else{
        ROS_ERROR("Number of servos is incprrect %d", static_cast<int>(values.size()));
        return false;
    }
    //    mavros_msgs::ManualControl c;
    //    c.x = 10;
    //    c.y = 10;
    //    c.z = 0;
    //    c.r = 10;

    //    pub = n.advertise<mavros_msgs::ManualControl>("/mavros/manual_control/send", 1000);
    //    pub.publish(c);

    return true;
}

bool WaypointControl::takeOverRC(){
    return overrideRCChannels(std::vector<RCManualOverride>(8,RCManualOverride::CHAN_ARREST));
}

bool WaypointControl::giveBackRC(){
    return overrideRCChannels(std::vector<RCManualOverride>(8,RCManualOverride::CHAN_RELEASE));
}

bool WaypointControl::setMode(unsigned char mode, std::string custom)
{   // Base Modes
    //unsigned char MAV_MODE_PREFLIGHT = 0
    //unsigned char MAV_MODE_STABILIZE_DISARMED = 80
    //unsigned char MAV_MODE_STABILIZE_ARMED = 208
    //unsigned char MAV_MODE_MANUAL_DISARMED = 64
    //unsigned char MAV_MODE_MANUAL_ARMED = 192
    //unsigned char MAV_MODE_GUIDED_DISARMED = 88
    //unsigned char MAV_MODE_GUIDED_ARMED = 216
    //unsigned char MAV_MODE_AUTO_DISARMED = 92
    //unsigned char MAV_MODE_AUTO_ARMED = 220
    //unsigned char MAV_MODE_TEST_DISARMED = 66
    //unsigned char MAV_MODE_TEST_ARMED = 194

    // Custom Modes are strings which are specific to each unmanned vehicle
    // http://wiki.ros.org/mavros/CustomModes
    //ex: APM:Rover
    //Numeric	String
    //0		MANUAL
    //2		LEARNING
    //3		STEERING
    //4		HOLD
    //10	AUTO
    //11	RTL
    //15	GUIDED
    //16	INITIALISING

    client = n.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    mavros_msgs::SetMode setModeService;
    if(!custom.empty())
    {
        setModeService.request.base_mode = 0;
        setModeService.request.custom_mode = custom;
    }
    else
        setModeService.request.base_mode = mode;

    if (client.call(setModeService)) {
        ROS_INFO("Send OK %d Value:", setModeService.response.mode_sent);
    } else {
        ROS_ERROR("Failed SetMode");
        return false;
    }
    return setModeService.response.mode_sent;
}

void WaypointControl::waypointCallback(const mavros_msgs::WaypointList& wps)
{
    ROS_INFO("Current Waypoint %d ", wps.current_seq);
    waypoints = wps.waypoints;
    for(int i=0;i<waypoints.size();++i)
        ROS_INFO("Waypoint %d received: lat %g, long %g", i, wps.waypoints[i].x_lat, wps.waypoints[i].y_long);
}

bool WaypointControl::setRCChannels(std::vector<unsigned char> values)
{
    // Check if number of channels is correct
    int numChannels=8;
    mavros_msgs::OverrideRCIn s;

    if(values.size() > 0 && values.size() < 8){
        ROS_WARN("RC Control values span only over %d channels", static_cast<int>(values.size()));
        numChannels=values.size();
    }
    else if(values.size() > 0 && values.size() <= 8){
        // Send override command
        for(int i = 0; i< numChannels; ++i)
            s.channels[i] = values[i];

        pub = n.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 1000);
        pub.publish(s);
    }
    else{
        ROS_ERROR("Number of servos is incprrect %d", static_cast<int>(values.size()));
        return false;
    }

}
