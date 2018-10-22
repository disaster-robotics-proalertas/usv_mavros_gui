#include "include/waypointcontrol.h"
#include <iostream>
#include <fstream>
#include <sstream>

unsigned short servoStopped [] ={1550, 0, 1200, 0, 0, 0, 0, 0};

std::vector<mavros_msgs::Waypoint> globalWaypoints;

int seq = 0;

WaypointControl::WaypointControl() :
    n(),
    client(),
    subWps(),
    subHome(),
    pub(),
    waypoints(globalWaypoints),
    servos(servoStopped, servoStopped + sizeof(servoStopped) / sizeof(servoStopped[0]))
{
    subWps = n.subscribe("/mavros/mission/waypoints", 1, &WaypointControl::waypointCallback, this);
    subHome = n.subscribe("/mavros/home_position/home", 1, &WaypointControl::homePositionCallback, this);

}

/////////////////////////////////////////////////
/// Callbacks                                 ///
/////////////////////////////////////////////////
void WaypointControl::homePositionCallback(const mavros_msgs::HomePosition& h)
{
    home = h;
    ROS_INFO("Current Home Position: latitude %g, longitude %g", h.geo.latitude, h.geo.longitude);
    ROS_INFO("Current Home Position: x %g, y %g, z %g", h.position.x, h.position.y, h.position.z);
}

void WaypointControl::waypointCallback(const mavros_msgs::WaypointList& wps)
{
    ROS_INFO("Current Waypoint %d ", wps.current_seq);
    waypoints = wps.waypoints;
    for(int i=0;i<waypoints.size();++i)
        ROS_INFO("Waypoint %d received: lat %g, long %g", i, wps.waypoints[i].x_lat, wps.waypoints[i].y_long);
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
            s.channels[i] = static_cast<unsigned short>(values[i]);

        pub = n.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 10);
        pub.publish(s);
        ROS_INFO("Override/Relase sent %hu", s.channels[0]);
    }
    else{
        ROS_ERROR("Number of servos is incprrect %d", static_cast<int>(values.size()));
        return false;
    }
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

//    Value	Meaning
//      0	Manual
//      1	Acro
//      3	Steering
//      4	Hold
//      5	Loiter
//      6	Follow
//      7	Simple
//      10	Auto
//      11	RTL
//      12	SmartRTL
//      15	Guided

    client = n.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    mavros_msgs::SetMode setModeService;

    if(!custom.empty())
    {
        setModeService.request.base_mode = 0;
        setModeService.request.custom_mode = custom;
    }
    else
    {
        setModeService.request.base_mode = 0;
        setModeService.request.base_mode = mode;
    }
    if (client.call(setModeService)) {
        if(!setModeService.response.mode_sent)
            ROS_ERROR("SetMode called but failed: %d", setModeService.response.mode_sent);
         else
            ROS_INFO("Mode set to: %d %s",mode, custom.c_str());
    } else {
        ROS_ERROR("Failed call SetMode");
        return false;
    }
    return setModeService.response.mode_sent;
}



bool WaypointControl::setRCChannels(std::vector<unsigned short> values)
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

        pub = n.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 3);
        pub.publish(s);
        ROS_INFO("Data sent to RC Channel[0]=%hu Channel[2]=%hu", values[0], values[2]);

    }
    else{
        ROS_ERROR("Number of servos is incprrect %d", static_cast<int>(values.size()));
        return false;
    }

    return true;
}

bool WaypointControl::getParam(std::string param_id, int& intVal, double& realVal)
{
    ros::ServiceClient getParamService = n.serviceClient<mavros_msgs::ParamGet>("/mavros/param/get");
    mavros_msgs::ParamGet param;
    param.request.param_id = param_id;
    if (getParamService.call(param)) {
        ros::spinOnce();
        ROS_INFO("Got Param %s response: values are %lu %lf", param_id.c_str(),
                                                        param.response.value.integer,
                                                        param.response.value.real);
        intVal = param.response.value.integer;
        realVal= param.response.value.real;
    }
    else {
//        ROS_ERROR("Failed to get param %s info", param_id);
        return false;
    }

    return true;
}

bool WaypointControl::setParam(std::string param_id, int intVal, double realVal)
{
    ros::ServiceClient setParamService = n.serviceClient<mavros_msgs::ParamSet>("/mavros/param/set");
    mavros_msgs::ParamSet param;
    param.request.param_id = param_id; //"SYSID_MYGCS"
    param.request.value.integer = intVal;
    param.request.value.real = realVal;
    if (setParamService.call(param)) {
        ROS_INFO("param %s: set", param_id.c_str());
    } else {
        ROS_ERROR("Failed to call service %s", param_id.c_str());
        return false;
    }
    return true;
}

bool WaypointControl::setStreamRate(unsigned char stream_id, unsigned short message_rate, bool onoff)
{
    client = n.serviceClient<mavros_msgs::StreamRate>("/mavros/set_stream_rate");
    mavros_msgs::StreamRate srv;
    srv.request.stream_id = stream_id;
    srv.request.message_rate = message_rate;
    srv.request.on_off = static_cast<unsigned char>(onoff);

    if (client.call(srv)) {
        ROS_INFO("StreamRate set OK");
    } else {
        ROS_ERROR("Failed to call StreamRate service");
        return false;
    }
    return true;
}

bool WaypointControl::cmdVel(double lx, double ly, double lz, double ax, double ay, double az)
{
    pub = n.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    geometry_msgs::Twist s;

    s.linear.x = lx;
    s.linear.y = ly;
    s.linear.z = lz;
    s.angular.x = ax;
    s.angular.y = ay;
    s.angular.z = az;
    ROS_INFO("Sending twist messages linear: %f %f %f angular: %f %f %f ", lx,ly,lz,ax,ay,az);

    pub.publish(s);

    return true;
}

bool WaypointControl::manualControl(float x, float y, float z, float r, unsigned short buttons)
{
    pub = n.advertise<mavros_msgs::ManualControl>("/mavros/manual_control/send",10);
    mavros_msgs::ManualControl mc;
    std_msgs::Header h;
//    seq++;
//    h.seq=seq;
    mc.header.stamp = ros::Time::now();
    mc.header.frame_id=1;
    mc.x = x;
    mc.y = y;
    mc.z = z;
    mc.r = r;
    mc.buttons=buttons;
    ROS_INFO("Sending manual control messages: x=%f y=%f x=%f r=%f buttons=%hu ", x,y,z,r,buttons);

    pub.publish(mc);

    return true;
}


bool WaypointControl::thrust(float t)
{
    pub = n.advertise<mavros_msgs::Thrust>("/mavros/setpoint_attitude/thrust",10);
    mavros_msgs::Thrust tm;

    // fill header
    tm.header.stamp = ros::Time::now();
    tm.header.frame_id = 1;

    //fill thrust vale 0~1
    tm.thrust = t;
    ROS_INFO("Sending thrust=%f ", t);
    pub.publish(tm);

    return true;
}

bool WaypointControl::cmd_vel(double lx, double ly, double lz, double ax, double ay, double az)
{
    pub = n.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_attitude/cmd_vel",10);
    geometry_msgs::TwistStamped tm;

    // fill header
    tm.header.stamp = ros::Time::now();
    tm.header.frame_id = 1;

    tm.twist.linear.x = lx;
    tm.twist.linear.y = ly;
    tm.twist.linear.z = lz;
    tm.twist.angular.x = ax;
    tm.twist.angular.y = ay;
    tm.twist.angular.z = az;
    ROS_INFO("Sending twist messages linear: %f %f %f angular: %f %f %f ", lx,ly,lz,ax,ay,az);

    pub.publish(tm);

    return true;
}

bool WaypointControl::deployVessel(float latitude, float longitude, float altitude)
{
    // Clear waypoints
    client = n.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
    mavros_msgs::CommandTOL tolService;

    tolService.request.latitude = latitude;
    tolService.request.longitude = longitude;
    tolService.request.altitude = altitude;
    tolService.request.yaw = std::numeric_limits<double>::quiet_NaN();
    tolService.request.min_pitch = 0;

    if (client.call(tolService))
    {
        if(tolService.response.success)
            ROS_INFO("Sent takeoff");
        else
            ROS_ERROR("Takeoff was called, but returned false");
    }
    else
    {
        ROS_ERROR("Failed to call takeoff service");
        return false;
    }

    return tolService.response.success;
}

bool WaypointControl::collectVessel(float latitude, float longitude, float altitude)
{
    // Clear waypoints
    client = n.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    mavros_msgs::CommandTOL tolService;

    tolService.request.latitude = latitude;
    tolService.request.longitude = longitude;
    tolService.request.altitude = altitude;
    tolService.request.yaw = std::numeric_limits<double>::quiet_NaN();
    tolService.request.min_pitch = 0;

    if (client.call(tolService))
    {
        if(tolService.response.success)
            ROS_INFO("Sent land");
        else
            ROS_ERROR("Land was called, but returned false");
    }
    else
    {
        ROS_ERROR("Failed to call land service");
        return false;
    }

    return tolService.response.success;
}

bool WaypointControl::arming(bool setArmed)
{
    // arm/disarm boat
    client = n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    mavros_msgs::CommandBool armService;

    armService.request.value = setArmed;


    if (client.call(armService))
    {
        if(armService.response.success)
            ROS_INFO("Commanded USV to arm");
        else
            ROS_ERROR("Arming was called, but returned false");
    }
    else
    {
        ROS_ERROR("Failed to call arming service");
        return false;
    }
    return armService.response.success;
}

bool WaypointControl::startEngine(float start, float coldStart)
{
    // start engine
    client = n.serviceClient<mavros_msgs::CommandInt>("/mavros/cmd/command_int");
    mavros_msgs::CommandInt startService;

    startService.request.frame = 1;
    startService.request.command = 223;
    startService.request.param1 = start;
    startService.request.param2 = coldStart;
    startService.request.param3 = 0;
    startService.request.param4 = 0;

    if (client.call(startService))
    {
        if(startService.response.success)
            ROS_INFO("Commanded USV to start engine");
        else
            ROS_ERROR("Engine start was called, but returned false");
    }
    else
    {
        ROS_ERROR("Failed to call start engine service");
        return false;
    }
    return startService.response.success;
}

bool WaypointControl::enableFence(float fenceEnable)
{
    // geo fence
    //  enable? (0=disable, 1=enable, 2=disable_floor_only)
    client = n.serviceClient<mavros_msgs::CommandInt>("/mavros/cmd/command_int");
    mavros_msgs::CommandInt fenceService;

    fenceService.request.frame = 1;
    fenceService.request.command = 207;
    fenceService.request.param1 = fenceEnable;

    if (client.call(fenceService))
    {
        if(fenceService.response.success)
            ROS_INFO("Commanded USV to toogle fence service");
        else
            ROS_ERROR("Engine start was called, but returned false");
    }
    else
    {
        ROS_ERROR("Failed to call fence service");
        return false;
    }
    return fenceService.response.success;
}

bool WaypointControl::setGuidedMode(bool val)
{
    client = n.serviceClient<mavros_msgs::CommandInt>("/mavros/cmd/command");
    mavros_msgs::CommandInt guidedService;

    guidedService.request.frame = 1;
    guidedService.request.command = 92;
    if(val)
        guidedService.request.param1 = 1.0; // > 0.5 enable
    else
        guidedService.request.param1 = 0.0; // <= 0.5 disable


    if (client.call(guidedService))
    {
        if(guidedService.response.success)
            ROS_INFO("Commanded USV to toogle GUIDED MODE");
        else
            ROS_ERROR("GUIDED MODE set service was called, but returned false");
    }
    else
    {
        ROS_ERROR("Failed to call GUIDED MODE set service");
        return false;
    }
    return guidedService.response.success;
}


bool WaypointControl::setYawSpeed(float angleDeg, float normalized_yaw_speed=0.5)
{
    // MAV_CMD_NAV_SET_YAW_SPEED: Sets a desired vehicle turn angle and speed change
    client = n.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");
    mavros_msgs::CommandLong yawService;
    yawService.request.confirmation = true;
    yawService.request.command = 213; // MAV_CMD_NAV_SET_YAW_SPEED
    yawService.request.param1 = angleDeg; //yaw angle to adjust steering by in centidegress 3000
    // set speed
    if(normalized_yaw_speed < 0.0 || normalized_yaw_speed >1.0)
        yawService.request.param2 = 0.5; // > 0.5 enable
    else
        yawService.request.param2 = normalized_yaw_speed; // > 0.5 enable


    if (client.call(yawService))
    {
        if(yawService.response.success)
            ROS_INFO("Commanded USV to yaw");
        else
            ROS_ERROR("Yaw service was called, but returned false");
    }
    else
    {
        ROS_ERROR("Failed to call yaw service");
        return false;
    }
    return yawService.response.success;
}
