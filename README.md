/***********************************************************************************

This program is provided as is, use it at your own risk

The program was constructed using Qt 5.9.1. and Qt5Core, Qt5Qml, Qt5Quick, Qt5Location, 
and Qt5Positioning. The Qt node makes use of several maps based on mapviewer example from
Qt 5.9.1. Compiling the code with versions previous to Qt. 5.9.1 will fail. The package 
is designed form ROS Kinetic and was created using the catkin tool. It requires mavros, 
mavros_msgs, and mavlink packages packages.  

it with previous versions of qt is not recommended. The following Qt
modules are used: core, serialport and websockets.  

Qt can be obtained at: <https://www.qt.io/download>

Vitor A. M. Jorge

***********************************************************************************/

# Documentation of an Interface with the USVs from the LSA using Navio Board

## Goal

This catkin based package presents very simple ways to interact with the USVs, changing its waypoints, home position and controlling the motors using the mavlink/mavros protocols.  

The main goal of this program is to enable the use of the usv for autonomous navigation. At this time the code has a node for a gui based operation and several simple examples using command line parameters.  

## Compiling the Code  

The code must be cloned from the git repository into a catkin workspace, which requires mavlink and mavros packages. To install them do it as follows:  

    `$ sudo apt-get install ros-kinetic-mavlink`  
    `$ sudo apt-get install ros-kinetic-mavros`  
    `$ sudo apt-get install ros-kinetic-mavros_msgs`  
    `$ sudo apt-get install ros-kinetic-mavros_extras`  

At this time, we recommend the installation of Qt>=5.9.1 using the online Qt intalation tool, even though with minor modifications it may be possible to compile the code using conventional Ubuntu qt packages. To try to compile everything using Qt5 base libraries do as follows (__We do not recommend this course of action.__)
    `$ sudo apt-get install qtcreator`  
    `$ sudo apt-get install libqt5positioning5-plugins libqt5positioning5 libqt5location5 libqt5location5-gles libqt5location5-plugins qtlocation5-dev qtpositioning5-dev`  
    
    __Note that depending on the Ubuntu system, gles versions of such packages may be required.__  

The CMakelists.txt points the Qt installation to a default location at starting at the home directory. This is used to find the appropriate Qt libs and includes -- not the system Qt installation. If you install it in a different dir, it must be changed in the CMakelists.txt accordingly.  
    
Then we must go to the catkin_workspace and compile it with the catkin_make command.  
    `$ cd <CATKIN_ROOT_DIR>/`  
    `<CATKIN_ROOT_DIR>$ catkin_make`  
    
If Qt is not properly installed, usv_mavros_gui_node and qmake header compilation will fail likely fail. At this point, the gui_node only works with Qt, however, the other *no_gui nodes do not require Qt to work. The class which is used to communicate with ROS and mavros only makes use of std libraries and should compile just fine. Otherwise everything will compile just fine.  

## Usage

There area few nodes available, we describe the below briefly. Recall that a roscore must at the server must be running the mavros services and topics. Set the ~/.bashrc file with the IP of the server.

1. The usv_mavros_gui_node starts a GUI which enables several features to edit waypoints and send them to the USV, including: selection of several map providers and maps. Deletion and addition of waypoints, sethome function and other features.   

![alt text](https://raw.githubusercontent.com/vitoramj/SimpleRobotWebRemote/master/ScreenShot.png)  

2. Another node available is the usv_mavros_gui_nogui_node, which loads a waypoint vector from file. The format of the file is as follows:  
current_waypoint index  
latitude longitude param2 param3  
...  

	Note that param2 and param3 refer to the waypoint approximation radius. if they are zero, the robot passes through the waypoint otherwise it corresponds to an approximation radius which when reached changes to next waypoint. if param3 is negative the waypoint is approached in a counter clockwise manner, if it is positive, the approach is performed in a clockwise manner. An example is shown below:  
0  
-30.046526 -51.235233 0 0  
-30.047193 -51.234848 0 0  
-30.047563 -51.234677 0 0  
-30.048712 -51.234677 0 0  
-30.049564 -51.234677 0 0  

in order to run the node just run:  
    `$ rosrun usv_mavros_gui usv_mavros_gui_nogui_node <path/filename>`  
    Note: there is an example file in the examples directory.  

3. The usv_mavros_gui_nogui_servo is an example of how to set servos controlled by the Navio board. This node is currently under development. To run the code just do:
    `$ rosrun usv_mavros_gui_nogui_servo <unit16> <unit16> <unit16> unit16> <unit16> <unit16> unit16> <unit16>`  

	Setting parameters for each servo is made setting the values of each channel.  

## The WaypointControl Class

This class is responsible for the hardwork and can be used to set waypoints in your code as well as other mavros functions. Technically it alone should be enough to add limited mavros services and topic functionalities. We have designed a lib compiled for such purpose.  

## Limitations  

1. Gui still does not work on default install and Qt 5.5.1 libs of Ubuntu Mate or Raspian due to failure in antialiazing Qt features.  
2. Raw servo control still does not work.  
3. CMakelists.txt still does not install the package  

Enjoy  
