#pragma once

// ros
#include <ros/ros.h>
#include "caspr_rviz/Visualization.hpp"
using namespace std;

int rviz_rate = 60;             // Default rviz rate
Visualization *visual_ptr;      // Pointer for visualization class
string robot_name;              // String holding robot's name
ros::NodeHandlePtr nh;          // Ros node pointer
bool deleteall = false;         // Boolean for whether deleting all markers 
