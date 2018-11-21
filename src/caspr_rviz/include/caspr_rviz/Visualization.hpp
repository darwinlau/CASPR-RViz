#pragma once

// ros
#include <mutex>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Vector3.h>

using namespace std;
using namespace chrono;

class Visualization{
public:
    Visualization();

    // Functions for publishing model from CASPR-MATLAB
    void publishLink();
    void publishCable();
    void publishEndEffector();
    void publishForce();
    void deleteAllMarkers();

    // Callback functions for subscribing from CASPR-MATLAB
    void Link_name(const std_msgs::String::ConstPtr &msg);
    void Link_tf(const std_msgs::Float32MultiArray::ConstPtr &msg);
    void Cable(const std_msgs::Float32MultiArray::ConstPtr &msg);
    void EndEffector(const std_msgs::Float32MultiArray::ConstPtr &msg);
    void Force(const std_msgs::Float32MultiArray::ConstPtr &msg);

protected:
    ros::NodeHandlePtr nh;
    uint message_counter;
    // Publisher for rviz markers
    ros::Publisher marker_visualization_pub;
    // Subscriber for CASPR-MATLAB
    ros::Subscriber link_name_sub, link_tf_sub, cable_sub, endEffector_sub, force_sub;

    // tf broadcaster
    tf::TransformBroadcaster tf_broadcaster;
    int ID;

    void publishForceArrows();
    void publishForceList();

private:
    mutex mux;

    int n_link;                           // Number of links to be visualized
    vector<string> link_names;            // Store link names
    vector<tf::Vector3> link_tf_pos;      // Store positions of links
    vector<tf::Quaternion> link_tf_rot;   // Store orientations of links
    vector<tf::Vector3> cable_start;      // Store starting points of cable segments
    vector<tf::Vector3> cable_end;        // Store ending points of cable segments
    vector<tf::Vector3> ee_pos;           // Store end-effector points
    vector<double> cable_force;           // Store cable forces
    // Parameters
    int max_ee_size = 300;                // Default max ee size
    double cable_scale = 0.005;           // Default scale of cables
    vector<double> cable_color;           // Storing cable colors
    double link_scale = 0.001;            // Default scale of links
    vector<double> link_color;            // Storing link colors
    double force_scale = 0.01;            // Default scale of force magnitude
    vector<double> force_arrow_scale;     // Storing scale of force arrows

};
