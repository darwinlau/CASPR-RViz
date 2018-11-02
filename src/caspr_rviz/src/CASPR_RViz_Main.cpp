#include "caspr_rviz/CASPR_RViz_Main.hpp"

int main(int argc, char **argv){
    ros::init(argc, argv, "CASPR_RViz_Main");

    nh = ros::NodeHandlePtr(new ros::NodeHandle);

    visual_ptr = new Visualization;

    ROS_INFO_STREAM("Defined node");
    // Robot ns
    if (nh->hasParam("robot_name")){
        nh->getParam("robot_name", robot_name);
        ROS_INFO_STREAM("CASPR-RViz visualizing: " << robot_name);
    }
    else
        ROS_ERROR("Robot namespace not defined!");

    // Define update loop rate
    if (nh->hasParam("rviz_rate"))
        nh->getParam("rviz_rate", rviz_rate);
    ros::Rate loop_rate(rviz_rate);

    // Main update loop
    while (ros::ok()){
        if (nh->hasParam("deleteall")){
            nh->getParam("deleteall", deleteall);
            if (deleteall){
                visual_ptr->deleteAllMarkers();
                ROS_INFO_STREAM("Delele all previous markers.");
            }
        }
        // Visualize robot through Visualization
        visual_ptr->publishLink();
        visual_ptr->publishCable();
        visual_ptr->publishEndEffector();
        visual_ptr->publishForce();

        nh->getParam("robot_name", robot_name);
        ROS_INFO_STREAM("CASPR-RViz visualizing: " << robot_name);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
