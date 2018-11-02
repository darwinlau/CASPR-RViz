#include "caspr_rviz/Visualization.hpp"

Visualization::Visualization(){
    nh = ros::NodeHandlePtr(new ros::NodeHandle);

    marker_visualization_pub = nh->advertise<visualization_msgs::Marker>("visualization_marker", 100);

    // Subscribers for CASPR model info
    link_name_sub = nh->subscribe("/link_name", 100, &Visualization::Link_name, this);
    link_tf_sub = nh->subscribe("/link_tf", 100, &Visualization::Link_tf, this);
    cable_sub = nh->subscribe("/cable", 100, &Visualization::Cable, this);
    endEffector_sub = nh->subscribe("/ee", 100, &Visualization::EndEffector, this);
    force_sub = nh->subscribe("/force", 100, &Visualization::Force, this);
}

// Function for visualizing links got from CASPR-MATLAB
// - Uses info in link_names, link_tf_pos and link_tf_rot
void Visualization::publishLink(){
    // Find robot namespace in rosparam
    string robot_name;
    if (nh->hasParam("robot_name")){
        nh->getParam("robot_name", robot_name);
    }
    else
        ROS_ERROR("Robot namespace not defined!");
    // Use param link alpha blending if specified
    if (nh->hasParam("link_alpha")){
        nh->getParam("link_alpha", link_alpha);
    }
    // Use param link alpha blending if specified
    if (nh->hasParam("link_scale")){
        nh->getParam("link_scale", link_scale);
    }

    // Ensure name, pos, rot are all the same length
    n_link = link_names.size();
    bool linkAvailable = false;
    if (link_tf_pos.size() == n_link && link_tf_rot.size() == n_link)
        linkAvailable = true;

    // If links are available, render those meshes
    if (linkAvailable){
        uint id = 20000;
        for (uint i = 0; i < n_link; i++) {
            auto start_tic = std::chrono::system_clock::now();
            // Get corresponding info of this link
            tf::Vector3 origin = link_tf_pos[i];
            tf::Quaternion quat = link_tf_rot[i];
            string name = link_names[i];

            // Create marker for the mesh
            visualization_msgs::Marker mesh;
            mesh.header.frame_id = "world";
            mesh.ns = name.c_str();
            mesh.type = visualization_msgs::Marker::MESH_RESOURCE;
            mesh.color.r = 0.8f;
            mesh.color.g = 0.8f;
            mesh.color.b = 0.8f;
            mesh.color.a = link_alpha;
            mesh.scale.x = link_scale;
            mesh.scale.y = link_scale;
            mesh.scale.z = link_scale;
            mesh.lifetime = ros::Duration(0);
            mesh.header.stamp = ros::Time::now();
            mesh.action = visualization_msgs::Marker::ADD;
            mesh.id = id++;

            // Define the pos and orientation of the mesh
            mesh.pose.position.x = origin.x();
            mesh.pose.position.y = origin.y();
            mesh.pose.position.z = origin.z();
            mesh.pose.orientation.x = quat.x();
            mesh.pose.orientation.y = quat.y();
            mesh.pose.orientation.z = quat.z();
            mesh.pose.orientation.w = quat.w();

            // Get the STL model and publish
            char meshpath[200];
            sprintf(meshpath, "package://robot_library/%s/%s.stl",
                    robot_name.c_str(), name.c_str());
            mesh.mesh_resource = meshpath;
            marker_visualization_pub.publish(mesh);

            // Pass info to tf
            tf::Transform trans;
            trans.setRotation(tf::Quaternion(quat.x(), quat.y(), quat.z(), quat.w()));
            trans.setOrigin(tf::Vector3(origin.x(), origin.y(), origin.z()));
            tf_broadcaster.sendTransform(tf::StampedTransform(trans, ros::Time::now(), "world", name.c_str()));

            // Wasting some time to prevent missing publish
            auto end_tic = std::chrono::system_clock::now();
            for (uint j = 0; j < 100; j++)
                end_tic = std::chrono::system_clock::now();
            std::chrono::duration<double> diff = end_tic - start_tic;
        }
    }
}

// Function for visualizing cables got from CASPR-MATLAB
// - Uses info in cable_start and cable_end vectors
void Visualization::publishCable() {
    // Use param cable scale if specified
    if (nh->hasParam("cable_scale")){
        nh->getParam("cable_scale", cable_scale);
    }

    // Check if the two vectors contain any info and their size match
    if (cable_start.size() != 0 && cable_start.size() == cable_end.size()){
        // Create line list marker
        visualization_msgs::Marker line_list;
        line_list.header.frame_id = "world";
        line_list.header.stamp = ros::Time::now();

        // Define properties of the line list
        line_list.ns = "cables";
        line_list.action = visualization_msgs::Marker::ADD;
        line_list.type = visualization_msgs::Marker::LINE_LIST;
        line_list.scale.x = cable_scale;
        line_list.color.r = 1.0;
        line_list.color.a = 0.9;
        line_list.pose.orientation.w = 1.0;
        line_list.lifetime = ros::Duration(0);
        line_list.id = 30000;
        // Load the start and end points into the line list
        for (uint i = 0; i < cable_start.size(); i++) {
            geometry_msgs::Point p;
            p.x = cable_start[i].x();
            p.y = cable_start[i].y();
            p.z = cable_start[i].z();
            line_list.points.push_back(p);
            p.x = cable_end[i].x();
            p.y = cable_end[i].y();
            p.z = cable_end[i].z();
            line_list.points.push_back(p);
        }
        marker_visualization_pub.publish(line_list);
    }
}

void Visualization::publishForce(){
    // Default max no of cables that allow arrow rendering is 50
    uint max_num_cable_for_arrows = 50;
    // Count the expected num of arrows to be rendered
    uint expected_num_arrows = 0;
    for (auto f:cable_force)
        if (f >= 0)
            expected_num_arrows++;
    // If too much arrows, render line lists instead
    if (expected_num_arrows > max_num_cable_for_arrows)
        publishForceList();
    else
        publishForceArrows();
}

// Function for visualizing cable forces as arrows
// - Only work for robots with small number of cable
// - Otherwise will cause serious performance issues
// - Uses info in cable_force, cable_start, cable_end
void Visualization::publishForceArrows() {
    // Use param force scale if specified
    if (nh->hasParam("force_scale")){
        nh->getParam("force_scale", force_scale);
    }

    // Check if the cable_force
    if (cable_force.size() != 0 && cable_start.size() == cable_force.size()){
        // Create arrow marker
        visualization_msgs::Marker arrow;
        arrow.header.frame_id = "world";
        arrow.header.stamp = ros::Time::now();
        arrow.ns = "cable_forces";
        arrow.type = visualization_msgs::Marker::ARROW;
        arrow.color.a = 1.0;
        arrow.lifetime = ros::Duration(1);
        // Use param force scale if specified
        if (nh->hasParam("force_arrow_scale")){
            nh->getParam("force_arrow_scale", force_arrow_scale);
            if (force_arrow_scale.size() != 3)
                ROS_INFO_STREAM("Wrong rosparam for force arrow scale!");
        }
        else{
            // Default arrow scale
            force_arrow_scale.push_back(3);
            force_arrow_scale.push_back(6);
            force_arrow_scale.push_back(6);
        }
        arrow.scale.x = force_arrow_scale[0]*cable_scale;
        arrow.scale.y = force_arrow_scale[1]*cable_scale;
        arrow.scale.z = force_arrow_scale[2]*cable_scale;
        arrow.pose.orientation.w = 1;
        arrow.pose.orientation.x = 0;
        arrow.pose.orientation.y = 0;
        arrow.pose.orientation.z = 0;

        uint id = 40000;
        for (uint i = 0; i < cable_force.size(); i++){
            // Check start and end of cable
            bool isCableStart = true;
            if (i != 0){
                // Check if next cable force is negative -> not end of cable
                if (cable_force[i-1] < 0){
                    isCableStart = false;
                }
            }
            bool isCableEnd = true;
            if (i != cable_start.size() - 1){
                // Check if next cable force is negative -> not end of cable
                if (cable_force[i+1] < 0){
                    isCableEnd = false;
                }
            }
            if (cable_force[i] < 0){
                isCableStart = false;
                isCableEnd = false;
            }
            // Unit direction
            tf::Vector3 direction = cable_end[i] - cable_start[i];
            direction.normalize();
            if (isCableStart){
                // 1st arrow
                arrow.id = id + 2*i;
                arrow.color.r = 1.0f;
                arrow.color.g = 1.0f;
                arrow.color.b = 0.0f;
                arrow.header.stamp = ros::Time::now();
                arrow.points.clear();
                geometry_msgs::Point p;
                // start_1
                p.x = cable_start[i].x();
                p.y = cable_start[i].y();
                p.z = cable_start[i].z();
                arrow.points.push_back(p);
                // end_1
                p.x += direction.x()*force_scale*cable_force[i];
                p.y += direction.y()*force_scale*cable_force[i];
                p.z += direction.z()*force_scale*cable_force[i];
                arrow.points.push_back(p);
                marker_visualization_pub.publish(arrow);
            }

            if (isCableEnd){
                // 2nd arrow
                arrow.id = id + 2*i + 1;
                arrow.color.r = 0.0f;
                arrow.color.g = 1.0f;
                arrow.color.b = 0.0f;
                arrow.header.stamp = ros::Time::now();
                arrow.points.clear();
                geometry_msgs::Point p;
                // start_1
                p.x = cable_end[i].x();
                p.y = cable_end[i].y();
                p.z = cable_end[i].z();
                arrow.points.push_back(p);
                // end_1
                p.x -= direction.x()*force_scale*cable_force[i];
                p.y -= direction.y()*force_scale*cable_force[i];
                p.z -= direction.z()*force_scale*cable_force[i];
                arrow.points.push_back(p);
                marker_visualization_pub.publish(arrow);
            }
        }
    }
}

// Function for visualizing cable forces as line_list
// - Uses info in cable_force, cable_start, cable_end
void Visualization::publishForceList() {
    // Use param force scale if specified
    if (nh->hasParam("force_scale")){
        nh->getParam("force_scale", force_scale);
    }

    // Check if the two vectors contain any info and their size match
    if (cable_force.size() != 0 && cable_force.size() == cable_start.size()){
        // Create line list marker
        visualization_msgs::Marker line_list_action;
        line_list_action.header.frame_id = "world";
        line_list_action.header.stamp = ros::Time::now();

        // Define properties of the line list
        line_list_action.ns = "cable_force_actions";
        line_list_action.action = visualization_msgs::Marker::ADD;
        line_list_action.type = visualization_msgs::Marker::LINE_LIST;
        line_list_action.scale.x = 2*cable_scale;
        line_list_action.color.g = 1.0;
        line_list_action.color.a = 1.0;
        line_list_action.pose.orientation.w = 0.9;
        line_list_action.lifetime = ros::Duration(0);
        line_list_action.id = 40000;

        // Create line list marker
        visualization_msgs::Marker line_list_reaction;
        line_list_reaction.header.frame_id = "world";
        line_list_reaction.header.stamp = ros::Time::now();

        // Define properties of the line list
        line_list_reaction.ns = "cable_force_reactions";
        line_list_reaction.action = visualization_msgs::Marker::ADD;
        line_list_reaction.type = visualization_msgs::Marker::LINE_LIST;
        line_list_reaction.scale.x = 2*cable_scale;
        line_list_reaction.color.r = 1.0;
        line_list_reaction.color.g = 1.0;
        line_list_reaction.color.a = 1.0;
        line_list_reaction.pose.orientation.w = 0.9;
        line_list_reaction.lifetime = ros::Duration(0);
        line_list_reaction.id = 50001;

        // Load the start and end points into the line list
        for (uint i = 0; i < cable_start.size(); i++) {
            // Check start and end of cable
            bool isCableStart = true;
            if (i != 0){
                // Check if next cable force is negative -> not end of cable
                if (cable_force[i-1] < 0){
                    isCableStart = false;
                }
            }
            bool isCableEnd = true;
            if (i != cable_start.size() - 1){
                // Check if next cable force is negative -> not end of cable
                if (cable_force[i+1] < 0){
                    isCableEnd = false;
                }
            }
            if (cable_force[i] < 0){
                isCableStart = false;
                isCableEnd = false;
            }
            // Unit direction
            tf::Vector3 direction = cable_end[i] - cable_start[i];
            direction.normalize();
            // action
            geometry_msgs::Point p;
            // Only draw action arrow if is start of cable
            if (isCableStart){
                if (cable_force[i] < 0)
                    ROS_INFO_STREAM("Printing negative start force");
                // start_action
                p.x = cable_start[i].x();
                p.y = cable_start[i].y();
                p.z = cable_start[i].z();
                line_list_action.points.push_back(p);
                // end_action
                p.x += direction.x()*force_scale*cable_force[i];
                p.y += direction.y()*force_scale*cable_force[i];
                p.z += direction.z()*force_scale*cable_force[i];
                line_list_action.points.push_back(p);
            }

            // Only draw reaction arrow if is end of cable
            if (isCableEnd){
                if (cable_force[i] < 0)
                    ROS_INFO_STREAM("Printing negative end force");
                // reaction
                // start_reaction
                p.x = cable_end[i].x();
                p.y = cable_end[i].y();
                p.z = cable_end[i].z();
                line_list_reaction.points.push_back(p);
                // end_reaction
                p.x -= direction.x()*force_scale*cable_force[i];
                p.y -= direction.y()*force_scale*cable_force[i];
                p.z -= direction.z()*force_scale*cable_force[i];
                line_list_reaction.points.push_back(p);
            }
        }
        marker_visualization_pub.publish(line_list_action);
        marker_visualization_pub.publish(line_list_reaction);
    }
}

// Function for visualizing end-effector got from CASPR-MATLAB
// - Uses info in ee_pos
void Visualization::publishEndEffector() {
    // Use param max ee size if specified
    if (nh->hasParam("max_ee_size")){
        nh->getParam("max_ee_size", max_ee_size);
    }

    // Use default max ee size
    if (ee_pos.size() > max_ee_size){
        for (uint i = 0; i < ee_pos.size() - max_ee_size; i++){
            ee_pos.erase(ee_pos.begin());
        }
    }

    if (ee_pos.size() != 0){
        ROS_INFO_STREAM("creating ee!");
        // Create a series of spheres
        visualization_msgs::Marker sphere;
        sphere.header.frame_id = "world";
        char eenamespace[20];
        sprintf(eenamespace, "ee");
        sphere.ns = eenamespace;
        sphere.type = visualization_msgs::Marker::SPHERE_LIST;
        sphere.color.r = 0.0;
        sphere.color.g = 1.0;
        sphere.color.b = 1.0;
        sphere.color.a = 1.0;
        sphere.lifetime = ros::Duration(0);
        // Radius temporarily defined here, although not too convenient
        double radius = 0.01;
        sphere.scale.x = radius;
        sphere.scale.y = radius;
        sphere.scale.z = radius;
        sphere.action = visualization_msgs::Marker::ADD;
        sphere.header.stamp = ros::Time::now();
        sphere.id = 1000000;
        for(uint i = 0; i < ee_pos.size(); i++){
          geometry_msgs::Point p;
          p.x = ee_pos[i].x();
          p.y = ee_pos[i].y();
          p.z = ee_pos[i].z();
          sphere.points.push_back(p);
        }
        marker_visualization_pub.publish(sphere);
    }
}

// Function to delete all markers
void Visualization::deleteAllMarkers(){
    // Init vector variables
    cable_start.clear();
    cable_end.clear();
    cable_force.clear();
    link_tf_pos.clear();
    link_tf_rot.clear();
    link_names.clear();
    // Delete all markers
    visualization_msgs::Marker marker;
    marker.action = visualization_msgs::Marker::DELETEALL;
    for (int i = 0; i < 100; i++)
        marker_visualization_pub.publish(marker);
    nh->setParam("deleteall", false);
}

//////////////////////////////////////
// Callback functions for Subscriber
//////////////////////////////////////

// Get the names of the links that are supposed to be visualized
// Such that the corresponding STL files can be loaded
void Visualization::Link_name(const std_msgs::String::ConstPtr &msg){
    lock_guard<mutex> lock(mux);
    link_names.clear();
    string msg_string = msg->data;
    string delim = ",;/";
    auto start = 0U;
    auto end = msg_string.find(delim);
    while (end != string::npos)
    {
        link_names.push_back(msg_string.substr(start, end - start));
        start = end + delim.length();
        end = msg_string.find(delim, start);
    }
}

// Get the transforms of the links (Pos: Vector3; Orientation: Quaternion)
// The array comes in the form of 1 x 7n array, where n refers to no. of links
// 7 comes from 3d Pos and 4d Quaternion
void Visualization::Link_tf(const std_msgs::Float32MultiArray::ConstPtr &msg){
    lock_guard<mutex> lock(mux);
    link_tf_pos.clear();
    link_tf_rot.clear();
    for (uint i = 0; i < msg->data.size() / 7; i++) {
        tf::Vector3 pos = tf::Vector3(msg->data[i*7],msg->data[i*7+1],msg->data[i*7+2]);
        link_tf_pos.push_back(pos);
        tf::Quaternion quat = tf::Quaternion(msg->data[i*7+3],msg->data[i*7+4],msg->data[i*7+5],msg->data[i*7+6]);
        link_tf_rot.push_back(quat);
    }
}

// Get an array of start and end points
// Vector form: [start_1, end_1, start_2, end_2, ...]
void Visualization::Cable(const std_msgs::Float32MultiArray::ConstPtr &msg){
    lock_guard<mutex> lock(mux);
    cable_start.clear();
    cable_end.clear();
    for (uint i = 0; i < msg->data.size() / 6; i++) {
        tf::Vector3 start_point = tf::Vector3(msg->data[i*6],msg->data[i*6+1],msg->data[i*6+2]);
        cable_start.push_back(start_point);
        tf::Vector3 end_point = tf::Vector3(msg->data[i*6+3],msg->data[i*6+4],msg->data[i*6+5]);
        cable_end.push_back(end_point);
    }
}

// Get an array of end-effector points (in case there are multiple end-effectors)
// Vector form: [ee_1, ee_2, ...]
void Visualization::EndEffector(const std_msgs::Float32MultiArray::ConstPtr &msg){
    lock_guard<mutex> lock(mux);
    // ee_pos.clear();
    for (uint i = 0; i < msg->data.size() / 3; i++) {
        tf::Vector3 this_ee_pos = tf::Vector3(msg->data[i*6],msg->data[i*6+1],msg->data[i*6+2]);
        ee_pos.push_back(this_ee_pos);
    }
}

// Get an array of cable forces
// Vector form: [f_1, f_2, ...]
void Visualization::Force(const std_msgs::Float32MultiArray::ConstPtr &msg){
    lock_guard<mutex> lock(mux);
    cable_force.clear();
    for (auto f:msg->data) {
        cable_force.push_back(f);
    }
}
