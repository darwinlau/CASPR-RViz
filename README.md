# CASPR-RViz
CASPR-RViz is an RViz interface for visualizing cable-driven parallel robots (CDPRs) in [CASPR](https://github.com/darwinlau/CASPR). 

## Installing CASPR-RViz:
### Requirements:
1. **ROS**
- Install ROS on your Linux platform [(ROS Kinetic Installation)](http://wiki.ros.org/kinetic/Installation)
- Make sure your ROS installation includes **rviz** (e.g. Desktop-Full Install)
2. **CASPR**
- Install [CASPR](https://github.com/darwinlau/CASPR) on MATLAB 
3. **Robotics System Toolbox**
- Install [Robotics System Toolbox](https://www.mathworks.com/products/robotics.html) on MATLAB to use ROS on the MATLAB side

### Steps:
1. Clone **CASPR-RViz** to your Linux platform
```
git clone https://github.com/darwinlau/CASPR-RViz.git
```
2. Build **CASPR-RViz**
```
cd CASPR-RViz && catkin_make
```
3. Configure ROS
- Connect the platforms that you installed CASPR-RViz and CASPR
- Note the IP of the platform that you built CASPR-RViz by `ifconfig`
- Edit the `~/.bashrc` file by `gedit ~/.bashrc`
- Put the following lines in your `~/.bashrc` file and save it:
```
export ROS_IP=[caspr-rviz IP]
export ROS_MASTER_URI=http://$ROS_IP:11311
```
where `[caspr-rviz IP]` refers to the IP of your CASPR-RViz platform

## Setting up CASPR-RViz:
### RViz-side:
1. Add environment variables
```
source devel/setup.bash
```
2. Launch RViz with:
``` 
roslaunch caspr_rviz caspr_rviz.launch
```

3. Enable visualization of meshes and transformations
- Add `Marker` and `TF` by pressing the `Add` button of the RViz `Display` panel

4. `Save Config` for convenience

### CASPR-side:
1. Configure CASPR-RViz interface in CASPR
- Note the IP of the platform that you installed CASPR by `ifconfig` (Linux) or `ipconfig` (Windows)
- Call the following function to set the config of CASPR-RViz interface in CASPR:
```
CASPRRViz_configuration.SetROSConfig('[caspr-rviz IP]','[caspr IP]');
```
where `[caspr-rviz IP]` refers to the IP of your CASPR-RViz platform, and `[caspr IP]` refers to the IP of your CASPR platform

## Using CASPR-RViz:
1. Launch **CASPR-RViz**
```
source devel/setup.bash
roslaunch caspr_rviz caspr_rviz.launch
```

2. Launch CASPR GUI on CASPR (MATLAB)
``` 
CASPR_GUI
```

### Static visualization:
1. Select your desired model in the `Model` drop down menu of CASPR GUI

2. Press the `To RViz` button on CASPR GUI

3. Select `world` in the `Global Options > Fixed Frame` drop down menu on the RViz `Display` panel

4. Verify that the robot is successfully visualized in RViz

### Kinematics simulation:
1. Select your desired model in the `Model` drop down menu of CASPR GUI

2. Press the `Kinematics` button in `Simulators`

3. Press the `Run` button in Kinematics Simulator to run a simulation

4. After the simulation is finished, press the `RViz` button to visualize the robot's motion

### Dynamics simulation:
1. Select your desired model in the `Model` drop down menu of CASPR GUI

2. Press the `Dynamics` button in `Simulators`

3. Press the `Run` button in Dynamics Simulator to run a simulation

4. After the simulation is finished, press the `RViz` button to visualize the robot's motion and force vectors

## Customizing visualization:
### Cables:
1. Change the thickness of the cables by:
```
rosparam set /cable_scale scale
```
2. Change the color (RGBA) of the cables by: 
```
rosparam set /cable_color [R,G,B,A]
```
### Links:
1. Change the color (RGBA) of the links by:
```
rosparam set /link_color [R,G,B,A]
```
### Cable forces:
1. Change the magnitude of the forces by:
```
rosparam set /force_scale scale
```
2. Change the size and shape of the force arrows by:
```
rosparam set /force_arrow_scale [x,y,z]
```
