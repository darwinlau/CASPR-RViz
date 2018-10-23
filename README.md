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

## Using CASPR-RViz:
### RViz-side:
1. Add environment variables
```
source devel/setup.bash
```
2. Launch **CASPR-RViz** with:
``` 
roslaunch caspr_rviz caspr_rviz.launch
```
- RViz should pop up.

3. Enable visualization of meshes and transformations
- Add `Marker` and `TF` by pressing the `Add` button of the RViz `Display` panel

4. `Save Config` for convenience

### CASPR-side:
1. Configure CASPR-RViz interface in CASPR
- Note the IP of the platform that you installed CASPR by `ifconfig` (Linux) or `ipconfig` (Windows)
- Call the following function to set the config of CASPR-RViz interface in CASPR:
```
CASPRRViz_configuration.SetROSConfig('http://[caspr-rviz IP]:11311','[caspr IP]');
```
where `[caspr-rviz IP]` refers to the IP of your CASPR-RViz platform, and `[caspr IP]` refers to the IP of your CASPR platform

2. Launch CASPR GUI for tests
``` 
CASPR_GUI
```

3. Select `SpiderBot UR3` in the `Model` drop down menu 

4. Make sure the **CASPR-RViz** side is running and Press the `To RViz` button

5. Verify the visualization on RViz 

6. **ENJOY!**


