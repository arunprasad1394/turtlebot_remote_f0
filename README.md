# turtlebot remote

## Setup

### sourcing ROS

Add the following to your bashrc file for ros and the workspace to be sourced when opening a new terminal in dev_ws. make sure your are not sourcing ROS in your bashrc file.


```bash
if [ -f "/dev_ws/setup.bash" ]; then
    source /dev_ws/setup.bash
fi
```

## use

### build the image

Running the following command from the root of the repo will execute the build image shell script

```bash
.docker/build_image.sh
```

### run the image

Running the following command from the root of the repo will execute the run image shell script

```bash
.docker/run_user.sh
```

with Nvidia support (requires nvidia-docker2)

```bash
.docker/run_user_nvidia.sh
```
Once inside the container, ake ownership of the workspace with

```bash
sudo chown -R $USER /dev_ws
```

### Connection (Turtebot)
1. Connect to the robot
   Connect the robot wifi, change the user ip address to 10.41.1.10, mask 24
2. SSH to the robot
   open a terminal on local pc and this terminal will connect to the system in robot
   
   ```bash
   ssh iaac@10.41.1.1
   ```
3. Bringup the robot
   ```bash
   cd turtlebot_robot
   .docker/run_user.sh
   roslaunch turtlebot_bringup iaac.launch
   ```
### Remote computer(PC)
1. Build the image and run the container
   ```bash
   cd turtlebot_remote_f0
   ```
   git pull(make sure all the files are up to date)
   ```bash
   .docker/build_image.sh
   ```
   Run the image
   ```bash
   .docker/run_user.sh
   sudo chown -R $user /dev_ws
   ```
2. Gmapping(all in terminator)
   terminator(split when you want to run multiple nodes)
   1. rviz(add robotmodel, fix frame, add laser_scan from topics)
   2. launch the gmapping node
      ```bash
      roslaunch turtlebot_navigation turtlebot_gmapping.launch
      ```
   3. teleoperation
      ```bash
      roslaunch turtlebot_teleop keyboard_teleop.launch
      ```
   4. Save the map
      ```bash
      rosrun map_server map_saver /dev_ws/src/turtlebot_apps/turtlebot_navigation/maps/<NAME>
      ```
3. Navigation(same terminator)
   1. change the map name in the amcl_demo_iaac launch file
   2. rviz(load the map, robotmode, laser_scan, image)
   3. Launch the navigation node
      ```bash
      roslaunch turtlebot_navgation amcl_demo_iaac.launch
      ```
   4. Estimate the pose and position of the robot
   5. Navigate to a position.


