# Fruit Ninja - Rosbot 2 Pro Edition

The purpose of this project is to follow a white line whilst avoiding obstacles and classifying fruit objects from the [freshie fruits dataset](https://universe.roboflow.com/freshie/freshie-fruits)

<p align="center">
  <img width="60%" src="./assets/fruitninja.gif"/>
</p>

## Features
<p align="center">
  <img src="./assets/cam_view.gif" alt="animated"/>
  <img src="./assets/cam_view_2.gif" alt="animated"/>
  <img src="./assets/rviz_view.png"  width=400/>
  <img src="./assets/fruit_detection.jpeg" width=400/>
  <img src="./assets/hsv_image.jpg"  width=400/>
  <img src="./assets/box.jpg" width=400/>
</p>


# Rosbot Topics
<p align="center">
  <img src="./assets/rosbot_topics.png" width=500 center/>
</p>
* Note that this are the default topics from the rosbot 2 pro and it is missing the topics created by the execution of the package. Some of the topics created are:

  - `fruit_detector`
  - `visualisation_marker`
  - `map`

The `fruit_detector` uses a custom `Fruits.msg` to publish the detected fruit in the image. The `visualisation_marker` is used to track the path followed by the rosbot in the `rviz` map environment. The `map` topic is used to visualize the odometry of the rosbot in the `rviz` map environment.


# Instructions to run

ROS Version: Noetic - [Installation Instructions](https://wiki.ros.org/noetic/Installation/Ubuntu)

* Notice that the Computer Vision model for fruit classification is running in a remote PC for faster inference. On the other hand, the line_follower is running locally on the robot to avoid delays caused in the transmission of the camera topics to the remote PC.

To check the IP of your robot/PC run the following command:
```
ifconfig | grep inet
```

#### Dependencies:
- `gmapping` for the odometry visualization in `rviz`
```bash
sudo apt install ros-noetic-slam-gmapping
```
- `ultralytics` for the yolo prediction model
```bash
pip install ultralytics
```

### Step 1: Connect remote computer to run object detection model:

```bash
export ROS_MASTER_URI=http://{ROSBOT_IP}:11311
export ROS_IP={LOCAL_BROADCASTED_IP}
```

### Step 2: Set Rosbot's env variables to broadcasted IP:

```bash
export ROS_MASTER_URI=http://{ROSBOT_IP}:11311
export ROS_IP={ROSBOT_IP}
```

*** When the env variables are localhost the robot won't stream ROS and it will be a loopback address


### Step 3: Run `line_follower` locally in the robot
```bash
cd catkin_ws
cd src
git clone https://github.com/dsanmart/rosbot-2-pro.git
cd ..
catkin_make --only-pkg-with-deps rosbot_2_pro
source devel/setup.bash
roslaunch rosbot_2_pro vision.launch
```

### Step 4: Run `vision` in the PC with enough space for the CV model

```bash
ssh username@{ROSBOT_IP}
cd catkin_ws
cd src
git clone https://github.com/dsanmart/rosbot-2-pro.git
cd ..
catkin_make --only-pkg-with-deps rosbot_2_pro
source devel/setup.bash
roslaunch rosbot_2_pro line_follower.launch
```
