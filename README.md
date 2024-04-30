# Rosbot 2 Pro - Line Follower + Object Detection Project

Demo Videos:






## Instructions to run

ROS Version: Noetic

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


### Step 3: Run line_follower locally in the robot
```bash
cd catkin_ws
cd src
git clone https://github.com/dsanmart/rosbot-2-pro.git
cd ..
catkin_make --only-pkg-with-deps rosbot_2_pro
source devel/setup.bash
roslaunch rosbot_2_pro vision.launch
```

### Step 4: Run line_follower locally in the robot

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