# Intro2ROS Project 
Group 15 "Geminis" focuses on the ROS Project "Autonomous Drones".

# Repository
git@gitlab.lrz.de:yipeng.zhou/intro2ros-project.git

https://gitlab.lrz.de/yipeng.zhou/intro2ros-project.git

# Members and Assignment Distribution
### SLAM 
Zijian Ma (03727367) <br>
Yipeng Zhou (03743111) <br>

### Navigation
Zihao Li (03749952) <br>Zhi Zheng(03735915) <br>
### State_Machine
Bohua Zou (03727823) <br>

# Running Steps

### Install external pkgs
```
sudo apt-get install ros-noetic-depth-image-proc
sudo apt-get install ros-noetic-octomap-server
sudo apt-get install ros-noetic-move-base
sudo apt-get install python3
sudo apt-get install python3-wxgtk4.0
sudo apt-get install xdot
sudo apt-get install ros-noetic-smach
```

### Download Repository, Build and Source
Our final achievement is on branch "main", please make sure you are in this branch after clone this repository.
```
cd AutonomousDrones
catkin build
source devel/setup.bash
```

Now, unzip the Unity file and copy the files to “AutonomousDrones/devel/lib/simulation/“

### Start Simulation

```
roslaunch simulation simulation.launch
```

The rviz and the smach_viewer will run automatically. Rviz is for presenting the generated map and smach_viewer shows the current state of two drones. 

The Details of our code are explained in *Document.pdf*. 

Besides, *rosgraph.png* and *frames.png* provide an overview of our project.

There is low probability that one or two drones get stuck. Please rerun the launch file.

If you have any problem when running this project, please contact us freely.

Enjoy!
