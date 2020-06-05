# Ind-bot
An implementation of a custom navigation stack.

This is a project done in partial fulfilment of the course, Robotics: Automation and Control

## Installation
1. Install ros-melodic

2. Install python-catkin-tools
```
    sudo apt-get install python-catkin-tools
```
3. Clone the repository
```
    # Clone the repo
    cd catkin_ws/src
    git clone https://github.com/SuhrudhSarathy/indbot.git

    #Build the workspace
    cd ..
    catkin build
    source devel/setup.bash
```
## Usage
1. Open turtlebot empty world using
```
    roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
```
2. Open rviz using
```
    roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch
```
3. Run the following launch file
```
    roslaunch indbot_controls demo.launch
```
4. To give a goal to the robot use rviz 2D Nav goal
