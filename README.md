# ROS-Turtlebot-Navigation-Challenges

## Project Overview
This repository hosts the code for a ROS-based project aimed at controlling a Turtlebot 3 Burger in various complex environments. The project involves solving three key challenges: line following, corridor navigation, and navigation in a cluttered environment, leveraging both simulated and real-world scenarios.

## Installation

### Prerequisites
- ROS Noetic
- Turtlebot3 Packages
- OpenCV for ROS

### Setup
Clone the repository and install necessary ROS packages:
```bash
cd ~/catkin_ws/src
git clone https://github.com/Yang-LI1/ROS-Turtlebot-Navigation-Challenges.git
sudo apt-get install ros-noetic-turtlebot3-*
sudo apt-get install ros-noetic-cv-bridge ros-noetic-image-transport
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Usage
###Running the Simulation
Launch the simulation environment for each challenge using the provided launch files:

# Challenge 1: Line Following
roslaunch ROS-Turtlebot-Navigation-Challenges challenge1.launch

# Challenge 2: Corridor Navigation
roslaunch ROS-Turtlebot-Navigation-Challenges challenge2.launch

# Challenge 3: Cluttered Environment Navigation
roslaunch ROS-Turtlebot-Navigation-Challenges challenge3.launch

# To run the program at once (ie the 3 connected challenges) the launchfile is required: challenge.launch


Given that sometimes the results are not very correct we give you attached a drive link to two videos of the results obtained for the same codes on two different computers.
We also note that the execution times are not at all the same.
https://drive.google.com/drive/folders/174tjcmNm5-rXcAyDNE76QKAlH98nswd6?usp=sharing

Another link in case the other doesn't work
https://filetransfer.io/data-package/BeUTQyxV#link


