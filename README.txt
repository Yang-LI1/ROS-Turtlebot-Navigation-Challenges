The project package there are the following nodes:
-Mybot_teleop.py
-arrene.py
-Corridor.py
-Line_following.py
-Obstacle.py
-Red_line_detector.py
-Robot_controller.py

These nodes are all in the 'scripts' file of the package.

To run the program at once (ie the 3 connected challenges) the launchfile is required: challenge.launch
This launchfile launches all nodes (apart from the teleoperation node)
It will take a moment before the robot starts to move.

Note that the program can sometimes give bad results:
It happens that the robot does not correctly make all the way to the finish.
In this case please restart the program.

To run each challenge independently you can do the following:
Challenge 1: launch  challenge1.launch
Challenge 1b (avoiding obstacles): launch challenge1b.launch
Challenge 2: launch  challenge2.launch
Challenge 3: launch  challenge3.launch

Given that sometimes the results are not very correct we give you attached a drive link to two videos of the results obtained for the same codes on two different computers.
We also note that the execution times are not at all the same.
https://drive.google.com/drive/folders/174tjcmNm5-rXcAyDNE76QKAlH98nswd6?usp=sharing

Another link in case the other doesn't work
https://filetransfer.io/data-package/BeUTQyxV#link
