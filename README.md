# common_bebop_application

## References:
1. [Quick Start Guide: ROS Kinetic](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)
2. [Github: AutonomyLab/bebop_autonomy](https://github.com/AutonomyLab/bebop_autonomy)
3. [Github: AutonomyLab/parrot_arsdk](https://github.com/AutonomyLab/parrot_arsdk)
4. [Issue when running the command: rosdep install --from-paths src -i](https://github.com/gsilano/BebopS/issues/3)
5. [Install python and pip on Ubuntu 16.04 (Xenial)](https://medium.com/@vishal.sharma./pinstall-python-and-pip-on-ubuntu-16-04-xenial-9bd11704b577)
6. [bebop_autonomy - ROS Driver for Parrot Bebop Drone (quadrocopter) 1.0 & 2.0](https://bebop-autonomy.readthedocs.io/en/latest/index.html)
7. [apriltag 0.0.16](https://pypi.org/project/apriltag/)
8. [Basic Syntax: The Markdown elements outlined in the original design document](https://www.markdownguide.org/basic-syntax/)

## Pre-eliminary Steps:
1. Edit **.bashrc** file
	1. Open to edit bashrc file:
		>  gedit ~/.bashrc
	2. Add this line to end of file:
		>
		> alias eb='gedit ~/.bashrc'
		>
		> alias sb='source ~/.bashrc'
		> 
		> alias gs='git status'
		> 
		> alias gp='git pull'
		> 
		> alias cw='cd ~/catkin_ws'
		> 
		> alias cs='cd ~/catkin_ws/src'
		> 
		> alias cm='cd ~/catkin_ws && rosdep install -y --from-paths src --ignore-src --rosdistro kinetic && catkin_make && rospack profile'
		> 
		> source /opt/ros/kinetic/setup.bash
		> 
		> source ~/catkin_ws/devel/setup.bash
		> 
		> export ROS_MASTER_URI=http://192.168.42.45:11311
		> 
		> export ROS_HOSTNAME=192.168.42.45
		> 
		> export EDITOR='gedit -w'
		> 
		> alias gumn='gp && git add . && git commit -m "latest" && git push origin main'
		> 
		>
	3. source ~/.bashrc

## Steps:
```
$ cd ~/catkin_ws/src **or** cs
$ git clone https://github.com/KhairulIzwan/common_bebop_application.git
$ git clone https://github.com/AutonomyLab/parrot_arsdk.git
$ git clone https://github.com/AutonomyLab/bebop_autonomy.git
$ catkin_make **or** cm
```

## Additional Steps: 
1. Install pip:
	```
	$ sudo apt-get install python-pip
	```
2. Install apriltag:
	```
	$ python -m pip install apriltag
	```
3. Install imutils:
	```
	$ python -m pip install imutils
	```
