# Pick and Place with TIAGO Robot in ROS2

Team project for Intelligent Robotics course in University of Padua, 2023


Team members: **Mohamed Abdelwahab**, **Victor Miguel**, **Onur Akman**

## Docs
* **[Instructions](https://github.com/aonurakman/ROS-Pick-n-Place/blob/b44f005c84018cad49ff78600be390d785f54468/Instructions.pdf)**: Instructions for this project, tasks and additional requirements.
* **[Report](https://github.com/aonurakman/ROS-Pick-n-Place/blob/b44f005c84018cad49ff78600be390d785f54468/Report.pdf)**: Our final report, our approach and reasoning.

![Picking](https://i.hizliresim.com/o8t5eu5.PNG)

### 1) Building the workspace
	
	$ cd ~/tiago_public_ws
	$ catkin build

### 2) Starting the simulation:

	$ roslaunch tiago_iaslab_simulation start_simulation.launch world_name:=ias_lab_room_full_tables

### 3) Starting the apriltag:

	$ roslaunch tiago_iaslab_simulation apriltag.launch

### 4) Starting the navigation:

	$ roslaunch tiago_iaslab_simulation navigation.launch

### 5) Running the human node:

	$ rosrun tiago_iaslab_simulation human_node

### 6) Configuring paramteres

Go to /config directory inside our package that will depend on where you put the files in your machine
	
	$ cd ~/tiago_public_ws/src/ir2223_group_19/tiago_iaslab_simulation/config
	$ rosparam load param.yaml

### 7) Starting the detection node:
	
Go to /scripts directory inside our package that will depend on where you put the files in your machine

	$ cd ~/tiago_public_ws/src/ir2223_group_19/tiago_iaslab_simulation/scripts
	$ chmod +x *.py
	$ rosrun tiago_iaslab_simulation tags_process.py
or,

	$ python tags_process.py
### Note:
If you encounter this error 

	/usr/bin/env: ‘python\r’: No such file or directory
Do the following inside /scripts directory:
	
	$ sudo apt install dos2unix
	$ dos2unix *.py

### 8) Starting the navigation server:

	$ cd ~/tiago_public_ws/src/ir2223_group_19/tiago_iaslab_simulation/scripts
	$ rosrun tiago_iaslab_simulation move_srv.py
or,

	$ python move_srv.py

### 9) Starting the pick server:

	$ cd ~/tiago_public_ws/src/ir2223_group_19/tiago_iaslab_simulation/scripts
	$ rosrun tiago_iaslab_simulation pick.py
or,

	$ python pick.py

### 10) Starting the place server:

	$ cd ~/tiago_public_ws/src/ir2223_group_19/tiago_iaslab_simulation/scripts
	$ rosrun tiago_iaslab_simulation place.py
or,

	$ python place.py

### 11) Finally, starting the main node:

	$ cd ~/tiago_public_ws/src/ir2223_group_19/tiago_iaslab_simulation/scripts
	$ rosrun tiago_iaslab_simulation main.py
or,

	$ python main.py



# Report and video link

https://drive.google.com/drive/folders/1LiuZGObPPD1PmshwVTLND4qRcyQgVKm6?usp=share_link

