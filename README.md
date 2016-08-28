#  Installation guide
This project works with Ubuntu 14.04 and ROS-indigo

1. Install some dependencies

	```
	sudo apt-get install build-essential python-rosdep python-catkin-tools
	```

2. Install [RosJava](http://wiki.ros.org/rosjava/Tutorials/indigo/Installation)

3. Clone the repository and initialize submodules


	```
	git clone https://github.com/hoangtungdinh/rats.git
	cd rats/
	git submodule init
	```

4. Install the ros driver (bebop-autonomy) for bebops
	
	```
	cd bebop_ws/
	catkin init
	rosdep update
	rosdep install --from-paths src -i
	catkin build -DCMAKE_BUILD_TYPE=RelWithDebInfo
	cd ..
	```

5. Install java packages (BeSwarm and ARLocROS)

	```
	cd rats_ws/src/
	catkin_init_workspace
	cd ..
	catkin_make
	cd ..
	```

6. Source the two workspaces

	```
	source bebop_ws/devel/setup.zsh
	source rats_ws/devel/setup.zsh

	```
	
