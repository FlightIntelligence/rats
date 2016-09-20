A detailed installation guide could be found [here](install.txt).

#  Installation guide
This project works with Ubuntu 14.04 and ROS-indigo

1. Install some dependencies

	```
	sudo apt-get install build-essential python-rosdep python-catkin-tools
	```

2. Install [RosJava](http://wiki.ros.org/rosjava/Tutorials/indigo/Installation). If you install RosJava from source, remember to source the package before moving on to the next step.

3. Clone the repository and initialize submodules


	```
	git clone https://github.com/hoangtungdinh/rats.git
	cd rats/
	git submodule init
	git submodule update
	```

4. Install the ros driver (bebop-autonomy) for bebops
	
	```
	cd bebop_ws/
	catkin init
	rosdep update
	rosdep install --from-paths src -i
	catkin build -DCMAKE_BUILD_TYPE=RelWithDebInfo
	cd ..
	source bebop_ws/devel/setup.zsh
	```

5. Install java packages (BeSwarm and ARLocROS)

	```
	cd rats_ws/src/
	catkin_init_workspace
	cd ..
	catkin_make
	cd ..
	source rats_ws/devel/setup.zsh
	```

## Issues
If you have **peer not authenticated** error, make sure that you are using Java 8.

If ARLocROS gives execption "no opencv-248 in java library path" you have to add the path to the native opencv library.
In linux that can be done, normally with:

    ```
    export  LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/jni/
    ```
 

	
