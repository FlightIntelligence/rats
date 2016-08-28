BeSwarm is a pure Java implementation to control and simulate drones.

#  Installation
BeSwarm currently only supports the [ROS](http://www.ros.org) communication middleware (version Indigo).

To work with ROS, BeSwarm requires [RosJava](http://wiki.ros.org/rosjava/Tutorials/indigo/Installation) to be installed.

BeSwarm currently supports the following drones and simulators (via ROS):

- [Parrot Bebop](http://www.parrot.com/ca/products/bebop-drone/) 1.0 and 2.0 drones: (via [bebop_autonomy](http://bebop-autonomy.readthedocs.io/en/latest/installation.html))
- [TUM simulator for AR drone](https://github.com/dougvk/tum_simulator)
- [Crates simulator](https://bitbucket.org/vicengomez/crates)
- [hector_quadrotor](https://github.com/tu-darmstadt-ros-pkg/hector_quadrotor) is going to be supported.

# Getting started
- [Create a RosJava package](http://wiki.ros.org/rosjava_build_tools/Tutorials/indigo/Creating%20Rosjava%20Packages#RosJava_Catkin_Packages)
- Clone this repository into your package directory
- Open `your_package/settings.gradle` and add `include 'BeSwarm'` to the end of file
- Compile the project by one of the following ways:
    1. Compiling with Gradle, this will only compile your project alone: 
        - Go to `your_package/BeSwarm/` and run `../gradlew installApp`
    2. Compiling using `catkin_make`, with this, cmake will run through your entire workspace and when it gets to your new project, it will pass off the build to gradle:
        - Open `your_package/CMakelist.txt`
        - Find and replace the line `catkin_rosjava_setup()` by `catkin_rosjava_setup(installApp publishMavenJavaPublicationToMavenRepository)`
        - Go to your workspace and run `catkin_make`
        - For more information on building a RosJava project, see [this page, section 2](http://wiki.ros.org/rosjava_build_tools/Tutorials/indigo/WritingPublisherSubscriber%28Java%29)

- Run your ros node(s): 
    - Go to `your_package/BeSwarm/build/install/BeSwarm/bin`
    - Run `./BeSwarm your_rosnode_class_directory`