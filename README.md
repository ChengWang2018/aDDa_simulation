# aDDa_simulation

The Gazebo Simulation simulates the aDDa vehicles (Mercedes S450) in the simulation environment Gazebo (http://gazebosim.org/). The program simulated in a customizable 3D environment. Gazebo simulates the physical and dynamic behavior of the car, the different sensors and the steering behavior. A detailed description and explanation of the simulation can be found in the thesis document.
Simulated sensors:
* stereo camera (further cameras can be added in future)
* LIDAR
* ADMA
* GPS

The sensors data is published on different ROS topics.

The car can be controlled by the /cmd_vel message using robot_control GUI, a joystick controller, or the integrated path planning module.
The path planning module implements the navigation stack, using predefined maps, the global planer and the TEB_local_planner. The module respects kinematic behaviors of the vehicle and is limited at 18.33m/s (50km/h). RVIZ or an action listener can be used to set goals for the planning module. The car will drive to the goal, if possible. Safety distance to obstacles is set to 4m.

To launch the Gazebo simulation run: 

roslaunch adda_gazebo parametrized.launch

It may take some time to start gazebo and load the model. If the tires are not displayed, try to run:

roslaunch adda_gazebo load_controller.launch

The car should spawn in an empty map in gazebo if everything works correctly.

Packages:
* **adda_description:** contains the description of the aDDa vehicle in URDF format. The meshes folder contaions the meshes used for the visualization. For computers with small computation power I recommend to choose a mesh with a smaller resolution (normal > reduced_1 > reduced_2). The meshes can be exchanged in the URDF file. Visualization_RVIZ launches a visualitation of the URDF model in RVIZ.
* **adda_gazebo:** contains files necessary to run the URDF model in the Gazbeo simulation and connect it with the path planning module. The control.yaml describes the PID controller which are used in the Hardware Resource Interface Layer as Interface to Gazbeo. The Gazbeo worlds are saved in the world folder, the map folder contains the predefined maps of the path planning module. The node folder contains the ground truth transformer, which transforms the actual velocity of the car into the base_link frame.
 * *parametrized.launch* [rviz:=false gui:=true paused:=false]: launches the model in an empty world in the gazebo simulation. Parameters can be changed by appending them (without brackets)
 * *navigation.launch* [map:=_mapname_, gui:=false]: launches necessary nodes for the path planning module (navigation stack with TEB_Local_planner), the map param can either be blank or the name of a yaml file of the map folder
 * *pathplanning.launch* [map:=_mapname_, gui:=false]: launches the gazebo simulation without gui together with the navigation modules. RVIZ is launched as interface. New goals can be set by rviz, the gui param allows to launch the GUI of Gazebo additionally to RVIZ Gui. Map is forwarded to navigation.launch
 * *joystick.launch:* launches joystick input node and converts the input to right format. If no joystick is found you probably chose the wrong input device number in the joy.config.yaml (Further Information can be found here: http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick )

* **robot_control:** the robot control nodes, simulating the ackerman steering. Twist message is converted to ackerman message before the robot control node calculates the exact steering and velocity request for every wheel.

If anything is missing, wrong or you want to contribute to the project, I would be happy to receive your email ;)
Bug reports and feature request are welcome too.

# Installation Guide for required ROS Packages

### For Kinetic

    sudo apt-get install ros-kinetic-navigation
    sudo apt-get install ros-kinetic-teb-local-planner
    sudo apt-get install ros-kinetic-gazebo-ros-controller
    sudo apt-get install ros-kinetic-controller-manager
    sudo apt-get install ros-kinetic-joint-state-controller
    sudo apt-get install ros-kinetic-effort-controllers
    sudo apt-get install ros-kinetic-velocity-controllers
    sudo apt-get install ros-kinetic-velodyne-gazebo-plugin
    sudo apt-get install ros-kinetic-joystick-driver
    sudo apt-get install ros-kinetic-teleop-twist-joy
    sudo apt-get install ros-kinetic-message-to-tf
    sudo apt-get install ros-kinetic-ackermann-msgs
    sudo apt-get install ros-kinetic-hector-gazebo-plugins


### In case of errors maybe try
    
    sudo apt-get install aptitude
    sudo aptitude install ros-kinetic-navigation
