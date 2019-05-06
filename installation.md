# aDDa_simulation

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

### adda_description
contains all necessary files for the URDF model of the aDDa-vehicle. The model can be visualized in RVIZ.
### adda_gazebo
contains launchfiles, configurations, worlds and maps to use the aDDa_description model in gazebo or start a combined simulation using gazebo physics and sensor simulation combined with a pathplanning module. Goals can be set by the action libary or simply using RVIZ NavGoal.
### robot_control
contains the control node for the gazebo simulation. The control node translates cmd_vel into commands for an ackerman four wheel driven ackerman vehicle
