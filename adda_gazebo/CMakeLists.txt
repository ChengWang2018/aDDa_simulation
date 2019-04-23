cmake_minimum_required(VERSION 2.8.3)
project(adda_gazebo)
find_package(catkin REQUIRED COMPONENTS
        )


catkin_package( CATKIN_DEPENDS
)

install(DIRECTORY launch urdf robot_control
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
