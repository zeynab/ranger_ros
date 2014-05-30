cmake_minimum_required(VERSION 2.8.3)
project(ranger_ros)

# Find all required modules for compilation
find_package(catkin REQUIRED COMPONENTS
    tf
    geometry_msgs
    nav_msgs)

# Nothing that can be imported by other modules
catkin_package()


#And this will install python nodes
install(DIRECTORY nodes/
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} 
        USE_SOURCE_PERMISSIONS)

# Instructions to install launch files
install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch )

# Instructions to install configuration files
install(DIRECTORY nav_conf/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/nav_conf )
install(DIRECTORY maps/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/maps )
