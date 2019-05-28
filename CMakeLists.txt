cmake_minimum_required(VERSION 2.8.3)
project(franka_sim)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

##Used for declaring/adding dependencies/etc for the CPP file
set(BINNAME door_demo)
set(SOURCES src/door_demo.cpp)

## Find catkin macros and libraries
find_package(catkin REQUIRED)

## System dependencies are found with CMake's conventions
find_package(catkin REQUIRED COMPONENTS 
        xacro
        moveit_core 
        moveit_visual_tools 
        moveit_ros_planning 
        moveit_ros_planning_interface
)

##Generate the urdf files from urdf.xacro files
file(GLOB xacro_files model/robots/*.urdf.xacro)
xacro_add_files(${xacro_files} TARGET media_files INORDER)

## The catkin_package macro generates cmake config files for your package
catkin_package(CATKIN_DEPENDS)

## Specify additional locations of header files
include_directories(
    include
    ${catkin_INCLUDE_DIRS} 
)

## Declare a C++ executable
add_executable(${BINNAME} ${SOURCES})

## Add cmake target dependencies of the executable
add_dependencies(${BINNAME} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${BINNAME} ${catkin_LIBRARIES})

## Mark executables and/or libraries for installation
install(TARGETS ${BINNAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

## Mark cpp header files for installation
install(DIRECTORY include DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY model DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
