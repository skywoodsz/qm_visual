cmake_minimum_required(VERSION 3.0.2)
project(ibvs_control)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# release
SET(CMAKE_BUILD_TYPE "Release")

find_package(pinocchio REQUIRED)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        sensor_msgs
        cv_bridge
        image_transport
        tf
        nav_msgs
        tf2_ros
        dynamic_reconfigure
        )

generate_dynamic_reconfigure_options(
        cfg/Param.cfg
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ibvs_control
        CATKIN_DEPENDS roscpp 
        DEPENDS
)

###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${pinocchio_INCLUDE_DIRS}
)

link_directories(
        ${pinocchio_LIBRARY_DIRS}
)

add_library(ibvs_eye_to_hand_moving_object
        src/ibvs_eye_to_hand_moving_object.cpp
        )
add_dependencies(ibvs_eye_to_hand_moving_object ${PROJECT_NAME}_gencfg)
target_link_libraries(ibvs_eye_to_hand_moving_object
        ${catkin_LIBRARIES}
        ${pinocchio_LIBRARIES}
        )
###########
##  EXE  ##
###########
add_executable(ibvs_eye_to_hand_moving_object_node
        src/ibvs_eye_to_hand_moving_object_node.cpp
        )
target_link_libraries(ibvs_eye_to_hand_moving_object_node
        ${catkin_LIBRARIES} ibvs_eye_to_hand_moving_object
        )