# \file     CMakeLists.txt
# \author   Antoine Dozois <dozois.a@gmail.com>
# \date     01/06/2018
# \copyright    2018 Club SONIA AUV, ETS. All rights reserved.
# Use of this source code is governed by the MIT license that can be
# found in the LICENSE file.

#===============================================================================
# G L O B A L   P R O J E C T   C O N F I G U R A T I O N

cmake_minimum_required(VERSION 2.8.3)
project(proc_deep_detector)

set(CMAKE_CXX_STANDARD 11)
set(proc_deep_detector_SRC_DIR "src")


set(EIGEN334_INCLUDE_DIRS $ENV{HOME}/eigen3)

#===============================================================================
# C A T K I N   C O N F I G U R A T I O N

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        message_generation
        image_transport
        cv_bridge
        roslaunch
        lib_atlas
        OpenCV
)

# CMake does not detect new file when globbing.
# Please, regenerate your CMake solution if you add a file.
file(GLOB srv_files RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/srv/"
        "${CMAKE_CURRENT_SOURCE_DIR}/srv/*.srv")
add_service_files(DIRECTORY srv FILES ${srv_files})

file(GLOB msg_files RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/srv/"
        "${CMAKE_CURRENT_SOURCE_DIR}/msg/*.msg")
add_message_files(DIRECTORY msg FILES ${msg_files})

generate_messages(DEPENDENCIES std_msgs geometry_msgs sensor_msgs )

catkin_package(
        INCLUDE_DIRS ${proc_deep_detector_SRC_DIR}
        LIBRARIES
        CATKIN_DEPENDS
        roscpp
        std_msgs
        message_runtime
        image_transport
        cv_bridge
        roslaunch
        lib_atlas
)

#===============================================================================
# I N C L U D E   F I L E S

file(GLOB_RECURSE proc_deep_detector_FILES
        "${proc_deep_detector_SRC_DIR}/*.cc"
        "${proc_deep_detector_SRC_DIR}/*.h")

INCLUDE_DIRECTORIES(
        ${catkin_INCLUDE_DIRS}
        ${${PROJECT_NAME}_SRC_DIR}
        ${lib_atlas_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        yaml-cpp
        /usr/local/include/tensorflow/
        ${EIGEN334_INCLUDE_DIRS}
)

LINK_DIRECTORIES(/usr/local/lib/)


add_executable(${PROJECT_NAME}_node ${${PROJECT_NAME}_SRC_DIR}/${PROJECT_NAME}/main.cc ${proc_deep_detector_FILES})

TARGET_LINK_LIBRARIES(${PROJECT_NAME}_node
        tensorflow_cc
        tensorflow_framework
        ${OpenCV_LIBRARIES}
        ${catkin_LIBRARIES}
        yaml-cpp
        )

add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_generate_messages_cpp)
