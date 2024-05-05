#  Software License Agreement (BSD License)
#  Copyright (c) 2019-2024, AMBF.
#  (https://github.com/WPI-AIM/ambf)
#
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#  * Redistributions of source code must retain the above copyright
#  notice, this list of conditions and the following disclaimer.
#
#  * Redistributions in binary form must reproduce the above
#  copyright notice, this list of conditions and the following
#  disclaimer in the documentation and/or other materials provided
#  with the distribution.
#
#  * Neither the name of authors nor the names of its contributors may
#  be used to endorse or promote products derived from this software
#  without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#
#  $Author: Adnan Munawar $
#  $Date:  $
#  $Rev:  $


cmake_minimum_required(VERSION 3.10)
project (ros_comm_plugin)

set (ROS1_depends ambf_server cv_bridge image_transport)
find_package (catkin QUIET
  COMPONENTS ${ROS1_depends})

# ROS 1
if (catkin_FOUND)
  add_definitions (-DROS1)
  set (ROS_VERSION 1)
  message(STATUS "ROS VERSION = 1")

  catkin_package(
  LIBRARIES depth_streamer_plugin object_comm_plugin video_streamer_plugin world_comm_plugin
  CATKIN_DEPENDS ambf_server # Removed AMBF from here
  )
else (catkin_FOUND)

  # look for ROS2
  find_package (ament_cmake QUIET)
  if (ament_cmake_FOUND)
    add_definitions (-DROS2)
    set (ROS_VERSION 2)
    message(STATUS "ROS VERSION = 2")
    set (ROS2_depends rclcpp std_msgs geometry_msgs sensor_msgs diagnostic_msgs tf2_ros tf2_msgs ambf_msgs)
    foreach (_dep ${ROS2_depends})
        find_package (${_dep} REQUIRED)
    endforeach()
    find_package(rclcpp REQUIRED)
  endif (ament_cmake_FOUND)
endif (catkin_FOUND)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
if (ROS_VERSION EQUAL 1)
  include_directories(include ${catkin_INCLUDE_DIRS})
else(ROS_VERSION 2)
  include_directories(
    include
    ${rclcpp_INCLUDE_DIRS}
  )
endif()

find_package (AMBF REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)

include_directories (${AMBF_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})

set (HDR_FILES
  DepthStreamerPlugin.h
  ObjectCommPlugin.h
  VideoStreamerPlugin.h
  WorldCommPlugin.h)

set (SRC_FILES
  DepthStreamerPlugin.cpp
  ObjectCommPlugin.cpp
  VideoStreamerPlugin.cpp
  WorldCommPlugin.cpp)

## Declare a C++ library
## this is weird
##link_directories(/home/lauraconnolly/ros2-changes/restructure/catkin_ws/src/ambf/core/external/DHD/lib/lin-x86_64)
link_directories(../../core/external/DHD/lib/lin-x86_64)


## add_library(ros_comm_plugin ${HDR_FILES} ${SRC_FILES})
add_library(depth_streamer_plugin SHARED DepthStreamerPlugin.cpp)
add_library(object_comm_plugin SHARED ObjectCommPlugin.cpp)
add_library(video_streamer_plugin SHARED VideoStreamerPlugin.cpp)
add_library(world_comm_plugin SHARED WorldCommPlugin.cpp)




find_package (OpenCV QUIET)
if (OpenCV_FOUND)
  message (STATUS "-- *** FOUND OPENCV")
  if (cv_bridge_FOUND AND image_transport_FOUND)
    message(STATUS "-- *** FOUND ROS_CV_BRIDGE and IMAGE_TRANSPORT, ENABLING VIDEO PUBLISHING SUPPORT")
    add_definitions (-DAF_ENABLE_OPEN_CV_SUPPORT)
    include_directories (${OpenCV_INCLUDE_DIRS})
    ## NEED to fix this target_link_libraries (ros_comm_plugin ${OpenCV_LIBRARIES})
  else()
    message (STATUS "-- *** ROS CV_BRIDGE and IMAGE_TRANSPORT NOT FOUND")
  endif ()
else ()
  message (STATUS "-- *** OPENCV NOT FOUND")
endif ()

if (ROS_VERSION EQUAL 1)
  ## add_dependencies(ros_comm_plugin ${catkin_EXPORTED_TARGETS})
  ## Specify libraries to link a library or executable target against
  ## target_link_libraries(ros_comm_plugin ${catkin_LIBRARIES})
  add_dependencies(depth_streamer_plugin ${catkin_EXPORTED_TARGETS})
  add_dependencies(object_comm_plugin ${catkin_EXPORTED_TARGETS})
  add_dependencies(video_streamer_plugin ${catkin_EXPORTED_TARGETS})
  add_dependencies(world_comm_plugin ${catkin_EXPORTED_TARGETS})

  target_link_libraries(depth_streamer_plugin ${catkin_LIBRARIES} ${AMBF_LIBRARIES} ${Boost_LIBRARIES} drd)
  target_link_libraries(object_comm_plugin ${catkin_LIBRARIES} ${AMBF_LIBRARIES} ${Boost_LIBRARIES} drd)
  target_link_libraries(video_streamer_plugin ${catkin_LIBRARIES} ${AMBF_LIBRARIES} ${Boost_LIBRARIES} drd)
  target_link_libraries(world_comm_plugin ${catkin_LIBRARIES} ${AMBF_LIBRARIES} ${Boost_LIBRARIES} drd)

else(ROS_VERSION EQUAL 2)
  ## ament_target_dependencies(ros_comm_plugin ${ROS2_depends})
  ## target_link_libraries(ros_comm_plugin ${rclcpp_LIBRARIES})
  ament_target_dependencies(depth_streamer_plugin ${ROS2_depends})
  ament_target_dependencies(object_comm_plugin ${ROS2_depends})
  ament_target_dependencies(video_streamer_plugin ${ROS2_depends})
  ament_target_dependencies(world_comm_plugin ${ROS2_depends})

  target_link_libraries(depth_streamer_plugin ${rclcpp_LIBRARIES})
  target_link_libraries(object_comm_plugin ${rclcpp_LIBRARIES})
  target_link_libraries(video_streamer_plugin ${rclcpp_LIBRARIES})
  target_link_libraries(world_comm_plugin ${rclcpp_LIBRARIES})

  # target_link_libraries(ambf_server
  #   ${ambf_msgs_LIBRARIES}
  # )
  ament_package ()
endif()
