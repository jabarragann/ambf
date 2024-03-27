#  Software License Agreement (BSD License)
#  Copyright (c) 2019-2021, AMBF.
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

set (ROS1_depends AMBF ambf_server)
find_package (catkin QUIET
  COMPONENTS ${ROS1_depends})

# ROS 1
if (catkin_FOUND)
  add_definitions (-DROS1)
  set (ROS_VERSION 1)
  message(STATUS "ROS VERSION = 1")
  catkin_package(
    LIBRARIES ros_comm_plugin
    CATKIN_DEPENDS ${ROS1_depends}
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
message ("--------------------- ${AMBF_INCLUDE_DIRS} -------------")
include_directories (${AMBF_INCLUDE_DIRS})

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
add_library(ros_comm_plugin ${HDR_FILES} ${SRC_FILES})

if (ROS_VERSION EQUAL 1)
  add_dependencies(ros_comm_plugin ${catkin_EXPORTED_TARGETS})
  ## Specify libraries to link a library or executable target against
  target_link_libraries(ros_comm_plugin ${catkin_LIBRARIES})
else(ROS_VERSION EQUAL 2)
  ament_target_dependencies(
    ros_comm_plugin
    ${ROS2_depends}
    )
  target_link_libraries(ros_comm_plugin
    ${rclcpp_LIBRARIES}
    )
  # target_link_libraries(ambf_server
  #   ${ambf_msgs_LIBRARIES}
  # )
  ament_package ()
endif()
