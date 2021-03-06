# \file     CMakeLists.txt
# \author   Etienne Boudreault-Pilon <etienne.b.pilon@gmail.com>
# \author   Thibaut Mattio <thibaut.mattio@gmail.com>
# \date     08/05/2015
#
# \copyright Copyright (c) 2015 S.O.N.I.A. All rights reserved.
#
# \section LICENSE
#
# This file is part of S.O.N.I.A. software.
#
# S.O.N.I.A. software is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# S.O.N.I.A. software is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with S.O.N.I.A. software. If not, see <http://www.gnu.org/licenses/>.

#===============================================================================
# G L O B A L   P R O J E C T   C O N F I G U R A T I O N

cmake_minimum_required(VERSION 2.8.3)
project(proc_navigation)

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
endif (NOT CMAKE_BUILD_TYPE)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -std=c++11 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -std=c++11 -O3 -s")

set(proc_navigation_SRC_DIR "src")

#===============================================================================
# C A T K I N   C O N F I G U R A T I O N

find_package(PkgConfig)

find_package(
    catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    sensor_msgs
    tf
    sonia_common
)

catkin_package(
    INCLUDE_DIRS ${proc_navigation_SRC_DIR}
    LIBRARIES
    CATKIN_DEPENDS roscpp std_msgs sonia_common
)

#===============================================================================
# I N C L U D E   F I L E S

file(GLOB_RECURSE proc_navigation_FILES
        "${proc_navigation_SRC_DIR}/*.cc"
        "${proc_navigation_SRC_DIR}/*.h")

list(REMOVE_ITEM proc_navigation_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/${proc_navigation_SRC_DIR}/${PROJECT_NAME}/main.cc)

# Include all the directories where source can be found.
include_directories(
        ${catkin_INCLUDE_DIRS}
        ${proc_navigation_SRC_DIR})


#==========================================================================
# C R E A T E   E X E C U T A B L E

add_executable(${PROJECT_NAME}_node ${proc_navigation_SRC_DIR}/${PROJECT_NAME}/main.cc ${proc_navigation_FILES})
target_link_libraries(${PROJECT_NAME}_node
                      ${catkin_LIBRARIES}
                      ${BFL_LIBRARIES})
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})

#==========================================================================
# U N I T   T E S T S

if (CATKIN_ENABLE_TESTING AND ${CMAKE_CURRENT_SOURCE_DIR}/test)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/test)
endif ()
