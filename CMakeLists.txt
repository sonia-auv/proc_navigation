cmake_minimum_required(VERSION 2.8.3)
project(proc_navigation)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING
            "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
            FORCE)
endif (NOT CMAKE_BUILD_TYPE)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -std=c++11 -O3")
set(proc_navigation_SRC_DIR "src")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(
        catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        message_generation
        provider_dvl
        sonia_msgs
        lib_atlas
)


add_message_files(
        DIRECTORY msg
        FILES
        #DVL.msg
)


catkin_package(
        INCLUDE_DIRS ${proc_navigation_SRC_DIR}
        LIBRARIES
        CATKIN_DEPENDS roscpp message_runtime std_msgs provider_dvl sonia_msgs
)
## System dependencies are found with CMake's conventions
file(GLOB_RECURSE proc_navigation_FILES "${proc_navigation_SRC_DIR}/*.cc" "${proc_navigation_SRC_DIR}/*.h")


include_directories(
        ${catkin_INCLUDE_DIRS}
        ${proc_navigation_SRC_DIR}
        ${lib_atlas_INCLUDE_DIRS}
        ${provider_dvl_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node ${proc_navigation_SRC_DIR}/${PROJECT_NAME}/main.cc ${proc_navigation_FILES})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${lib_atlas_LIBRARIES})
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_generate_messages_cpp)
