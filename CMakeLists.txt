cmake_minimum_required(VERSION 2.8.12)  # Ships with Ubuntu 14.04 (Trusty)

project(pose_graph)

#set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall   -O0")

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall  -O3")


find_package(Boost REQUIRED COMPONENTS iostreams)
find_package(Ceres REQUIRED COMPONENTS SparseLinearAlgebraLibrary)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)

#find_package(Lua REQUIRED)
set(LUA_INCLUDE_DIR /usr/include/lua5.2)
set(LUA_LIBRARIES /usr/lib/x86_64-linux-gnu/liblua5.2.so /usr/lib/x86_64-linux-gnu/libm.so)

add_definitions(-DEIGEN_DONT_ALIGN_STATICALLY)

find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs  message_filters laser_geometry  rosbag)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${PROJECT_BINARY_DIR})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${CERES_INCLUDE_DIRS})
include_directories(${Pangolin_INCLUDE_DIRS})
file(GLOB_RECURSE ALL_SRCS "*.cc")


add_executable(${PROJECT_NAME}  ${ALL_SRCS})

add_definitions(${PCL_DEFINITIONS})

target_link_libraries(${PROJECT_NAME}
${LUA_LIBRARIES}
${catkin_LIBRARIES}
${Boost_LIBRARIES}
${CERES_LIBRARIES}
${Pangolin_LIBRARIES}
dl
)

