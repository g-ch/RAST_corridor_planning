cmake_minimum_required(VERSION 3.0.2)
project(rast_corridor_planning)

add_definitions(-std=c++17 -g -O3 -ftree-vectorize -ffast-math -march=native -lpthread)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_filters
  sensor_msgs #
  image_transport
  mav_msgs
)


add_definitions(${PCL_DEFINITIONS})

catkin_package(
)


find_package(osqp REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)


add_library(MINIMUM_SNAP_CORRIDOR
        src/corridor_minisnap.cpp
        )


include_directories(/usr/local/include/munkres)
set(MunkresLIB
        /usr/local/lib/libmunkres.a
        )

add_executable(map_sim_example src/map_sim_example.cpp)
target_link_libraries(map_sim_example ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${MunkresLIB})

add_executable(planning_node src/planning_node.cpp)
target_link_libraries(planning_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${MunkresLIB} MINIMUM_SNAP_CORRIDOR osqp::osqp)
