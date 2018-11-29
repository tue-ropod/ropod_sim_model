cmake_minimum_required(VERSION 2.8.3)
project(ropod_sim_model)


find_package(catkin REQUIRED COMPONENTS
    geolib2
    code_profiler
    roslib   
    roscpp
    std_msgs
)


# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

catkin_package()

# ------------------------------------------------------------------------------------------------
#                                              BUILD
# ------------------------------------------------------------------------------------------------


include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})



# Build our plugin
include_directories(
    include
    3rdparty
    ${catkin_INCLUDE_DIRS}
)

#list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(Odometry_Plugin SHARED simulator/gazebo_plugins/odometry/src/Odometry_Plugin.cc)
target_link_libraries(Odometry_Plugin ${GAZEBO_libraries} ${roscpp_LIBRARIES})

add_library(Custom_Laser_Plugin SHARED simulator/gazebo_plugins/laser/src/Custom_Laser_Plugin.cc)
target_link_libraries(Custom_Laser_Plugin ${GAZEBO_libraries} ${roscpp_LIBRARIES})

#add_library(animated_cylinder2 SHARED simulator/gazebo_plugins/animatedCylinder2/animated_cylinder2.cc)
#target_link_libraries(animated_cylinder2 ${GAZEBO_LIBRARIES})

#add_library(animated_cylinder SHARED simulator/gazebo_plugins/animatedCylinder/animated_cylinder.cc)
#target_link_libraries(animated_cylinder ${GAZEBO_LIBRARIES})

#add_library(animated_box SHARED simulator/gazebo_plugins/animatedBox/animated_box.cc)
#target_link_libraries(animated_box ${GAZEBO_LIBRARIES})

#add_executable(integrated_main simulator/gazebo_plugins/animatedBox/integrated_main.cc)
#target_link_libraries(integrated_main ${GAZEBO_LIBRARIES} pthread)

#add_executable(independent_listener simulator/gazebo_plugins/animatedBox/independent_listener.cc)
#target_link_libraries(independent_listener ${GAZEBO_LIBRARIES} pthread)

