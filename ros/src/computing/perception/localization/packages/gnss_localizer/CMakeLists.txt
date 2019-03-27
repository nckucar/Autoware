cmake_minimum_required(VERSION 2.8.3)
project(gnss_localizer)

find_package(catkin REQUIRED COMPONENTS
        PCL REQUIRED
        autoware_build_flags
        roscpp
        std_msgs
        nmea_msgs
        gnss
        tf
        autoware_config_msgs
        velodyne_pointcloud
        ndt_cpu
        ${PCL_OPENMP_PACKAGES}
        )

catkin_package(
        CATKIN_DEPENDS std_msgs velodyne_pointcloud autoware_config_msgs ndt_cpu ${PCL_OPENMP_PACKAGES}
        DEPENDS PCL
)

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
        ${catkin_INCLUDE_DIRS}
)

add_executable(fix2tfpose
        nodes/fix2tfpose/fix2tfpose.cpp
        )
target_link_libraries(fix2tfpose ${catkin_LIBRARIES})
add_dependencies(fix2tfpose ${catkin_EXPORTED_TARGETS})


add_executable(nmea2tfpose
        nodes/nmea2tfpose/nmea2tfpose_core.cpp
        nodes/nmea2tfpose/nmea2tfpose_node.cpp
        )
target_include_directories(nmea2tfpose PRIVATE nodes/nmea2tfpose ${catkin_INCLUDE_DIRS})
target_link_libraries(nmea2tfpose ${catkin_LIBRARIES})
add_dependencies(nmea2tfpose ${catkin_EXPORTED_TARGETS})

add_executable(gnss_mapping nodes/gnss_mapping/gnss_mapping.cpp)
target_link_libraries(gnss_mapping ${catkin_LIBRARIES})
add_dependencies(gnss_mapping ${catkin_EXPORTED_TARGETS})

install(TARGETS nmea2tfpose fix2tfpose gnss_mapping
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)
