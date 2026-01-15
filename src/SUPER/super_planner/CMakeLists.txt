cmake_minimum_required(VERSION 3.5)
project(super_planner)

set(CMAKE_CXX_STANDARD 17)
option(RELEASE "Enable Release mode" ON)
#set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-O3 -Wall")

add_compile_options(-Werror=return-type)
add_definitions(-DROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/")

# Define the voxelize and raycasting method
add_definitions(-DORIGIN_AT_CORNER)
add_definitions(-DFMT_HEADER_ONLY)
add_definitions(-DUSE_ROS2)


string(TOUPPER $ENV{ROS_DISTRO} ROS_VERSION)
message(STATUS "ROS version: ${ROS_VERSION}")

# for CLION to search the package, user should change to his/her own relative location
set(CMAKE_PREFIX_PATH ../../../install)
include_directories(
        SYSTEM
        include
        ../../../devel/include

        ${Eigen3_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rog_map REQUIRED)
find_package(mars_quadrotor_msgs REQUIRED)
find_package(px4ctrl_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)


set(includes
        include
        ${Eigen_INCLUDE_DIRS}
        #            ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

set(third_party_libs
        #            ${OpenCV_LIBRARIES}
        ${PCL_LIBRARIES}
        yaml-cpp
        -ldw
)

set(ros_libs
        rclcpp
        std_msgs
        mars_quadrotor_msgs
        px4ctrl_msgs
        sensor_msgs
        tf2_ros
        nav_msgs
        geometry_msgs
        visualization_msgs
        tf2_ros
        pcl_conversions
        rog_map
)

include_directories(
        ${includes}
)


#=================FOR libsuper ============================#
file(GLOB_RECURSE SRC_FILES CONFIGURE_DEPENDS src/*.cpp include/*.h include/*.hpp)
# Add utils files to the library
list(APPEND SRC_FILES
    src/utils/banded_system.cpp
    src/utils/ellipsoid.cpp
    src/utils/geometry_utils.cpp
    src/utils/lbfgs.cpp
    src/utils/minco.cpp
    src/utils/mvie.cpp
    src/utils/optimization_utils.cpp
    src/utils/piece.cpp
    src/utils/polynomial_interpolation.cpp
    src/utils/polytope.cpp
    src/utils/quickhull.cpp
    src/utils/root_finder.cpp
    src/utils/sdlp.cpp
    src/utils/trajectory.cpp
    src/utils/waypoint_trajectory_optimizer.cpp
)
add_library(super SHARED
        ${SRC_FILES}
)
target_include_directories(super PUBLIC
        ${includes})
ament_target_dependencies(super PUBLIC
        ${ros_libs}
        rog_map)
find_library(ROG_MAP_CORE_LIBRARY rog_map_core PATHS ${rog_map_LIBRARY_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/../../../install/rog_map/lib NO_DEFAULT_PATH)
if(ROG_MAP_CORE_LIBRARY)
    message(STATUS "Found rog_map_core library: ${ROG_MAP_CORE_LIBRARY}")
    target_link_libraries(super PUBLIC ${PCL_LIBRARIES} ${ROG_MAP_CORE_LIBRARY})
else()
    message(FATAL_ERROR "Could not find rog_map_core library")
endif()

# Add include directories explicitly for rog_map dependencies
if(rog_map_FOUND)
    target_include_directories(super PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${rog_map_INCLUDE_DIRS}
    )
    ament_target_dependencies(super PUBLIC rog_map)
endif()

#=================FOR Benchmark Node ============================#
add_executable(fsm_node
        Apps/fsm_node_ros2.cpp
)
target_link_libraries(fsm_node PUBLIC
        super
        ${PCL_LIBRARIES}
        ${third_party_libs}
)
ament_target_dependencies(fsm_node PUBLIC
        ${ros_libs}
)

#=================FOR Test Dynamic Params Node ============================#
add_executable(test_dynamic_params
        test/test_dynamic_params.cpp
)
target_link_libraries(test_dynamic_params PUBLIC
        super
        ${PCL_LIBRARIES}
        ${third_party_libs}
)
ament_target_dependencies(test_dynamic_params PUBLIC
        ${ros_libs}
)

#=================INSTALL all targets ============================#

install(
        TARGETS
        super
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(
        TARGETS
        fsm_node
        RUNTIME DESTINATION lib/${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(${includes})

install(DIRECTORY include/
        DESTINATION include/
)

# install the launch file
install(DIRECTORY
        launch
        config
        DESTINATION share/${PROJECT_NAME}/
)


ament_package()