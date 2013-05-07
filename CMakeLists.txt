cmake_minimum_required(VERSION 2.4.6)

# Push the ROS_PACKAGE_PATH and ROS_ROOT we were passed up into the environment, to allow us to find things at configure time
#set(ENV{ROS_ROOT} ${ROS_ROOT})
#set(ENV{ROS_PACKAGE_PATH} ${ROS_PACKAGE_PATH})
# Push the PKG_CONFIG_PATH we were passed up into the environment, to allow us to find gazebo
set(ENV{PKG_CONFIG_PATH} ${PKG_CONFIG_PATH}:$ENV{PKG_CONFIG_PATH})

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

project(RobotController)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()
# Note: recovering PROJECT_NAME here to drcsim leads to build failure

# VERY IMPORTANT: Tell CMake not to strip off the RPATH from libs and executables during installation.  This is needed to resolve
# dependencies on ROS libraries.
set (CMAKE_INSTALL_RPATH_USE_LINK_PATH 1)

#find gazebo include (FindPkgConfig)
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
else()
  message(FATAL_ERROR "pkg-config is required; please install it")
endif()

# depends on DRCVehiclePlugin
include_directories(
  ${GAZEBO_INCLUDE_DIRS}
)
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${PLUGIN_LINK_DIRECTORIES}
)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin/)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib/)
#INSTALL(CODE "EXECUTE_PROCESS(COMMAND mkdir -p ${CMAKE_INSTALL_PREFIX}/lib/${DRC_SIM_PROJECT_NAME}-${DRC_SIM_VERSION}/plugins/)")

#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_genmsg()
rosbuild_gensrv()

rosbuild_add_boost_directories()

# needs pr2_controller_manager
#rosbuild_add_library(gazebo_ros_controller_manager gazebo_ros_controller_manager.cpp)
#rosbuild_link_boost(gazebo_ros_controller_manager thread)

# needs roscpp, trajectory_msgs
#rosbuild_add_executable(pub_joint_trajectory_test pub_joint_trajectory_test.cpp)
#rosbuild_add_executable(pub_joint_commands pub_joint_commands.cpp)
#rosbuild_add_executable(pub_atlas_command pub_atlas_command.cpp)
#rosbuild_add_executable(pub_atlas_state pub_atlas_state.cpp)
#rosbuild_add_executable(pub_joint_states pub_joint_states.cpp)

#rosbuild_add_library(gazebo_ros_imu gazebo_ros_imu.cpp)

#rosbuild_add_library(gazebo_ros_joint_trajectory gazebo_ros_joint_trajectory.cpp)

#rosbuild_add_library(gazebo_ros_api_plugin gazebo_ros_api_plugin.cpp)

#rosbuild_add_library(gazebo_ros_p3d gazebo_ros_p3d.cpp)

#rosbuild_add_library(gazebo_ros_force gazebo_ros_force.cpp)


# compile ControllerPlugin
rosbuild_add_library(ControllerPlugin src/ControllerPlugin.cpp)

#install(FILES manifest.xml DESTINATION ${CMAKE_INSTALL_PREFIX}/share/ROSControllerPlugin/ros/${PROJECT_NAME})
#install(DIRECTORY srv srv_gen msg msg_gen src DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${DRC_SIM_PROJECT_NAME}-${DRC_SIM_VERSION}/ros/${PROJECT_NAME} PATTERN ".*" EXCLUDE)
# install executables as private to the package, for launching via roslaunch or rosrun
#install(TARGETS pub_joint_trajectory_test pub_joint_commands pub_atlas_state pub_atlas_command pub_joint_states actionlib_server 
#        RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${DRC_SIM_PROJECT_NAME}-${DRC_SIM_VERSION}/ros/${PROJECT_NAME})
# for backwards compatibility, also install pub_atlas_command globally; to
# remove this, we need to update some launch files and scripts in atlas_msgs
install(TARGETS ControllerPlugin
        LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/gazebo_plugins)
#install(TARGETS gazebo_ros_controller_manager gazebo_ros_camera_utils gazebo_ros_camera MultiCameraPlugin gazebo_ros_multicamera gazebo_ros_laser gazebo_ros_imu gazebo_ros_joint_trajectory gazebo_ros_api_plugin gazebo_ros_force gazebo_ros_p3d MultiSenseSLPlugin SandiaHandPlugin AtlasPlugin VRCPlugin VRCScoringPlugin DRCVehicleROSPlugin pub_atlas_command
#        LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/${DRC_SIM_PROJECT_NAME}-${DRC_SIM_VERSION}/plugins
#        RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)


