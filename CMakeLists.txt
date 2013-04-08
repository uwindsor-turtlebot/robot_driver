cmake_minimum_required(VERSION 2.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")
find_package(libusb-1.0 REQUIRED)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${LIBUSB_1_INCLUDE_DIRS})

add_subdirectory(wrappers/cpp)

rosbuild_add_library(ROSRobot
	src/ROSRobot/RobotDriver.cpp
	src/ROSRobot/RobotCommander.cpp
	src/ROSRobot/RobotOdometry.cpp 
	src/ROSRobot/RobotSensors.cpp
	src/ROSRobot/MovementDetector.cpp
	src/ROSRobot/RotationDetector.cpp
	src/ROSRobot/CliffDetector.cpp
	src/ROSRobot/FrontBumperDetector.cpp
	src/ROSRobot/TimeDetector.cpp)

rosbuild_add_library(Kinect
	src/Kinect/MyFreenectDevice.cpp
	src/Kinect/Mutex.cpp
	src/Kinect/VisionDetector.cpp)
target_link_libraries(Kinect freenect)

rosbuild_add_executable(turtlebot_driver.o src/TurtleBot.cpp)
target_link_libraries(turtlebot_driver.o ROSRobot Kinect)

rosbuild_add_executable(test/movementdetector.o src/Test_MovementDetector.cpp)
target_link_libraries(test/movementdetector.o ROSRobot)

rosbuild_add_executable(test/rotationdetector.o src/Test_RotationDetector.cpp)
target_link_libraries(test/rotationdetector.o ROSRobot)

rosbuild_add_executable(test/frontbumperdetector.o src/Test_FrontBumperDetector.cpp)
target_link_libraries(test/frontbumperdetector.o ROSRobot)

rosbuild_add_executable(test/cliffdetector.o src/Test_CliffDetector.cpp)
target_link_libraries(test/cliffdetector.o ROSRobot)

rosbuild_add_executable(test/timedetector.o src/Test_TimeDetector.cpp)
target_link_libraries(test/timedetector.o ROSRobot)

rosbuild_add_executable(test/visiondetector.o src/Test_VisionDetector.cpp)
target_link_libraries(test/visiondetector.o ROSRobot)
target_link_libraries(test/visiondetector.o Kinect)
