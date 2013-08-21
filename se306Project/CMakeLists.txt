cmake_minimum_required(VERSION 2.4.6)
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

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#target_link_libraries(example ${PROJECT_NAME})



rosbuild_add_executable(Grass0 src/Grass0.cpp)
rosbuild_add_executable(Grass1 src/Grass1.cpp)
rosbuild_add_executable(Grass2 src/Grass2.cpp)
rosbuild_add_executable(Grass3 src/Grass3.cpp)
rosbuild_add_executable(Grass4 src/Grass4.cpp)
rosbuild_add_executable(Grass5 src/Grass5.cpp)
rosbuild_add_executable(Grass6 src/Grass6.cpp)
rosbuild_add_executable(Grass7 src/Grass7.cpp)
rosbuild_add_executable(Grass8 src/Grass8.cpp)
rosbuild_add_executable(farmer src/farmer.cpp)
rosbuild_add_executable(sheepdog src/sheepdog.cpp)
rosbuild_add_executable(SheepNode src/SheepNode.cpp)
rosbuild_add_executable(SheepMove src/SheepMove.cpp)
rosbuild_add_executable(truck src/truck.cpp)
rosbuild_add_executable(listener src/listener.cpp)
