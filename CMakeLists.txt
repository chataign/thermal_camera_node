cmake_minimum_required(VERSION 2.8.3)

project(thermal_camera_node)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs image_transport)

set(THERMALLIB "/usr/lib/")
set(THERMALINC "/usr/include/")
set(THERMAL_LIBRARY libthermalgrabber.so)

include_directories(${THERMALINC})
link_directories(${THERMALLIB})

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(thermal_camera src/thermal_camera_node.cpp)

target_link_libraries(thermal_camera ${THERMAL_LIBRARY} ${catkin_LIBRARIES})