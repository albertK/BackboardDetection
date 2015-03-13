cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(backboard_detection)

find_package(PCL 1.7.2 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (backboard_detection_final src/backboard_detection_v8.5.cpp)
target_link_libraries (backboard_detection_final ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable (backboard_detection_final_v2 src/backboard_detection_v8.6.cpp src/openni_frame_source_albert.cpp)
target_link_libraries (backboard_detection_final_v2 ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable (communication src/communication.cpp)
add_executable (communication_test src/communication_test.cpp)