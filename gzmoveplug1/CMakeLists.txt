cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(model_push SHARED model_push.cc)
target_link_libraries(model_push ${GAZEBO_LIBRARIES})
add_library(model_drive SHARED model_drive.cc)
target_link_libraries(model_drive ${GAZEBO_LIBRARIES})
add_library(model_drivecircle SHARED model_drivecircle.cc)
target_link_libraries(model_drivecircle ${GAZEBO_LIBRARIES})
add_library(model_drivesqr SHARED model_drivesqr.cc)
target_link_libraries(model_drivesqr ${GAZEBO_LIBRARIES})
add_library(model_rotate SHARED model_rotate.cc)
target_link_libraries(model_rotate ${GAZEBO_LIBRARIES})
add_library(model_DR SHARED model_DR.cc)
target_link_libraries(model_DR ${GAZEBO_LIBRARIES})
add_library(model_follow SHARED model_follow.cc)
target_link_libraries(model_follow ${GAZEBO_LIBRARIES})
add_library(model_followside SHARED model_followside.cc)
target_link_libraries(model_followside ${GAZEBO_LIBRARIES})
add_library(HydraPlugin SHARED HydraPlugin.cc)
target_link_libraries(HydraPlugin ${GAZEBO_LIBRARIES})



