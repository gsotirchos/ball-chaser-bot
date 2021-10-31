cmake_minimum_required(VERSION 3.0.2)
project(ball_chaser)

add_compile_options(-std=c++11)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_service_files(
  FILES
  DriveToTarget.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package()

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(drive_bot src/drive_bot.cpp)
add_executable(process_image src/process_image.cpp)

add_dependencies(drive_bot ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(process_image ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(drive_bot
  ${catkin_LIBRARIES}
)
target_link_libraries(process_image
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

catkin_install_python(PROGRAMS
  scripts/process_image_opencv.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
