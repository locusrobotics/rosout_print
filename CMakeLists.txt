cmake_minimum_required(VERSION 3.16.3)
project(rosout_print)

find_package(catkin REQUIRED COMPONENTS
  rosbag
  rosbag_manager_msgs
  roscpp
  rosgraph_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  CATKIN_DEPENDS roscpp
)

###########
## Build ##
###########

catkin_add_env_hooks(
  50.rosout_print
  SHELLS sh
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_compile_options(-std=c++17;-Wall;-Wextra)

add_executable(rosout_print src/rosout_print.cpp src/process_rosout.cpp)

target_link_libraries(rosout_print
  boost_program_options
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS rosout_print
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

#############
## Testing ##
#############
if (CATKIN_ENABLE_TESTING)
  find_package(catkin_clang_format)
  catkin_clang_lint_cpp()
  catkin_clang_format_cpp()
  catkin_clang_lint_add_test()
endif()


if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(catkin REQUIRED COMPONENTS rosbag)

  catkin_add_nosetests(tests)

  add_rostest_gtest(test_rosout_print tests/rosout_print.test
    tests/test_rosout_print.cpp src/process_rosout.cpp)
  target_link_libraries(test_rosout_print ${catkin_LIBRARIES})

endif()
