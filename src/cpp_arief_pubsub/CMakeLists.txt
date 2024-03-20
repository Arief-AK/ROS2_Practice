cmake_minimum_required(VERSION 3.8)
project(cpp_arief_pubsub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# include the directories for the headers
include_directories(include/${PROJECT_NAME})

# Add library
set(HEADER_FILES include/${PROJECT_NAME}/Publisher.hpp include/${PROJECT_NAME}/Subscriber.hpp)
add_library(pubsub_lib src/Publisher.cpp src/Subscriber.cpp ${HEADER_FILES})
ament_target_dependencies(pubsub_lib rclcpp std_msgs)

if(BUILD_TESTING)
  # Get GTest dependency
  find_package(ament_cmake_gtest REQUIRED)

  # Set the test files
  set(TESTFILES
    test/main.cpp
    test/pub_test.cpp
  )

  # Add GTest executable
  ament_add_gtest(${PROJECT_NAME}_test ${TESTFILES})
  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(${PROJECT_NAME}_test rclcpp std_msgs)
  target_link_libraries(${PROJECT_NAME}_test pubsub_lib)

  # Install GTest targets
  install(TARGETS
    ${PROJECT_NAME}_test
    DESTINATION lib/${PROJECT_NAME})
endif()

# Add talker executable
add_executable(talker src/talker.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

# Add listener executable
add_executable(listener src/listener.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

# Add main executable
add_executable(main src/main.cpp)
target_link_libraries(main pubsub_lib)

# Install targets
install(TARGETS
  main
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})

ament_package()