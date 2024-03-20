cmake_minimum_required(VERSION 3.8)
project(simple_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)

# Include directories of the headers
include_directories(include/${PROJECT_NAME})

# Add librarie(s)
set(HEADER_FILES include/${PROJECT_NAME}/Queue.hpp)
add_library(dat_struct_lib src/Queue.cpp ${HEADER_FILES})
# ament_target_dependencies(dat_struct_lib ...) - If there are ament dependencies, put them here...

# Testing configurations
if(BUILD_TESTING)
  # Get GTest dependency
  find_package(ament_cmake_gtest REQUIRED)

  # Set the tests files
  set(TEST_FILES
    test/main.cpp
    test/basic_tests.cpp
    # Any other tests...
  )

  # Add GTest executable
  ament_add_gtest(${PROJECT_NAME}_test ${TEST_FILES})
  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(${PROJECT_NAME}_test std_msgs)
  
  # Link the local library
  target_link_libraries(${PROJECT_NAME}_test dat_struct_lib)

  # Install GTest targets
  install(TARGETS
    ${PROJECT_NAME}_test
    DESTINATION lib/${PROJECT_NAME})
endif()

# Add main executable and link local library(s)
add_executable(main src/main.cpp)
target_link_libraries(main dat_struct_lib)

# Install executable targets
install(TARGETS
  main
  DESTINATION lib/${PROJECT_NAME})

ament_package()
