![ROS2 setup with tooling](https://github.com/Arief-AK/ROS2_Practice/actions/workflows/setup_ros_with_tooling.yml/badge.svg) ![ROS2 setup with custom scripts](https://github.com/Arief-AK/ROS2_Practice/actions/workflows/setup_ros_bare_metal.yml/badge.svg)

# ROS2 Practice
The purpose of this repository is to familiarise and document the practice of using ROS2. This repository follows the [ROS2 Iron Tutorials](https://docs.ros.org/en/iron/Tutorials.html).

## The Quickstart
Clone the git repository into a new workspace by using `git clone` and build the packages with the following command.
```bash
colcon build
```

To ensure that the functionality of the packages are proper, run the following command.
```bash
colcon test
```
If you want to test a specific package, run the following command.
```bash
colcon test --packages-select <package_name> --event-handler=console_direct+
```
This should run the GTest testing suite on the packages.

## GTest with ROS2-Iron
### Objective: Run tests on ROS2 packages

For this demonstration, a ROS2 package called `simple_package` will be created. In this package, the GTest suite will be used to perform unit-tests on the packages. The result of this is found in this [repo](https://github.com/Arief-AK/ROS2_Practice/tree/main).

### Implementation:
1. Create a ROS2 package using
```bash
ros2 pkg create simple_package --build-type ament_cmake --license Apache-2.0
```
2. Head to `package.xml` and insert the following line(s)
```cmake
<depend>std_msgs</depend>

<test_depend>ament_cmake_gtest</test_depend>
```
3. Head to the include directory
```bash
cd ~/ros2_ws/src/simple_package/include/simple_package
```
4. Create the `Queue.h` header file
<details>
    <summary>
    Queue.h
    </summary>

```cpp
#pragma once

#include <iostream>
#include <stdexcept>
#include <gtest/gtest.h>

namespace CustomDataTypes
{
    typedef struct LinkedListNode
    {
        int value;
        struct LinkedListNode *next;
        struct LinkedListNode *prev;

    } LinkedListNode;

    class Queue
    {
    public:

        Queue();
        ~Queue();

        void push_back(int value);
        void push_front(int value);
        void clear();
        
        int pop_back();
        int pop_front();
        LinkedListNode *find_value(int value);
        int size();
        
    private:
        int count;
        LinkedListNode *head;
        LinkedListNode *tail;
    };
}
```
</details>

5. Head to the `src` directory
```
cd ~/ros2_ws/src/simple_package/src
```
6. Create the `Queue.cpp` source file
<details>
    <summary>
    Queue.cpp
    </summary>
    
```cpp
#include "../include/simple_package/Queue.hpp"

using namespace CustomDataTypes;

CustomDataTypes::Queue::Queue()
{
    head = nullptr;
    tail = nullptr;
    count = 0;
}

CustomDataTypes::Queue::~Queue()
{
    delete head;
    delete tail;
}

void CustomDataTypes::Queue::push_back(int value)
{
    LinkedListNode *new_node = new LinkedListNode;
    new_node->value = value;
    new_node->prev = tail;
    new_node->next = nullptr;

    // If queue is empty - new_node becomes head and tail
    if(head == nullptr){
        head = new_node;
    }
    else{
        // Set previous tail to have new_node as next node
        tail->next = new_node;
    }
    
    // Housekeeping
    tail = new_node;
    count++;
}

void CustomDataTypes::Queue::push_front(int value)
{
    LinkedListNode *new_node = new LinkedListNode;
    new_node->value = value;
    new_node->prev = nullptr;
    new_node->next = head;

    // If queue is empty - new_node becomes head and tail
    if(tail == nullptr){
        tail = new_node;
    }
    else{
        // Set previous pointer to current head to become new_node
        head->prev = new_node;
    }
    
    // Housekeeping
    head = new_node;
    count++;
}

void CustomDataTypes::Queue::clear()
{
    // Initialise empty flag
    auto empty = false;

    // If queue is not empty
    while (!empty)
    {
        // Clear the queue
        if(head != nullptr){
            // Pop every element from the front
            auto return_value = pop_front();
        }else{
            empty = true;
        }
    }
}

int CustomDataTypes::Queue::pop_back()
{
    // Get the tail
    auto old_tail = tail;
    auto value = old_tail->value;

    // Set the tail of queue to be the previous node of old tail
    tail = old_tail->prev;
    
    // If queue is not empty - ensure that the next pointer of the current tail is null
    if(tail != nullptr){
        tail->next = nullptr;
    }
    // Else, set both head and tail to null
    else{
        head = nullptr;
    }

    // Housekeeping
    delete old_tail;
    count--;

    // Return
    return value;
}

int CustomDataTypes::Queue::pop_front()
{
    // Get the head
    auto old_head = head;
    auto value = old_head->value;

    // Set the head of queue to be the next node of old head
    head = old_head->next;
    
    // If queue is not empty - ensure that the previous pointer of the current head is null
    if(head != nullptr){
        head->prev = nullptr;
    }
    // Else, set both head and tail to null
    else{
        tail = nullptr;
    }
    
    // Housekeeping
    delete old_head;
    count--;

    // Return
    return value;
}

LinkedListNode *CustomDataTypes::Queue::find_value(int value)
{
    auto found = false;
    auto current_node = head;

    if(current_node == nullptr){
        throw std::runtime_error("Queue is empty");
    }

    // Sequentially check each node
    while (!found){   
        // Compare value
        if(current_node->value == value){
            found = true;
        }else{
            if(current_node->next != nullptr)
            {
                // If not, then move on to the next node
                auto next_node = current_node->next;
                current_node = next_node;
            }else{
                throw std::runtime_error("Value does not exist in Queue");
            }
        }
    }

    // If correct, then return address of the node
    return current_node;
}

int CustomDataTypes::Queue::size()
{
    return count;
}
```
</details>

7. Whilst, we are still in the `src` directory, create the `main.cpp` source file
<details>
    <summary>
    main.cpp
    </summary>
    
```cpp
#include "../include/simple_package/Queue.hpp"

#include <iostream>
#include <gtest/gtest.h>

// Main function - can use terminal arguments as well
int main(){
    std::cout << "Hello, from simple package!\n";
    return 0;
}
```
</details>

8. Now we need to create the tests, create a new `test` directory
```bash
cd ../
mkdir test
cd test
```

9. Create a new test file `basic_tests.cpp`
<details>
    <summary>
    basic_tests.cpp
    </summary>

```cpp
#include <iostream>

#include "../include/simple_package/Queue.hpp"
using namespace CustomDataTypes;

// Queue - Zero Input
TEST(QueueTest, HandlesZeroInput)
{
    // GIVEN: Queue class is called
    auto new_queue = Queue();

    // WHEN: Startup
    // THEN: Result is an empty queue
    EXPECT_EQ(new_queue.size(), 0);
}

// Queue - Multiple Input
TEST(QueueTest, Appending)
{
    // GIVEN: Queue class is called
    auto new_queue = Queue();

    // WHEN: Startup
    // THEN: Appends values to the queue
    new_queue.push_back(10);
    new_queue.push_back(100);
    EXPECT_EQ(new_queue.size(), 2);
}

// Queue - Multiple Removal
TEST(QueueTest, Removing)
{
    // GIVEN: Queue class is called
    auto new_queue = Queue();

    // WHEN: Startup
    // THEN: Appends values to the queue - size should be 2
    new_queue.push_back(1);
    new_queue.push_back(2);
    EXPECT_EQ(new_queue.size(), 2);

    // WHEN: Startup
    // THEN: Removes the values from the queue
    auto popped_value = new_queue.pop_back();
    EXPECT_EQ(popped_value, 2);
    popped_value = new_queue.pop_back();
    EXPECT_EQ(popped_value, 1);
}

// Queue - Find value in queue
TEST(QueueTest, FindValue)
{
    // GIVEN: Queue class is filled with 3 elements
    auto new_queue = Queue();
    new_queue.push_back(1);
    new_queue.push_back(2);
    new_queue.push_back(3);

    // WHEN: Startup
    // THEN: Finds the element in the middle (position 1)
    auto found_node = new_queue.find_value(2);
    EXPECT_EQ(found_node->value, 2);

    // WHEN: Startup
    // THEN: Attempts to find non-existing element in queue
    EXPECT_THROW(new_queue.find_value(4), std::runtime_error );
}

// Queue - Clear the queue
TEST(QueueTest, ClearQueue)
{
    // GIVEN: Queue class is filled with 3 elements
    auto new_queue = Queue();
    new_queue.push_back(1);
    new_queue.push_back(2);
    new_queue.push_back(3);

    // WHEN: Startup
    // THEN: Attempts to clear the queue
    new_queue.clear();
    EXPECT_EQ(new_queue.size(), 0);
}
```    
</details>

10. Also, create a `main.cpp` source file in this directory
<details>
    <summary>
    main.cpp
    </summary>

```cpp
#include <gtest/gtest.h>

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
```    
</details>

11. At this point, we should have all the ingridients required to utilise GTest in this package. It is now time, to link everyting with CMake. Head to the `CMakeLists.txt` file in the main package directory.
<details>
    <summary>
    CMakeLists.txt
    </summary>

```cmake
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

```
</details>

11. Now, everything is linked. Let's build!
```bash
cd ~/ros2_ws
colcon build
```

12. If everything works, a similar output to the following should be shown
```bash
Starting >>> simple_package
Finished <<< simple_package [0.36s]                  

Summary: 1 package finished [1.24s]
```

13. We can now test this package by running the following command
```bash
colcon test --packages-select simple_package --event-handler=console_direct+
```

14. We should get the following output
<details>
    <summary>
    Test output
    </summary>

```bash
Starting >>> simple_package
UpdateCTestConfiguration  from :/home/arief/ros2_ws/build/simple_package/CTestConfiguration.ini
Parse Config file:/home/arief/ros2_ws/build/simple_package/CTestConfiguration.ini
   Site: ARIEF-ROG-G531
   Build name: (empty)
 Add coverage exclude regular expressions.
Create new tag: 20240321-1330 - Experimental
UpdateCTestConfiguration  from :/home/arief/ros2_ws/build/simple_package/CTestConfiguration.ini
Parse Config file:/home/arief/ros2_ws/build/simple_package/CTestConfiguration.ini
Test project /home/arief/ros2_ws/build/simple_package
Constructing a list of tests
Done constructing a list of tests
Updating test list for fixtures
Added 0 tests to meet fixture requirements
Checking test dependency graph...
Checking test dependency graph end
test 1
    Start 1: simple_package_test

1: Test command: /usr/bin/python3.10 "-u" "/opt/ros/iron/share/ament_cmake_test/cmake/run_test.py" "/home/arief/ros2_ws/build/simple_package/test_results/simple_package/simple_package_test.gtest.xml" "--package-name" "simple_package" "--output-file" "/home/arief/ros2_ws/build/simple_package/ament_cmake_gtest/simple_package_test.txt" "--command" "/home/arief/ros2_ws/build/simple_package/simple_package_test" "--gtest_output=xml:/home/arief/ros2_ws/build/simple_package/test_results/simple_package/simple_package_test.gtest.xml"
1: Test timeout computed to be: 60
1: -- run_test.py: invoking following command in '/home/arief/ros2_ws/build/simple_package':
1:  - /home/arief/ros2_ws/build/simple_package/simple_package_test --gtest_output=xml:/home/arief/ros2_ws/build/simple_package/test_results/simple_package/simple_package_test.gtest.xml
1: [==========] Running 5 tests from 1 test suite.
1: [----------] Global test environment set-up.
1: [----------] 5 tests from QueueTest
1: [ RUN      ] QueueTest.HandlesZeroInput
1: [       OK ] QueueTest.HandlesZeroInput (0 ms)
1: [ RUN      ] QueueTest.Appending
1: [       OK ] QueueTest.Appending (0 ms)
1: [ RUN      ] QueueTest.Removing
1: [       OK ] QueueTest.Removing (0 ms)
1: [ RUN      ] QueueTest.FindValue
1: [       OK ] QueueTest.FindValue (0 ms)
1: [ RUN      ] QueueTest.ClearQueue
1: [       OK ] QueueTest.ClearQueue (0 ms)
1: [----------] 5 tests from QueueTest (0 ms total)
1: 
1: [----------] Global test environment tear-down
1: [==========] 5 tests from 1 test suite ran. (0 ms total)
1: [  PASSED  ] 5 tests.
1: -- run_test.py: return code 0
1: -- run_test.py: inject classname prefix into gtest result file '/home/arief/ros2_ws/build/simple_package/test_results/simple_package/simple_package_test.gtest.xml'
1: -- run_test.py: verify result file '/home/arief/ros2_ws/build/simple_package/test_results/simple_package/simple_package_test.gtest.xml'
1/1 Test #1: simple_package_test ..............   Passed    0.07 sec

100% tests passed, 0 tests failed out of 1

Label Time Summary:
gtest    =   0.07 sec*proc (1 test)

Total Test time (real) =   0.07 sec
Finished <<< simple_package [0.19s]

Summary: 1 package finished [0.96s]
```
</details>

You can find the results of this package [here](src/simple_package/)