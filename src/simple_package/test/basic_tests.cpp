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