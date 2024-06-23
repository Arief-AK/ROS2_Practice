#include "../include/simple_package/Queue.hpp"

#include <iostream>
#include <gtest/gtest.h>

int main(){
    std::cout << "Hello, from simple package!\n";
    
    // Create a queue
    auto new_queue = CustomDataTypes::Queue();

    // Push elements into the queue
    std::cout << "Push elements 10 and 100 into queue\n";
    new_queue.push_back(10);
    new_queue.push_back(100);
    new_queue.print();
    std::cout << std::endl;

    // Push elements into the front and back of the queue
    std::cout << "Push elements 0 and 1000 into the front and back of the queue respectively\n";
    new_queue.push_front(0);
    new_queue.push_back(1000);
    new_queue.print();
    std::cout << std::endl;

    // Find the value 100
    std::cout << "Attempting to find 100";
    auto value_found = new_queue.find_value(100);
    std::cout << "\nFound value: " << value_found->value << std::endl;
    std::cout << std::endl;

    // Pop front and back
    std::cout << "Pop front and back elements\n";
    new_queue.pop_front();
    new_queue.pop_back();
    new_queue.print();
    std::cout << std::endl;

    // Clear the queue
    std::cout << "Clear all elements\n";
    new_queue.clear();
    new_queue.print();
    std::cout << std::endl;

    return 0;
}
