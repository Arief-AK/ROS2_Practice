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
