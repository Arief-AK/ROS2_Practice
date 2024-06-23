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
        void print();
        
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