#pragma once

#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MinimalSubscriber : public rclcpp::Node
{
    public:
        MinimalSubscriber();

    private:
        void topic_callback(const std_msgs::msg::String & msg) const;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};