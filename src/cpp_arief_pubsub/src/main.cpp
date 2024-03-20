#include <cstring>
#include "../include/cpp_arief_pubsub/Publisher.hpp"
#include "../include/cpp_arief_pubsub/Subscriber.hpp"

int main(int argc, char ** argv)
{
  // Check for command-line arguments
  auto i = 0;
  auto found = false;

  while (i < argc)
  {
    if(strcmp(argv[i], "-talker") == 0){
      std::cout << "Publisher running..." << std::endl;
      rclcpp::init(argc, argv);
      rclcpp::spin(std::make_shared<MinimalPublisher>());
      rclcpp::shutdown();
      found = true;
    }else if (strcmp(argv[i], "-listener") == 0)
    {
      std::cout << "Subscriber running..." << std::endl;
      rclcpp::init(argc, argv);
      rclcpp::spin(std::make_shared<MinimalSubscriber>());
      rclcpp::shutdown();
      found = true;
    }
    i++;
  }

  if(!found){
    std::cout << "Please enter -talker or -listener argument" << std::endl;
  }

  return 0;
}