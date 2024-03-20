#include <gtest/gtest.h>

#include "include/cpp_arief_pubsub/Publisher.hpp"
#include "include/cpp_arief_pubsub/Subscriber.hpp"

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}