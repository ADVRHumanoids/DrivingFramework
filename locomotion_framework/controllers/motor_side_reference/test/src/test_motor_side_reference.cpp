#include "mwoibn/tests_common/test.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{

  ros::init(argc, argv, "motor_side_reference_test"); // initalize node

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
