#ifndef COMMUNICATION_MODULES_ALL_H
#define COMMUNICATION_MODULES_ALL_H


//#define ROS_CONTROL
#include "mwoibn/communication_modules/config.h"

#ifdef ROS_CONTROL
  #include "mwoibn/communication_modules/custom_controller.h"
  #include "mwoibn/communication_modules/velocity_controller.h"
  #include <custom_controller/controller_utils.h>
#endif

#include "mwoibn/communication_modules/ros_controller.h"
#include "mwoibn/communication_modules/ros_feedback.h"
#include "mwoibn/communication_modules/ros_operational_euler.h"
#include "mwoibn/communication_modules/shared_controller.h"
#include "mwoibn/communication_modules/shared_feedback.h"
#endif
