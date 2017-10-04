#ifndef TEST_COLLISION_H
#define TEST_COLLISION_H

#include "mwoibn/tests_common/test.h"
#include "mwoibn/collision_model/robot_collision.h"
#include "mwoibn/robot_class/robot.h"

namespace mwoibn {

namespace tests_common {

inline std::unique_ptr<mwoibn::collision_model::RobotCollision>
_initTestCollisionModel(mwoibn::robot_class::Robot& robot)
{
  std::unique_ptr<mwoibn::collision_model::RobotCollision> collision_ptr;

  std::string path = ros::package::getPath("tests_common");
  std::string file_name =
      path + "/resources/urdf/centauro.urdf"; // load test urdf file

  std::ifstream myurdf(file_name);
  if (!myurdf.is_open())
    throw std::invalid_argument("Couldn't read test urdf file, abort.");

  std::string urdf_file(static_cast<std::stringstream const&>(
                            std::stringstream() << myurdf.rdbuf()).str());
  myurdf.close();

  // load test srdf file
  file_name = path + "/resources/srdf/centauro.srdf";
  std::string srdf_file = "";
  std::ifstream mysrdf(file_name);
  if (!mysrdf.is_open())
    throw std::invalid_argument("Couldn't read test srdf file, abort.");

  srdf_file = static_cast<std::stringstream const&>(std::stringstream()
                                                    << mysrdf.rdbuf()).str();
  mysrdf.close();

  collision_ptr.reset(new mwoibn::collision_model::RobotCollision(robot, urdf_file,
                                                          srdf_file, ros::package::getPath("centauro")));

  return std::move(collision_ptr);
}

} // namespace package
} // namespace library

#endif // TEST_COLLISION_H
