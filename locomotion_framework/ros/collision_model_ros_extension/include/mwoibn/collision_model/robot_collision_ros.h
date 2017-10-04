#ifndef COLLISION_MODEL_ROBOT_COLLISION_ROS_H
#define COLLISION_MODEL_ROBOT_COLLISION_ROS_H

#include <ros/ros.h>
#include <sch/STP-BV/STP_BV.h>
#include <ros/console.h>
#include <sch/S_Object/S_Sphere.h>
#include <sch/S_Object/S_Box.h>
#include "mwoibn/collision_model/robot_collision.h"

//#include <gazebo_msgs/LinkStates.h>
#include <rbdl/rbdl.h>
#include "mwoibn/robot_class/robot.h"
#include <ros/package.h>

namespace mwoibn{
namespace collision_model
{

//! The class provides the ros implementation to initializae robot collision with use of ros parameters
//model
class RobotCollisionRos : public RobotCollision
{

public:
  RobotCollisionRos(mwoibn::robot_class::Robot& robot,
                    std::string urdf_topic = "/robot_description",
                    std::string srdf_topic = "/robot_semantic_description",
                    std::string pkg = "")
      : RobotCollision(robot)
  {

    ros::NodeHandle node;
    std::string srdf_file;
    std::string urdf_file;
    std::string package_path = "";

    if (!node.getParam(urdf_topic, urdf_file))
      throw std::invalid_argument(
          std::string("Could not retrieve a robot_description - a parameter ") +
          urdf_topic + std::string(" from parameter server."));

    if (!node.getParam(srdf_topic, srdf_file))
      LOG_INFO << "didn't find a srdf desription in the parameter "
               << srdf_topic << " consider all pair valid "
               << std::endl; // warning

    if (!pkg.empty())
      try
      {
        package_path = ros::package::getPath(pkg);
      }
      catch (...)
      {
        LOG_INFO << "could not find a path to package " << pkg
                 << std::endl; // warning
      }

    _readMesh(urdf_file, srdf_file, package_path);

    _dof = _objects.size();
    _pairs_number = _pairs.size();

    updatePositions();
  }

  virtual ~RobotCollisionRos() {}
  //bool valid = false; // TODO remove
};

} // namespace package
} // namespace library
#endif
