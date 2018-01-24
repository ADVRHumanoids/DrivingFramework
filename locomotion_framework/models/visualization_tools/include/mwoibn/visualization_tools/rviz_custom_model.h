#ifndef VISUALIZATION_TOOLS_RVIS_CUSTOM_MODEL_H
#define VISUALIZATION_TOOLS_RVIS_CUSTOM_MODEL_H

#include <vector>
#include <ros/ros.h>
#include "mwoibn/visualization_tools/rviz_track_point.h"

#include "mwoibn/robot_class/robot.h"
#include <urdf/model.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace mwoibn
{
namespace visualization_tools
{

//! Facilitate the visualization of a tracked points and/or the trajectories
/**
 *
 * \todo add tracker names to facilitate usage
 * \todo add support for updating the marker by Eigen/RBDL vector
 * \todo add some legend generation (how to provide colors(?))
 * \todo add an option to manually set-up colors
 *
 */
class RvizCustomModel : public RvizTrackPoint
{
public:
  RvizCustomModel(std::string topic, mwoibn::robot_class::Robot& robot,
                  std::string robot_topic = "/robot_description")
      : _robot(robot), RvizTrackPoint(topic)
  {
    _readMesh(robot_topic);
  }
  virtual ~RvizCustomModel() {}
  virtual bool updateModel() {}
  double getDofs() { return _dofs; }

protected:
  mwoibn::robot_class::Robot& _robot;
  virtual bool _readMesh(std::string robot_topic);
  double _dofs;
};

} // namespace package
} // namespace library
#endif
