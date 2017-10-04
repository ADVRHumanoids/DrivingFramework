#ifndef VISUALIZATION_TOOLS_RVIS_TRACK_POINT_H
#define VISUALIZATION_TOOLS_RVIS_TRACK_POINT_H

#include <ros/ros.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "mwoibn/visualization_tools/visualization_tools.h"
#include "mwoibn/visualization_tools/visualization_tools.h"
#include <rbdl/rbdl.h>

namespace mwoibn{
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
class RvizTrackPoint
{
public:
  RvizTrackPoint(std::string topic);
  virtual ~RvizTrackPoint(){}
  int initMarker(Utils::TYPE type = Utils::TYPE::POINT, std::string frame = "pelvis",
                 double x = 0.01, double y = 0.01, double z = 0.01,
                 double pos_x = 0.0, double pos_y = 0.0, double pos_z = 0.0,
                 double or_x = 0.0, double or_y = 0.0, double or_z = 0.0,
                 double or_w = 1.0, std::string resource = "");
  bool updateMarker(int id, double x, double y, double z);
  bool updateMarker(int id, mwoibn::Vector3 new_point){
    updateMarker(id, new_point[0], new_point[1], new_point[2]);
  }

  void publish() {
    _pub.publish(_msg); }
  void setColor(int i, double r, double g, double b);
  bool reset(int id);
  bool closeLine(int id);

protected:
  ros::NodeHandle node;

  ros::Publisher _pub;
  visualization_msgs::MarkerArray _msg;
  void setColors();
};
} // namespace package
} // namespace library
#endif
