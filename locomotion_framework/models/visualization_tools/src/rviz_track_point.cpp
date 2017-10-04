#include "mwoibn/visualization_tools/rviz_track_point.h"
#include <type_traits> //for std::underlying_type

int mwoibn::visualization_tools::RvizTrackPoint::initMarker(
    mwoibn::visualization_tools::Utils::TYPE type, std::string frame, double x,
    double y, double z, double pos_x, double pos_y, double pos_z, double or_x,
    double or_y, double or_z, double or_w, std::string resource)
{
  visualization_msgs::Marker marker;
  marker.type = static_cast<
      std::underlying_type<mwoibn::visualization_tools::Utils::TYPE>::type>(
      type);
  marker.action = visualization_msgs::Marker::ADD;
  marker.header.frame_id = frame;
  marker.color.a = 1;
  marker.scale.x = x;
  marker.scale.y = y;
  marker.scale.z = z;
  marker.id = _msg.markers.size();

  if (type == mwoibn::visualization_tools::Utils::TYPE::POINT)
  {
    geometry_msgs::Point point;
    point.x = 0;
    point.y = 0;
    point.z = 0;
    marker.points.push_back(point);
  }
  else
  {
    marker.pose.position.x = pos_x;
    marker.pose.position.y = pos_y;
    marker.pose.position.z = pos_z;
    marker.pose.orientation.x = or_x;
    marker.pose.orientation.y = or_y;
    marker.pose.orientation.z = or_z;
    marker.pose.orientation.w = or_w;
  }
  if (type == mwoibn::visualization_tools::Utils::TYPE::MESH)
  {
    marker.mesh_resource = resource;
    marker.mesh_use_embedded_materials = true;
  }
  _msg.markers.push_back(marker);

  mwoibn::visualization_tools::RvizTrackPoint::setColors();

  return _msg.markers.size() - 1;
}

void mwoibn::visualization_tools::RvizTrackPoint::setColors()
{

  for (int i = 0; i < _msg.markers.size(); i++)
  {
    double factor = i / (double)_msg.markers.size();
    _msg.markers[i].color.b = 1.0 - factor;
    _msg.markers[i].color.r = (factor > 0.5) ? (0.5 + factor) : (0.5 - factor);
    _msg.markers[i].color.g = 0.0 + factor;
  }
}

void mwoibn::visualization_tools::RvizTrackPoint::setColor(int i, double r,
                                                           double g, double b)
{
  _msg.markers.at(i).color.b = b;
  _msg.markers.at(i).color.r = r;
  _msg.markers.at(i).color.g = g;

  return;
}

bool mwoibn::visualization_tools::RvizTrackPoint::updateMarker(int id, double x,
                                                               double y,
                                                               double z)
{

  if (id >= _msg.markers.size())
  {
    LOG_INFO << "Marker not defined in the class, number of markers "
             << _msg.markers.size() << " requested marker nr " << id + 1
             << "\n";
    return false;
  }
  else
  {
    _msg.markers[id].header.stamp = ros::Time::now();

    switch (static_cast<mwoibn::visualization_tools::Utils::TYPE>(
        _msg.markers[id].type))
    {
    case mwoibn::visualization_tools::Utils::TYPE::POINT:
      _msg.markers[id].points[0].x = x;
      _msg.markers[id].points[0].y = y;
      _msg.markers[id].points[0].z = z;
      return true;
    case mwoibn::visualization_tools::Utils::TYPE::LINE:
    {
      geometry_msgs::Point point;
      point.x = x;
      point.y = y;
      point.z = z;
      if (_msg.markers[id].points.size() >= 8000)
        _msg.markers[id].points.erase(_msg.markers[id].points.begin());
      _msg.markers[id].points.push_back(point);
      return true;
    }
    default:
      LOG_INFO << "Unknown type of Marker " << id << "\n";
      break;
    }
  };
}

bool mwoibn::visualization_tools::RvizTrackPoint::reset(int id)
{
  _msg.markers.at(id).points.clear();
}

mwoibn::visualization_tools::RvizTrackPoint::RvizTrackPoint(std::string topic)
{

  _pub = node.advertise<visualization_msgs::MarkerArray>(topic, 10);
}

bool mwoibn::visualization_tools::RvizTrackPoint::closeLine(int id)
{
  if (static_cast<mwoibn::visualization_tools::Utils::TYPE>(
          _msg.markers[id].type) !=
      mwoibn::visualization_tools::Utils::TYPE::LINE)
    return false;

  if (!_msg.markers[id].points.size())
    return false;

  updateMarker(id, _msg.markers[id].points[0].x, _msg.markers[id].points[0].y,
               _msg.markers[id].points[0].z);
  return true;
}
