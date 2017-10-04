#include "mwoibn/visualization_tools/rviz_distinc_links.h"

mwoibn::visualization_tools::RvizDistincLinks::RvizDistincLinks(
    std::string topic, mwoibn::robot_class::Robot& robot,
    std::string robot_topic)
    : RvizCustomModel(topic, robot, robot_topic)
{
  double r = double(246) / 255;
  double g = double(34) / 255;
  double b = double(34) / 255;
  std::vector<RigidBodyDynamics::Joint> mJoints =
      _robot.getModel().mJoints;
  for (int i = 0; i < _robot.getDofs(); i++)
    _map.push_back(-1);

  for (int i = 0; i < _dofs; i++)
  {
    _msg.markers[i].color.r = r;
    _msg.markers[i].color.g = g;
    _msg.markers[i].color.b = b;
    _msg.markers[i].color.a = 0;
    int dof = _robot.getModel().GetBodyId(
        _msg.markers[i].header.frame_id.c_str());
    _map[mJoints[dof].q_index] = i;
  }
}

void mwoibn::visualization_tools::RvizDistincLinks::updateLinks(
    std::vector<std::string> links)
{
  for (int i = 0; i < _dofs; i++)
  {
    auto check =
        std::find(links.begin(), links.end(), _msg.markers[i].header.frame_id);
    if (check != links.end())
      _msg.markers[i].color.a = 1;
    else
      _msg.markers[i].color.a = 0;
  }

  updateModel();
}
void mwoibn::visualization_tools::RvizDistincLinks::updateLinks(std::vector<int> links)
{
  for (int i = 0; i < _dofs; i++)
    _msg.markers[i].color.a = 0;

  for (auto& link : links)
  {
    if (_map[link] != -1)
      _msg.markers[_map[link]].color.a = 1;
  }
}
bool mwoibn::visualization_tools::RvizDistincLinks::updateModel() {}
