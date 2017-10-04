#ifndef VISULIZTION_TOOLS_RVIZ_JOINT_LIMITS_H
#define VISULIZTION_TOOLS_RVIZ_JOINT_LIMITS_H

#include "mwoibn/visualization_tools/rviz_custom_model.h"

namespace mwoibn
{
namespace visualization_tools
{

class RvizJointLimits : public RvizCustomModel
{
public:
  RvizJointLimits(std::string topic, mwoibn::robot_class::Robot& robot,
                  std::string robot_topic = "/robot_description")
      : RvizCustomModel(topic, robot, robot_topic)
  {
    std::tuple<double, double, double, double> alert_level;
    //! far from limits, green color
    std::get<0>(alert_level) = 0.8;
    std::get<1>(alert_level) = double(50) / 255;
    std::get<2>(alert_level) = double(205) / 255;
    std::get<3>(alert_level) = double(50) / 255;
    _alert_level.push_back(alert_level);
    //! low alert level from limits, yellow color
    std::get<0>(alert_level) = 0.9;
    std::get<1>(alert_level) = double(255) / 255;
    std::get<2>(alert_level) = double(255) / 255;
    std::get<3>(alert_level) = double(102) / 255;
    _alert_level.push_back(alert_level);
    //! medium alert level from limits, orange color
    std::get<0>(alert_level) = 0.95;
    std::get<1>(alert_level) = double(255) / 255;
    std::get<2>(alert_level) = double(140) / 255;
    std::get<3>(alert_level) = double(0) / 255;
    _alert_level.push_back(alert_level);
    //! high alert level from limits, red color
    std::get<0>(alert_level) = 1.001;
    std::get<1>(alert_level) = double(246) / 255;
    std::get<2>(alert_level) = double(34) / 255;
    std::get<3>(alert_level) = double(34) / 255;
    _alert_level.push_back(alert_level);
    //! behind the cross limits, black color
    std::get<0>(alert_level) = 200;
    std::get<1>(alert_level) = double(255) / 255;
    std::get<2>(alert_level) = double(255) / 255;
    std::get<3>(alert_level) = double(255) / 255;
    _alert_level.push_back(alert_level);

    urdf::Model urdf;

    if (!urdf.initParam(robot_topic))
    {
      LOG_INFO << "Failed to parse urdf file";
      return;
    }
    typedef std::vector<boost::shared_ptr<urdf::Link>> V_Link;
    V_Link links;
    urdf.getLinks(links);

    std::tuple<int, double, double> limits;
    std::string name;
    // only 1 DOF links are supported, add warning message

    _msg.markers.erase(std::remove_if(_msg.markers.begin(), _msg.markers.end(),
                   [this](visualization_msgs::Marker marker)
                   {

                     return (_robot.getDof(marker.header.frame_id).size() != 1);
                   }));

    for (auto& marker : _msg.markers)
    {
      name = marker.header.frame_id;
      auto link_ptr = std::find_if(links.begin(), links.end(),
                                   [&name](boost::shared_ptr<urdf::Link> link)
                                   {
                                     return link->name == name;
                                   });
      if (link_ptr == links.end())
      { // there is something really wrong if this happend
        std::stringstream errMsg;
        errMsg << "This should not be posiible";
        throw(std::invalid_argument(
            errMsg.str()
                .c_str())); // I don't know which exception would be the best
      }

      std::get<0>(limits) = _robot.getDof(name)[0];
      std::get<1>(limits) = (*link_ptr)->parent_joint->limits->lower;
      std::get<2>(limits) = (*link_ptr)->parent_joint->limits->upper;

      _limits.push_back(limits);
    }
  }
  virtual ~RvizJointLimits() {}
  virtual bool updateModel();

protected:
  std::vector<std::tuple<double, double, double, double>> _alert_level;
  std::vector<std::tuple<int, double, double>> _limits; // <q_index from
                                                        // rbdl, lowwer
                                                        // limit, upper
                                                        // limit,
                                                        // custom_model
                                                        // ref>
};
} // namespace package
} // namespace library
#endif // RVIZ_JOINT_LIMITS_H
