#ifndef RVIZDISTINCLINKS_H
#define RVIZDISTINCLINKS_H

#include "mwoibn/visualization_tools/rviz_custom_model.h"

namespace mwoibn{
namespace visualization_tools
{

class RvizDistincLinks : public RvizCustomModel
{
public:
  RvizDistincLinks(std::string topic,
                   mwoibn::robot_class::Robot& robot,
                   std::string robot_topic = "/robot_description");
  virtual ~RvizDistincLinks(){}
  void updateLinks(std::vector<int> links);
  void updateLinks(std::vector<std::string> links);

  virtual bool updateModel();


protected:
  std::vector<int> _map;
};

} // namespace package
} // namespace library
#endif // RVIZDISTINCLINKS_H
