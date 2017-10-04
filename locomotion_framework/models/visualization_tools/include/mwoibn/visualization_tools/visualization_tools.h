#ifndef VISUALIZATION_TOOLS_VISUALIZATION_TOOLS_H
#define VISUALIZATION_TOOLS_VISUALIZATION_TOOLS_H
#include "mwoibn/simple_log/log.h"
#include "mwoibn/common/types.h"

namespace mwoibn {

namespace visualization_tools{

const std::string PACKAGE  = "visualization_tools";

namespace Utils{
	
enum class TYPE {POINT = visualization_msgs::Marker::POINTS, LINE = visualization_msgs::Marker::LINE_STRIP, BOX = visualization_msgs::Marker::CUBE,  CYLINDER = visualization_msgs::Marker::CYLINDER, SPHERE = visualization_msgs::Marker::SPHERE, MESH = visualization_msgs::Marker::MESH_RESOURCE};

}

} // namespace package
} // namespace library
#endif
