#include "mwoibn/visualization_tools/rviz_custom_model.h"

bool mwoibn::visualization_tools::RvizCustomModel::_readMesh(
    std::string robot_topic)
{
  urdf::Model urdf;

  if (!urdf.initParam(robot_topic))
  {
    LOG_INFO << "Failed to parse urdf file";
    return false;
  }

  typedef std::vector<boost::shared_ptr<urdf::Link>> V_Link;
  V_Link links;
  urdf.getLinks(links);
  std::vector<RigidBodyDynamics::Joint> mJoints =
      _robot.getModel().mJoints;

  for (int i = 0; i < links.size(); i++)
  {

    if (links[i]->collision == NULL)
    {
      LOG_INFO << links[i]->name << " doesn't have a collision element"
               << "\n";
      continue;
    }

    boost::shared_ptr<urdf::Collision> col = links[i]->collision;
    LOG_INFO << links[i]->name << "\n";
    boost::shared_ptr<urdf::Geometry> gm = col->geometry;
    urdf::Vector3 u_position = col->origin.position;
    urdf::Rotation u_orientation = col->origin.rotation;
    ++_dofs;
    switch (gm->type)
    {
    case urdf::Geometry::BOX:
    {
      boost::shared_ptr<urdf::Box> p =
          boost::static_pointer_cast<urdf::Box>(gm);
      urdf::Vector3 vc = p->dim;
      initMarker(mwoibn::visualization_tools::Utils::TYPE::BOX, links[i]->name,
                 p->dim.x, p->dim.y, p->dim.z, u_position.x, u_position.y,
                 u_position.z, u_orientation.x, u_orientation.y,
                 u_orientation.z, u_orientation.w);
      break;
    }
    case urdf::Geometry::CYLINDER:
    {
      boost::shared_ptr<urdf::Cylinder> p =
          boost::static_pointer_cast<urdf::Cylinder>(gm);
      initMarker(mwoibn::visualization_tools::Utils::TYPE::CYLINDER,
                 links[i]->name, p->radius, p->radius, p->length, u_position.x,
                 u_position.y, u_position.z, u_orientation.x, u_orientation.y,
                 u_orientation.z, u_orientation.w);
      break;
    }
    case urdf::Geometry::SPHERE:
    {
      boost::shared_ptr<urdf::Sphere> p =
          boost::static_pointer_cast<urdf::Sphere>(gm);
      initMarker(mwoibn::visualization_tools::Utils::TYPE::SPHERE,
                 links[i]->name, p->radius, p->radius, p->radius, u_position.x,
                 u_position.y, u_position.z, u_orientation.x, u_orientation.y,
                 u_orientation.z, u_orientation.w);
      break;
    }
    case urdf::Geometry::MESH:
    {
      boost::shared_ptr<urdf::Mesh> p =
          boost::static_pointer_cast<urdf::Mesh>(gm);
      initMarker(mwoibn::visualization_tools::Utils::TYPE::MESH, links[i]->name,
                 p->scale.x, p->scale.y, p->scale.z, u_position.x, u_position.y,
                 u_position.z, u_orientation.x, u_orientation.y,
                 u_orientation.z, u_orientation.w, p->filename);
      break;
    }
    default:
      LOG_INFO << "unknown type" << links[i]->name << "\n";
      --_dofs;
      break;
    }
  }

//  LOG_INFO << "markers, making markers" << std::endl;
  return true;
}
