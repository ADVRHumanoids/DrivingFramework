#ifndef __MWOIBN_HIERARCHICAL_CONTROL_TASKS_CATRESIAN_SIMPLIFIED_PELVIS_8_H
#define __MWOIBN_HIERARCHICAL_CONTROL_TASKS_CATRESIAN_SIMPLIFIED_PELVIS_8_H

#include "mwoibn/hierarchical_control/tasks/contact_point_3D_rbdl_task.h"
#include "mwoibn/hierarchical_control/tasks/center_of_mass_task.h"
#include "mwoibn/robot_points/point.h"
#include "mwoibn/robot_points/ground_wheel.h"

namespace mwoibn
{
namespace hierarchical_control
{
namespace tasks
{


/**
 * @brief Uses the torus model
 */
class CartesianFlatReference5 : public ContactPoint3DRbdl
{

public:
/**
 * @param[in] ik the point handler mamber that defines which point is
 **controlled by this task instance it makes a local copy of a point handler to
 **prevent outside user from modifying a controlled point
 *
 */
CartesianFlatReference5(std::vector<std::string> names, mwoibn::robot_class::Robot& robot, YAML::Node config,
                        mwoibn::robot_points::Point& base_point, std::string base_link)
        : ContactPoint3DRbdl(robot, base_point, base_link)
{

    for(auto& contact: _robot.contacts())
    {
        std::string name = _robot.getBodyName(contact->wrench().getBodyId());
        if(!std::count(names.begin(), names.end(), name)){
          std::cout << "Tracked point " << name << " could not be initialized" << std::endl;
          names.erase(std::remove(names.begin(), names.end(), name), names.end());
          continue;
        }

        std::unique_ptr<mwoibn::robot_points::TorusModel> torus_(new mwoibn::robot_points::TorusModel(
                           _robot.getModel(), _robot.state, mwoibn::point_handling::FramePlus(name,
                           _robot.getModel(), _robot.state),
                           mwoibn::Axis(config["reference_axis"][name]["x"].as<double>(),
                                        config["reference_axis"][name]["y"].as<double>(),
                                        config["reference_axis"][name]["z"].as<double>()),
                                        config["minor_axis"].as<double>(), config["major_axis"].as<double>(),
                                        contact->getGroundNormal()));

        _wheel_transforms.push_back(std::unique_ptr<mwoibn::robot_points::Rotation>(
                  new mwoibn::robot_points::GroundWheel(torus_->axis(), torus_->groundNormal())));
        _contacts.add(std::move(torus_));

    }

        _allocate();
        init();
}


virtual ~CartesianFlatReference5() {
}

};
}
} // namespace package
} // namespace library
#endif
