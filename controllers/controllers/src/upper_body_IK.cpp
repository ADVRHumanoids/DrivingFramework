#include "mgnss/controllers/upper_body_IK.h"
#include <mgnss/higher_level/qp/constraints/minimum_limit.h>
#include <mgnss/higher_level/qp/constraints/maximum_limit.h>
#include <mgnss/higher_level/qp/constraints/integrate.h>



void mgnss::controllers::UpperBodyIK::_setInitialConditions()
{
      for(int i = 0; i < _arms_ptr->points().size(); i++){
        _desried_pos.segment<3>(3*i) = _arms_ptr->points().getPointStateWorld(i);
        _arms_ptr->setReference(i, _desried_pos.segment<3>(3*i));
      }
}

void mgnss::controllers::UpperBodyIK::_createTasks(YAML::Node config){
        if(!config["track"])
              throw std::invalid_argument(std::string("Wheels Controller: configuration doesn't containt required filed 'track'."));

        std::string group_ = config["track"].as<std::string>();
        std::vector<std::string> names = _robot.getLinks(group_);

        // add wheels to the contact group
        for(auto& name: names){
          auto contact = ranges::find_if(_robot.contacts(), [&](auto& contact)-> bool{return _robot.getBodyName(contact->wrench().getBodyId()) == name;});
          if ( contact != ranges::end(_robot.contacts()) )
            _robot.contacts().toGroup((*contact)->getName(), group_);
        }

        // Set-up hierachical controller
        _arms_ptr.reset( new mwoibn::hierarchical_control::tasks::CartesianWorld(
            mwoibn::point_handling::PositionsHandler("ROOT", _robot, names)));
            for(auto group: _robot.getLinks(group_))
              std::cout << group << std::endl;
        _tasks["ARMS"] = _arms_ptr.get();

        auto origin_names = _robot.getLinks(config["arm_base"].as<std::string>());
        for( auto&& [origin, end]:  ranges::view::zip(origin_names, names) ) {
            _body_points.add(mwoibn::robot_points::LinearPoint(origin, _robot));
            _body_points.add(mwoibn::robot_points::LinearPoint(end, _robot));
            _workspace_points.add(mwoibn::robot_points::Minus(_body_points.end(0), _body_points.end(1)) );
            _norms_points.add(mwoibn::robot_points::Norm(_workspace_points.end(0)));
        }
}

void mgnss::controllers::UpperBodyIK::_addConstraints(YAML::Node config, mgnss::higher_level::QrTask& task){
  _arm_workspace.setConstant(1, 0.53);
  for(auto& norm: _norms_points)
    task.hard_inequality.add(mgnss::higher_level::constraints::Integrate(
        mgnss::higher_level::constraints::MaximumLimit(norm->getJacobian(), _arm_workspace), _robot.rate(), norm->get(), false));

}

// void mgnss::controllers::UpperBodyIK::_allocate(){
//
//
