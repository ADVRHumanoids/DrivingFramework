#include "mgnss/controllers/upper_body_IK.h"



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

}


// void mgnss::controllers::UpperBodyIK::_allocate(){
//
// }
