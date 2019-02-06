#ifndef __MWOBIN_COMMUNICATION_MODULES__BASIC_POINT_H
#define __MWOBIN_COMMUNICATION_MODULES__BASIC_POINT_H

#include "mwoibn/common/all.h"
#include "mwoibn/point_handling/wrench.h"
#include "mwoibn/robot_class/map.h"
#include "mwoibn/filters/iir_second_order.h"
#include "mwoibn/communication_modules/communication_base.h"
// #include <rbdl/rbdl.h>

namespace mwoibn
{

namespace communication_modules
{

class BasicPoint: public CommunicationBase
{

public:
  BasicPoint(BasicPoint& other)
      : CommunicationBase(other), _point(other._point)
  {
  }
  BasicPoint(BasicPoint&& other)
      : CommunicationBase(other), _point(other._point)
  {
  }
  BasicPoint(mwoibn::point_handling::State& point, YAML::Node config)
      : CommunicationBase(point.size()), _point(point)
  {

    if(config["filter"]){
      if(!config["filter"]["run"])
        throw(std::invalid_argument("Missing required filter parameter: run"));

      _filter = config["filter"]["run"].as<bool>();

      if(_filter){
        if(!config["filter"]["frequency"])
          throw(std::invalid_argument("Missing required filter parameter: frequency"));
        if(!config["filter"]["damping"])
          throw(std::invalid_argument("Missing required filter parameter: damping"));

        _linear_filter_ptr.reset(new mwoibn::filters::IirSecondOrder(3, config["filter"]["frequency"].as<double>(), config["filter"]["damping"].as<double>()));
        _angular_filter_ptr.reset(new mwoibn::filters::IirSecondOrder(3, config["filter"]["frequency"].as<double>(), config["filter"]["damping"].as<double>()));

        std::cout << "\tFilter has been enabled. Cut-off frequency " << config["filter"]["frequency"] << ", damping " << config["filter"]["damping"] << "." << std::endl;
      }
    }
    else
      _filter = false;

  }

  virtual ~BasicPoint() {}

  virtual bool update() { run(); return true;}

  virtual mwoibn::VectorInt map() const {
    return mwoibn::eigen_utils::iota(6);
  }


  virtual bool reset(){return true;}
  virtual bool initialized(){return _initialized;}
  virtual bool initialize(){
    _initialized = true;
    return initialized();}

protected:
  mwoibn::point_handling::State& _point;
  std::unique_ptr<mwoibn::filters::IirSecondOrder> _linear_filter_ptr, _angular_filter_ptr;

  bool _filter;
  bool _initialized = false;
};
}
}

#endif // COMMUNICATION_MODULE_H
