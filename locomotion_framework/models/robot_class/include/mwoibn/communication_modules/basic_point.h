#ifndef __MWOBIN_COMMUNICATION_MODULES__BASIC_POINT_H
#define __MWOBIN_COMMUNICATION_MODULES__BASIC_POINT_H

#include "mwoibn/common/all.h"
#include "mwoibn/point_handling/wrench.h"
#include "mwoibn/robot_class/map.h"
#include "mwoibn/filters/iir_second_order.h"

#include <rbdl/rbdl.h>

namespace mwoibn
{

namespace communication_modules
{

class BasicPoint
{

public:
  BasicPoint(mwoibn::point_handling::Wrench& point)
      : _wrench(point)
  {
  }

  BasicPoint(mwoibn::point_handling::Wrench& point, YAML::Node config)
      : _wrench(point)
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

  virtual bool update() = 0;


  virtual bool reset(){return true;}
  virtual bool initialized(){return _initialized;}
  virtual bool initialize(){
    _initialized = true;
    return initialized();}

protected:
  mwoibn::point_handling::Wrench& _wrench;
  std::unique_ptr<mwoibn::filters::IirSecondOrder> _linear_filter_ptr, _angular_filter_ptr;

  bool _filter;
  bool _initialized = false;
};
}
}

#endif // COMMUNICATION_MODULE_H
