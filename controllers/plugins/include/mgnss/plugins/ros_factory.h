#ifndef __MGNSS_PLUGINS_ROS_FACTORY_H
#define __MGNSS_PLUGINS_ROS_FACTORY_H

#include "mgnss/modules/base.h"
#include "mgnss/plugins/ros_base.h"

#include <mwoibn/common/ros_logger.h>
#include <ros/ros.h>
#include <dlfcn.h>

namespace mgnss
{
namespace plugins
{

//! Function to dynamically generate ROS plugins
mgnss::plugins::RosBase* make(std::string name){
        void *hndl = dlopen( ("lib" + name + ".so").c_str(), RTLD_NOW);
        if(hndl == NULL) {
                std::cerr << dlerror() << std::endl;
                std::exit(-1);
        }
        void *mkr = dlsym(hndl, "make");

        return reinterpret_cast<mgnss::plugins::RosBase* (*)()>(mkr)();
      }
}
}
#endif // __MGNSS_PLUGINS_ROS_FACTORY_H
