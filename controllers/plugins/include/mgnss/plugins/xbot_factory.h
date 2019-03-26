#ifndef __MGNSS_PLUGINS_XBOT_FACTORY_H
#define __MGNSS_PLUGINS_XBOT_FACTORY_H

#include "mgnss/modules/base.h"
#include "mgnss/plugins/xbot_base_v2.h"
//#include <XCM/XBotControlPlugin.h>

#include <mwoibn/common/xbot_logger.h>
#include <dlfcn.h>

namespace mgnss
{
namespace plugins
{

mgnss::plugins::XbotBaseUnify* xbot_make(std::string name){
        void *hndl = dlopen( ("lib" + name + ".so").c_str(), RTLD_NOW);
        if(hndl == NULL) {
                std::cerr << dlerror() << std::endl;
                std::exit(-1);
        }
        void *mkr = dlsym(hndl, "create_instance");

        return reinterpret_cast<mgnss::plugins::XbotBaseUnify* (*)()>(mkr)();
        //return static_cast<mgnss::plugins::RosBase *()>(mkr)();
      }
}
}
#endif // RT_MY_TEST_H
