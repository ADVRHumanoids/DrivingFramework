#ifndef __MGNSS_ROS_PLUGINS_WHEELS_CONTROLLERS_H
#define __MGNSS_ROS_PLUGINS_WHEELS_CONTROLLERS_H

#include "mgnss/plugins/generator.h"

#include "mgnss/controllers/wheels_controller_extend.h"

#include "mgnss/controllers/wheeled_motion_event_v3.h"
#include "mgnss/controllers/wheels_zmp.h"
#include "mgnss/controllers/wheels_zmp_II.h"
#include "mgnss/controllers/wheels_zmp_III.h"
#include "mgnss/controllers/wheels_zmp_IV.h"

#include "mgnss/controllers/wheels_second_order.h"

#include "mgnss/controllers/wheels_reactif.h"
#include "mgnss/controllers/wheeled_motion_world.h"
//#include "mgnss/controllers/wheeled_motion_actions.h"
#include "mgnss/controllers/wheeled_motion_merge_v1.h"

//#include "mgnss/ros_callbacks/wheels_zmp.h"

//#include "mgnss/ros_callbacks/wheels_controller_extend.h"
#include "mgnss/ros_callbacks/wheels_controller_events.h"
#include "mgnss/ros_callbacks/wheels_controller_second_order.h"
#include "mgnss/ros_callbacks/wheels_controller_zmp.h"

//#include "mgnss/ros_callbacks/wheels_controller_actions.h"
//#include "mgnss/ros_callbacks/wheels_controller_merge_v1.h"


namespace mgnss {
namespace nrt_software {
namespace plugins {

  template<typename Subscriber, typename Service, typename Node, typename Publisher>
  class WheelsControllerExtend : public mgnss::plugins::Generator<Subscriber, Service, Node, Publisher>
  {
    typedef mgnss::plugins::Generator<Subscriber, Service, Node, Publisher> Generator_;

  public:
    WheelsControllerExtend() : Generator_("wheeled_motion"){
    }


    mgnss::controllers::WheelsControllerExtend& get(){
        return static_cast<mgnss::controllers::WheelsControllerExtend&>(*Generator_::controller_ptr);
    }
    mwoibn::robot_class::Robot& robot(){
            return *Generator_::_robot_ptr.begin()->second;
    }

    virtual ~WheelsControllerExtend(){
    }

    protected:
    virtual void _initCallbacks(YAML::Node config){
      Generator_::_srv.push_back(Generator_::n->template advertiseService<custom_services::updatePDGains::Request,
                                      custom_services::updatePDGains::Response>("wheels_command", boost::bind(&mgnss::ros_callbacks::wheels_controller_events::eventsHandler,  _1, _2, static_cast<mgnss::controllers::WheelsControllerExtend*>(Generator_::controller_ptr.get()))));

      Generator_::_sub.push_back(Generator_::n->template subscribe<custom_messages::CustomCmnd>("wheels_support", 1, boost::bind(&mgnss::ros_callbacks::wheels_controller_events::supportHandler,_1, static_cast<mgnss::controllers::WheelsControllerExtend*>(Generator_::controller_ptr.get()))));

      Generator_::_sub.push_back(Generator_::n->template subscribe<custom_messages::StateMsg>("wheels_state", 1, boost::bind(&mgnss::ros_callbacks::wheels_controller_events::stateHandler,_1, static_cast<mgnss::controllers::WheelsControllerExtend*>(Generator_::controller_ptr.get()))));

        // Generator_::_srv.push_back(Generator_::n->template advertiseService<custom_services::updatePDGains::Request,
        //                                custom_services::updatePDGains::Response>("wheels_command", boost::bind(&mgnss::ros_callbacks::wheels_controller_extend::eventsHandler,_1, _2, static_cast<mgnss::controllers::WheelsControllerExtend*>(Generator_::controller_ptr.get()))));
        //
        // Generator_::_sub.push_back(Generator_::n->template subscribe<custom_messages::CustomCmnd>("wheels_support", 1, boost::bind(&mgnss::ros_callbacks::wheels_controller_extend::supportHandler,_1, static_cast<mgnss::controllers::WheelsControllerExtend*>(Generator_::controller_ptr.get()))));
    }

    };



    template<typename Subscriber, typename Service, typename Node, typename Publisher>
    class WheelsSecondOrder : public mgnss::plugins::Generator<Subscriber, Service, Node, Publisher>
    {
      typedef mgnss::plugins::Generator<Subscriber, Service, Node, Publisher> Generator_;

      public:
        WheelsSecondOrder() : Generator_("wheeled_motion"){}

        mgnss::controllers::WheelsSecondOrder& get(){
            return static_cast<mgnss::controllers::WheelsSecondOrder&>(*Generator_::controller_ptr);
        }
        mwoibn::robot_class::Robot& robot(){
                return *Generator_::_robot_ptr.begin()->second;
        }


      protected:
        virtual void _resetPrt(YAML::Node config){
              Generator_::controller_ptr.reset(new mgnss::controllers::WheelsSecondOrder(*Generator_::_robot_ptr.begin()->second, config));
            }

        virtual void _initCallbacks(YAML::Node config){
                Generator_::_srv.push_back(Generator_::n->template advertiseService<custom_services::updatePDGains::Request,
                                                custom_services::updatePDGains::Response>("wheels_command", boost::bind(&mgnss::ros_callbacks::wheels_controller_second_order::eventsHandler,
                                                    _1, _2, static_cast<mgnss::controllers::WheelsSecondOrder*>(Generator_::controller_ptr.get()))));

                Generator_::_sub.push_back(Generator_::n->template subscribe<custom_messages::CustomCmnd>("wheels_support", 1, boost::bind(&mgnss::ros_callbacks::wheels_controller_second_order::supportHandler,
                  _1, static_cast<mgnss::controllers::WheelsSecondOrder*>(Generator_::controller_ptr.get()))));

                Generator_::_sub.push_back(Generator_::n->template subscribe<custom_messages::StateMsg>("wheels_state", 1, boost::bind(&mgnss::ros_callbacks::wheels_controller_second_order::stateHandler,
                  _1, static_cast<mgnss::controllers::WheelsSecondOrder*>(Generator_::controller_ptr.get()))));
                }
    };


template<typename Subscriber, typename Service, typename Node, typename Publisher>
class WheeledMotionEvent3 : public WheelsControllerExtend<Subscriber, Service, Node, Publisher>
{
  typedef WheelsControllerExtend<Subscriber, Service, Node, Publisher> Precedesor_;

public:

  WheeledMotionEvent3() : Precedesor_(){}
  virtual ~WheeledMotionEvent3(){
  }

protected:
  virtual void _resetPrt(YAML::Node config){
          Precedesor_::controller_ptr.reset(new mgnss::controllers::WheeledMotionEvent3(*Precedesor_::_robot_ptr.begin()->second, config));
  }

  // virtual void _initCallbacks(YAML::Node config){
  //         Precedesor_::_srv.push_back(Precedesor_::n->template advertiseService<custom_services::updatePDGains::Request,
  //                                         custom_services::updatePDGains::Response>("wheels_command", boost::bind(&mgnss::ros_callbacks::wheels_controller_events::eventsHandler,  _1, _2, static_cast<mgnss::controllers::WheelsControllerExtend*>(Precedesor_::controller_ptr.get()))));
  //
  //         Precedesor_::_sub.push_back(Precedesor_::n->template subscribe<custom_messages::CustomCmnd>("wheels_support", 1, boost::bind(&mgnss::ros_callbacks::wheels_controller_events::supportHandler,_1, static_cast<mgnss::controllers::WheelsControllerExtend*>(Precedesor_::controller_ptr.get()))));
  //
  //         Precedesor_::_sub.push_back(Precedesor_::n->template subscribe<custom_messages::StateMsg>("wheels_state", 1, boost::bind(&mgnss::ros_callbacks::wheels_controller_events::stateHandler,_1, static_cast<mgnss::controllers::WheelsControllerExtend*>(Precedesor_::controller_ptr.get()))));
  //
  // }

};

template<typename Subscriber, typename Service, typename Node, typename Publisher>
class WheelsReactif : public WheeledMotionEvent3<Subscriber, Service, Node, Publisher>
{
  typedef WheeledMotionEvent3<Subscriber, Service, Node, Publisher> Precedesor_;

public:

  WheelsReactif() : Precedesor_(){}
  virtual ~WheelsReactif(){
  }

  protected:
  virtual void _resetPrt(YAML::Node config){
          Precedesor_::controller_ptr.reset(new mgnss::controllers::WheelsReactif(*Precedesor_::_robot_ptr.begin()->second, config));
  }

  };


  template<typename Subscriber, typename Service, typename Node, typename Publisher>
  class WheeledMotionWorld : public WheelsControllerExtend<Subscriber, Service, Node, Publisher>
  {
    typedef WheelsControllerExtend<Subscriber, Service, Node, Publisher> Precedesor_;

  public:
  WheeledMotionWorld() : Precedesor_(){}
  virtual ~WheeledMotionWorld(){
  }

  protected:
  virtual void _resetPrt(YAML::Node config){
          Precedesor_::controller_ptr.reset(new mgnss::controllers::WheeledMotionWorld(*Precedesor_::_robot_ptr.begin()->second, config));
  }
  };

  template<typename Subscriber, typename Service, typename Node, typename Publisher>
  class WheelsZMP : public WheelsControllerExtend<Subscriber, Service, Node, Publisher>
  {
    typedef WheelsControllerExtend<Subscriber, Service, Node, Publisher> Precedesor_;

public:
  WheelsZMP() : Precedesor_(){}
  virtual ~WheelsZMP(){
  }

protected:
  virtual void _resetPrt(YAML::Node config){
          Precedesor_::controller_ptr.reset(new mgnss::controllers::WheelsZMP(*Precedesor_::_robot_ptr.begin()->second, config));
  }
};

  template<typename Subscriber, typename Service, typename Node, typename Publisher>
  class WheelsZMPII : public WheelsControllerExtend<Subscriber, Service, Node, Publisher>
  {
    typedef WheelsControllerExtend<Subscriber, Service, Node, Publisher> Precedesor_;

    public:
      WheelsZMPII() : Precedesor_(){}
      virtual ~WheelsZMPII(){
      }

    protected:
      virtual void _resetPrt(YAML::Node config){
              Precedesor_::controller_ptr.reset(new mgnss::controllers::WheelsZMPII(*Precedesor_::_robot_ptr.begin()->second, config));
      }

      virtual void _initCallbacks(YAML::Node config){
        Precedesor_::_initCallbacks(config);
        Precedesor_::_srv.push_back(Precedesor_::n->template advertiseService<mgnss_utils::force::Request,
                                        mgnss_utils::force::Response>("force_limits", boost::bind(&mgnss::ros_callbacks::wheels_controller_zmp::forceLimits,  _1, _2, static_cast<mgnss::controllers::WheelsZMPII*>(Precedesor_::controller_ptr.get()))));
      }


   };

  template<typename Subscriber, typename Service, typename Node, typename Publisher>
  class WheelsZMPIII : public WheelsControllerExtend<Subscriber, Service, Node, Publisher>
  {
    typedef WheelsControllerExtend<Subscriber, Service, Node, Publisher> Precedesor_;

    public:
      WheelsZMPIII() : Precedesor_(){}
      virtual ~WheelsZMPIII(){
      }

    protected:
      virtual void _resetPrt(YAML::Node config){
              Precedesor_::controller_ptr.reset(new mgnss::controllers::WheelsZMPIII(*Precedesor_::_robot_ptr.begin()->second, config));
      }
   };

   template<typename Subscriber, typename Service, typename Node, typename Publisher>
   class WheelsZMPIV : public WheelsControllerExtend<Subscriber, Service, Node, Publisher>
   {
     typedef WheelsControllerExtend<Subscriber, Service, Node, Publisher> Precedesor_;

     public:
       WheelsZMPIV() : Precedesor_(){}
       virtual ~WheelsZMPIV(){
       }

     protected:
       virtual void _resetPrt(YAML::Node config){
               Precedesor_::controller_ptr.reset(new mgnss::controllers::WheelsZMPIV(*Precedesor_::_robot_ptr.begin()->second, config));
       }
    };


template<typename Subscriber, typename Service, typename Node, typename Publisher>
class WheeledMotionMergeV1 : public WheeledMotionEvent3<Subscriber, Service, Node, Publisher>
{
  typedef WheeledMotionEvent3<Subscriber, Service, Node, Publisher> Precedesor_;

public:

WheeledMotionMergeV1() : Precedesor_(){}
virtual ~WheeledMotionMergeV1(){
}

protected:
virtual void _resetPrt(YAML::Node config){
        Precedesor_::controller_ptr.reset(new mgnss::controllers::WheeledMotionMergeV1(*Precedesor_::_robot_ptr.begin()->second, config));
}
// virtual void _initCallbacks(YAML::Node config){
//         Precedesor_::_srv.push_back(Precedesor_::n->template advertiseService<custom_services::updatePDGains::Request,
//                                        custom_services::updatePDGains::Response>("wheels_command", boost::bind(&mgnss::ros_callbacks::wheels_controller_merge::eventsHandler,_1, _2, static_cast<mgnss::controllers::WheeledMotionMergeV1*>(Precedesor_::controller_ptr.get()))));
//
//         Precedesor_::_sub.push_back(Precedesor_::n->template subscribe<custom_messages::CustomCmnd>("wheels_support", 1, boost::bind(&mgnss::ros_callbacks::wheels_controller_extend::supportHandler,_1, static_cast<mgnss::controllers::WheelsControllerExtend*>(Precedesor_::controller_ptr.get()))));
//
//         Precedesor_::_sub.push_back(Precedesor_::n->template subscribe<custom_messages::StateMsg>("wheels_state", 1, boost::bind(&mgnss::ros_callbacks::wheels_controller_events::stateHandler,_1, static_cast<mgnss::controllers::WheelsControllerExtend*>(Precedesor_::controller_ptr.get()))));
// }

};

}
}
}
#endif // RT_MY_TEST_H
