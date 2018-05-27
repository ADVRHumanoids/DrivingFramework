#include "mgnss/ros_callbacks/wheeled_motion_world.h"

bool mgnss::ros_callbacks::wheeled_motion_world::evenstHandler(custom_services::updatePDGains::Request& req, custom_services::updatePDGains::Response& res, mgnss::controllers::WheeledMotionWorld* controller_ptr){
 // std::cout << "callback\t" << req.p << "\t" << req.d << std::endl;

      if (req.p == 1)
      { // base
        if (req.d == 1)
          controller_ptr->setBaseDotX(req.nr / 100.0);
        else if (req.d == 2)
          controller_ptr->setBaseDotY(req.nr / 100.0);
        else if (req.d == 3)
          controller_ptr->setBaseDotZ(req.nr / 1000.0);
        else if (req.d == 4)
        {
 //         std::cout << "dot heading\t" << req.nr / 1000.0 << std::endl;
          controller_ptr->setBaseDotHeading(req.nr / 1000.0);
        }
        else if (req.d == 5)
          controller_ptr->setBaseRotVelX(req.nr / 100.0);
        else if (req.d == 6)
          controller_ptr->setBaseRotVelY(req.nr / 100.0);
        else
        {
          res.success = false;
          return false;
        }
        res.success = true;
        return true;
      }
      else if (req.p == 2)
      { // base
        if (req.d == 1)
          controller_ptr->setBaseX(req.nr / 100.0);
        else if (req.d == 2)
          controller_ptr->setBaseY(req.nr / 100.0);
        else if (req.d == 3)
          controller_ptr->setBaseZ(req.nr / 100.0);
        else if (req.d == 4)
          controller_ptr->setBaseHeading(req.nr / 1000.0);
        else if (req.d == 5)
          controller_ptr->rotateBaseX(req.nr / 100.0);
        else if (req.d == 6)
          controller_ptr->rotateBaseY(req.nr / 100.0);
        else
        {
          res.success = false;
          return false;
        }
        res.success = true;
        return true;
      }
      else if (req.p == 3)
      {
        if (req.d == 1 && req.nr >= 0 && req.nr < 4){
          controller_ptr->claim(req.nr);
          res.success = true;
          return true;
        }
        else if (req.d == 0 && req.nr >= 0 && req.nr < 4){
          controller_ptr->release(req.nr);
          res.success = true;
          return true;
        }
      }
      return false;
  }

void mgnss::ros_callbacks::wheeled_motion_world::supportHandler(const custom_messages::CustomCmndConstPtr& msg, mwoibn::VectorN* support, mgnss::controllers::WheeledMotionWorld* controller_ptr)
 {
    if(msg->position[12] == mwoibn::IS_VALID){
      for(int i = 0; i < 12; i++){
        (*support)[i] = msg->position[i];
      }
      }
    controller_ptr->setSupport(*support);
  }
