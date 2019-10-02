#include "mgnss/ros_callbacks/wheels_controller_extend.h"

bool mgnss::ros_callbacks::wheels_controller_extend::eventsHandler(custom_services::updatePDGains::Request& req, custom_services::updatePDGains::Response& res, mgnss::controllers::WheelsControllerExtend* controller_ptr){
        // std::cout << "callback\t" << req.p << "\t" << req.d << std::endl;

        if (req.p == 1)
        { // base
                if (req.d == 1)
                        controller_ptr->setBaseDotX(req.nr / 1000.0);
                else if (req.d == 2)
                        controller_ptr->setBaseDotY(req.nr / 1000.0);
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
                if (req.d == 1 && req.nr >= 0 && req.nr < 4) {
                        controller_ptr->claim(req.nr);
                        res.success = true;
                        return true;
                }
                else if (req.d == 0 && req.nr >= 0 && req.nr < 4) {
                        controller_ptr->release(req.nr);
                        res.success = true;
                        return true;
                }
        }
        else if (req.p == 4) {
                {
                        if (req.d >= 0 && req.d < 4) {
                                controller_ptr->setCamber(req.d, req.nr*mwoibn::PI/180/10);
                                res.success = true;
                                return true;
                        }
                }
        }
        else if (req.p == 5) {
                {
                        if (req.d >= 0 && req.d < 4) {
                                controller_ptr->setCastor(req.d, req.nr*mwoibn::PI/180/10);
                                res.success = true;
                                return true;
                        }
                }
        }
        else if (req.p == 6) {
                {
                        if (req.d >= 0 && req.d < 4) {
                                controller_ptr->setSteering(req.d, req.nr*mwoibn::PI/180/10);
                                res.success = true;
                                std::cout << req.nr << std::endl;
                                return true;
                        }
                }

        }
        return false;
}


void mgnss::ros_callbacks::wheels_controller_extend::supportHandler(const custom_messages::CustomCmndConstPtr& msg, mgnss::controllers::WheelsControllerExtend* controller_ptr)
{
        if(msg->position.size() > 12 && msg->position[12] == mwoibn::IS_VALID) {
                for(int i = 0; i < 12; i++) controller_ptr->setSupport(i, msg->position[i]);
        }

        if(msg->velocity.size() > 12 && msg->velocity[12] == mwoibn::IS_VALID) {
                for(int i = 0; i < 12; i++) controller_ptr->setSupportVel(i, msg->velocity[i]);
        }

}
