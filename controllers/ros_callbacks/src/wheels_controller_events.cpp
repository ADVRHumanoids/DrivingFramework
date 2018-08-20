#include "mgnss/ros_callbacks/wheels_controller_events.h"

bool mgnss::ros_callbacks::wheels_controller_events::eventsHandler(custom_services::updatePDGains::Request& req, custom_services::updatePDGains::Response& res, mgnss::controllers::WheelsControllerExtend* controller_ptr){
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
        return false;
}


void mgnss::ros_callbacks::wheels_controller_events::supportHandler(const custom_messages::CustomCmndConstPtr& msg, mwoibn::VectorN* support, mgnss::controllers::WheelsControllerExtend* controller_ptr)
{
        if(msg->position.size() > 12 && msg->position[12] == mwoibn::IS_VALID) {
                for(int i = 0; i < 12; i++) {
                        (*support)[i] = msg->position[i];
                }
                controller_ptr->setSupport(*support);
        }

        if(msg->velocity.size() > 12 && msg->velocity[12] == mwoibn::IS_VALID) {
                for(int i = 0; i < 12; i++) {
                        (*support)[i] = msg->velocity[i];
                }
                controller_ptr->setSupportVel(*support);
        }

}

bool mgnss::ros_callbacks::wheels_controller_events::stateHandler(const custom_messages::StateMsgConstPtr& msg, mgnss::controllers::WheelsControllerExtend* controller_ptr){
        // std::cout << "stateHandler " << std::endl;
        // std::cout << msg->x << std::endl;
        // std::cout << msg->y << std::endl;
        // std::cout << msg->z << std::endl;
        // std::cout << msg->rz << std::endl;
        controller_ptr->setBaseDotX(msg->x);
        controller_ptr->setBaseDotY(msg->y);
        controller_ptr->setBaseDotZ(msg->z);

        controller_ptr->setBaseDotHeading(msg->rz);

        if(msg->cs.size() == 4) {
                controller_ptr->setCamber(0, msg->cs[0]);
                controller_ptr->setCamber(1, msg->cs[1]);
                controller_ptr->setCamber(2, msg->cs[2]);
                controller_ptr->setCamber(3, msg->cs[3]);
        }
        if(msg->cm.size() == 4) {
                controller_ptr->setCastor(0, msg->cs[0]);
                controller_ptr->setCastor(1, msg->cs[1]);
                controller_ptr->setCastor(2, msg->cs[2]);
                controller_ptr->setCastor(3, msg->cs[3]);
        }
        if(msg->st.size() == 4) {
                controller_ptr->setSteering(0, msg->st[0]);
                controller_ptr->setSteering(1, msg->st[1]);
                controller_ptr->setSteering(2, msg->st[2]);
                controller_ptr->setSteering(3, msg->st[3]);
        }
}
