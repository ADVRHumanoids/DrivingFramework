#include "mgnss/ros_callbacks/wheels_controller_merge_v1.h"

bool mgnss::ros_callbacks::wheels_controller_merge::eventsHandler(custom_services::updatePDGains::Request& req, custom_services::updatePDGains::Response& res, mgnss::controllers::WheeledMotionMergeV1* controller_ptr){
        // std::cout << "callback\t" << req.p << "\t" << req.d << std::endl;
        if( wheels_controller_extend::eventsHandler(req,res,controller_ptr)) return true;
        if (req.p == 7) {
                controller_ptr->switchToCastor(req.nr, req.d);
                res.success = true;
                res.message = "switchToCastor";
                return true;
        }
        else if (req.p == 8) {
                controller_ptr->switchToCamber(req.nr, req.d);
                res.success = true;
                res.message = "switchToCamber";
                return true;
        }
        return false;
}
