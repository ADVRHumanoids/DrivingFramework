#include "mgnss/ros_callbacks/wheels_controller_zmp.h"

bool mgnss::ros_callbacks::wheels_controller_zmp::forceLimits(mgnss_utils::force::Request& req,
                 mgnss_utils::force::Response& res, mgnss::controllers::WheelsZMPII* controller_ptr){

        controller_ptr->F_min[req.nr] = req.min;
        controller_ptr->F_max[req.nr] = req.max;
        res.success = true;
        return true;

}
