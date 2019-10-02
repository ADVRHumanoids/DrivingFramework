#include "mgnss/ros_callbacks/upper_body_ik.h"

bool mgnss::ros_callbacks::upper_body_ik::setReference(mgnss_utils::point::Request& req,
     mgnss_utils::point::Response& res, mgnss::controllers::UpperBodyIK* controller_ptr)
{
        mwoibn::Vector3 reference;
        reference << req.x, req.y, req.z;
        controller_ptr->setReference(req.nr, reference);
        res.success = true;

        return true;
}
