#include "mgnss/nrt_software/plugins/wheels_controllers.h"
#include "mgnss/plugins/ros_base.h"
#if XBOT
  #include "mgnss/plugins/xbot_base_v2.h"
#endif

MGNSS_REGISTER_ROS_PLUGIN_(mgnss::nrt_software::plugins::WheeledMotionMergeV1)
#if XBOT
  MGNSS_REGISTER_XBOT_PLUGIN_(mgnss::nrt_software::plugins::WheeledMotionMergeV1)
#endif
