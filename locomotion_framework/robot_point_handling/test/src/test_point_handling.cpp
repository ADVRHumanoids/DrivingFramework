//#include "point_handling/points_handler.h"
#include "mwoibn/tests_common/test.h"
#include "mwoibn/point_handling/make_log.h"
#include "mwoibn/common/types.h"

int main(int argc, char** argv)
{
#ifndef MAKE_LOG
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
#endif
#ifdef MAKE_LOG
  mwoibn::point_handling::makeLog();
  return 0;
#endif
}
