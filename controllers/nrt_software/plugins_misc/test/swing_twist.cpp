//#include <mwoibn/eigen_utils/quaternion.h>
#include <iostream>
#include <mwoibn/common/types.h>

int main(int argc, char** argv)
{

  const double PI = 3.14159265359;

  double z = PI/4, x = PI/6, y = 0;
  Eigen::Matrix3d test_x, test_y, test_z;
  mwoibn::Vector3 axis;

  axis << 0,0,1;
  test_z << std::cos(z), -std::sin(z), 0, std::sin(z), std::cos(z), 0, 0, 0, 1;
  test_x << 1, 0, 0, 0, std::cos(x), -std::sin(x), 0, std::sin(x), std::cos(x);

  mwoibn::Quaternion quat_z = mwoibn::Quaternion::fromMatrix(test_z*test_x);
  std::cout << "quat_z " << quat_z << std::endl;

  mwoibn::Quaternion quat_swing, quat_twist;

  std::cout << "SWING-TWIST" << std::endl;

  quat_twist = quat_z.swingTwist(axis, quat_swing);


  std::cout << "quat_swing " << quat_swing << std::endl;
  std::cout << "quat_twist " << quat_twist << std::endl;

  std::cout << "check " << quat_swing*quat_twist << std::endl;

  mwoibn::Axis axis_twist;
  double ang = quat_twist.toAxisAngle(axis_twist);
  std::cout << "axis twist " << axis_twist.transpose() << std::endl;
  std::cout << "angle twist" << ang << std::endl;

  ang = quat_swing.toAxisAngle(axis_twist);
  std::cout << "axis swing " << axis_twist.transpose() << std::endl;
  std::cout << "angle swing " << ang << std::endl;

  ang = quat_z.toAxisAngle(axis_twist);
  std::cout << "axis whole " << axis_twist.transpose() << std::endl;
  std::cout << "angle whole" << ang << std::endl;


  std::cout << "TWIST-SWING" << std::endl;

  quat_twist = quat_z.twistSwing(axis, quat_swing);

  std::cout << "quat_swing " << quat_swing << std::endl;
  std::cout << "quat_twist " << quat_twist << std::endl;

  std::cout << "check " << quat_twist*quat_swing<< std::endl;

  ang = quat_twist.toAxisAngle(axis_twist);
  std::cout << "axis twist " << axis_twist.transpose() << std::endl;
  std::cout << "angle twist" << ang << std::endl;

  ang = quat_swing.toAxisAngle(axis_twist);
  std::cout << "axis swing " << axis_twist.transpose() << std::endl;
  std::cout << "angle swing " << ang << std::endl;

  ang = quat_z.toAxisAngle(axis_twist);
  std::cout << "axis whole " << axis_twist.transpose() << std::endl;
  std::cout << "angle whole" << ang << std::endl;
}
