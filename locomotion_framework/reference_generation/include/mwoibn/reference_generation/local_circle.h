#ifndef REFERENCE_GENERATION_LOCAL_CIRCLE_H
#define REFERENCE_GENERATION_LOCAL_CIRCLE_H

#include "mwoibn/reference_generation/utils.h"
#include "mwoibn/reference_generation/basic_object.h"

#include <Eigen/Dense>
namespace mwoibn {
namespace reference_generation
{

class Local_Circle : public BasicObject<float>
{
public:
Local_Circle(Eigen::VectorXd x_0, Eigen::VectorXd x_des, Eigen::Vector3d axis,
             float limit);

Local_Circle(Eigen::VectorXd x_0, Eigen::VectorXd x_des,
             Eigen::VectorXd x_pos, Eigen::Vector3d axis, float limit);
virtual ~Local_Circle() {
}

virtual Eigen::VectorXd nextStep();
virtual Eigen::VectorXd backStep();

virtual Eigen::VectorXd getPoint(double alpha);

virtual Eigen::VectorXd getFinalPoint();

virtual Eigen::VectorXd getOriginPoint();

virtual Eigen::VectorXd getCurrentPoint();


private:
bool _init(Eigen::VectorXd x_0, Eigen::VectorXd x_des, Eigen::Vector3d axis,
           float limit);
Circle _circle;
//  float _angle_step;
//  float _angle_end;
//  float _angle_current;
//  float _angle_origin;
};
} // namespace package
} // namespace library
#endif // REFERENCE_GENERATION_CIRCLE_H
