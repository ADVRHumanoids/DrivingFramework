#include "mwoibn/robot_points/contact_v2.h"

void mwoibn::robot_points::ContactV2::_initDirections(
    mwoibn::Matrix6 directions)
{

  for (int i = 0; i < directions.cols(); i++)
  {
    if (directions.block<3, 1>(0, i).norm() > 0.0001)
      _directions.block<3, 1>(0, i) = directions.block<3, 1>(0, i).normalized();
    else
      _directions.block<3, 1>(0, i) << 0, 0, 0;

    if (directions.block<3, 1>(3, i).norm() > 0.0001)
      _directions.block<3, 1>(3, i) = directions.block<3, 1>(3, i).normalized();
    else
      _directions.block<3, 1>(3, i) << 0, 0, 0;
  }

  _directions = _directions.transpose().eval();
//  _directions.transposeInPlace();
}

const mwoibn::Matrix& mwoibn::robot_points::ContactV2::getPointJacobian()
{
  return getPointJacobian(_frame.getRotationWorld().transpose()); // transpose?
}

const mwoibn::Matrix& mwoibn::robot_points::ContactV2::getWorldJacobian()
{
  _transformation.noalias() = getPointJacobian();
  _jacobian.noalias() = _frame.getRotationWorld()*_transformation;

  return _jacobian;
}


const mwoibn::Matrix&
mwoibn::robot_points::ContactV2::getPointJacobian(mwoibn::Matrix3 rotation_matrix)
{

  _jacobian.setZero();

  if (!_is_active) return _jacobian;

  _rotation.topLeftCorner<3,3>() = rotation_matrix;
  _rotation.bottomRightCorner<3,3>() = rotation_matrix;

  _transformation.noalias() = _directions * _rotation;
  _jacobian.noalias() = _transformation * _frame.getFullJacobian();

  return _jacobian;
}

mwoibn::VectorN mwoibn::robot_points::ContactV2::getPosition()
{
  return _frame.getFullStateWorld();
}

const mwoibn::VectorN& mwoibn::robot_points::ContactV2::getReactionForce(){
  return _wrench.getWorld();
}

void mwoibn::robot_points::ContactV2::_read(YAML::Node contact)
{

  if (!contact["position"])
    _readError("position");

  if (!contact["constraint_matrix"])
    _readError("constraint_matrix");


  std::vector<double> temp = contact["position"].as<std::vector<double>>();
  mwoibn::point_handling::Point::Current linear = mwoibn::Vector3::Map(temp.data(), temp.size());
  mwoibn::point_handling::Orientation::O orientation = mwoibn::point_handling::Orientation::O(0, 0, 0, 1);

  _frame.setLinearFixed(linear);
  _frame.setOrientationFixed(orientation);


  YAML::Node constraints = contact["constraint_matrix"];

  const std::vector<std::string> fields {"force_1", "force_2", "force_3", "momentum_1", "momentum_2", "momentum_3"};

  for (auto field: fields){
    if(!constraints[field])
      _readError(field);
    if(constraints[field].size() != 6){
      std::stringstream errMsg;
      errMsg << "Expected size 6 for " << field << "in \"constraint_matrix \" for a contact point " << _name
             << ". Got " << constraints[field].size() << " instead" << std::endl;
      throw(std::invalid_argument(errMsg.str().c_str()));
    }

  }

  mwoibn::Matrix6 directions = mwoibn::Matrix6::Zero(6, 6);

  directions.col(3) = mwoibn::VectorN::Map(
      constraints["force_1"].as<std::vector<double>>().data(), 6); // this is not a save call
  directions.col(4) = mwoibn::VectorN::Map(
      constraints["force_2"].as<std::vector<double>>().data(), 6); // this is not a save call
  directions.col(5) = mwoibn::VectorN::Map(
      constraints["force_3"].as<std::vector<double>>().data(), 6); // this is not a save call  //  directions.col(4) = temp_vector_6;
  directions.col(0) = mwoibn::VectorN::Map(
      constraints["momentum_1"].as<std::vector<double>>().data(), 6); // this is not a save call
  directions.col(1) = mwoibn::VectorN::Map(
      constraints["momentum_2"].as<std::vector<double>>().data(), 6); // this is not a save call
  directions.col(2) = mwoibn::VectorN::Map(
      constraints["momentum_3"].as<std::vector<double>>().data(), 6); // this is not a save call  //  directions.col(4) = temp_vector_6;

  _initDirections(directions);

}
