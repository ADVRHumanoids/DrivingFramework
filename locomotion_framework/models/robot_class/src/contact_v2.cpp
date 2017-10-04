#include "mwoibn/robot_class/contact_v2.h"

void mwoibn::robot_class::ContactV2::_initDirections(
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

const mwoibn::Matrix& mwoibn::robot_class::ContactV2::getPointJacobian(int i)
{
  return getPointJacobian(_points[i]->getRotationWorld(_positions));
}

const mwoibn::Matrix&
mwoibn::robot_class::ContactV2::getPointJacobian(mwoibn::Matrix3 rotation_matrix,
                                                 int i)
{

  _jacobian.setZero();

  if (!_is_active) return _jacobian;

  _rotation.topLeftCorner<3,3>() = rotation_matrix;
  _rotation.bottomRightCorner<3,3>() = rotation_matrix;

  _transformation.noalias() = _directions * _rotation;
  _jacobian.noalias() = _transformation * _points[i]->getFullJacobian(_positions);

  return _jacobian;
}

mwoibn::VectorN mwoibn::robot_class::ContactV2::getPosition()
{

  return getFullStateWorld();
}

void mwoibn::robot_class::ContactV2::_read(YAML::Node contact)
{

  // check if all required fileds are here
  if (!contact["end_frame"])
    _readError("end_frame");

  if (!contact["position"])
    _readError("position");

  if (!contact["constraint_matrix"])
    _readError("constraint_matrix");

  if(contact["active"] && contact["active"].as<bool>())
    _is_active = true;
  else
    _is_active = false;

  std::string end_frame = contact["end_frame"].as<std::string>();

  std::string name = contact["name"] ? contact["name"].as<std::string>() : "";

  mwoibn::Vector7 state;
  std::vector<double> temp = contact["position"].as<std::vector<double>>();
  state.head(3) = mwoibn::Vector3::Map(temp.data(), temp.size());
  state.tail(4) << 0, 0, 0, 1;

  addPoint(state, end_frame, name);

  computeChain();

  //  if (!contact["type"])
  //    throw(std::invalid_argument(
  //        _readError("type").c_str()));

  //  if (robot_class::contact_type.find(contact["type"]) ==
  //      robot_class::contact_type.end())
  //  {
  //    std::stringstream errMsg;
  //    errMsg << "A contact type for " << _contacts->size() + 1
  //           << " is not defined in a robot_class." << std::endl;
  //    throw(std::invalid_argument(errMsg.str().c_str()));
  //  }

  // check if all directions are specified

  YAML::Node constraints = contact["constraint_matrix"];

  const std::vector<std::string> fields {"force_1", "force_2", "force_3", "momentum_1", "momentum_2", "momentum_3"};

  for (auto field: fields){
    if(!constraints[field])
      _readError(field);
    if(constraints[field].size() != 6){
      std::stringstream errMsg;
      errMsg << "Expected size 6 for " << field << "in \"constraint_matrix \" for a contact point " << name
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

void mwoibn::robot_class::ContactV2::_readError(std::string param)
{
  std::stringstream errMsg;
  errMsg << "There was an error while reading the " << param << std::endl;
  throw(std::invalid_argument(errMsg.str().c_str()));
}
