#ifndef COMMUNICATION_MODULES_BASIC_OPERATIONAL_EULER_H
#define COMMUNICATION_MODULES_BASIC_OPERATIONAL_EULER_H

#include "mwoibn/communication_modules/basic_feedback.h"

#include <rbdl/rbdl.h>

namespace mwoibn
{

namespace communication_modules
{

class BasicOperationalEuler : public BasicFeedback
{

public:
  //  BasicOperationalEuler(mwoibn::robot_class::State& command,
  //                   mwoibn::robot_class::BiMap map, bool position, bool
  //                   velocity,
  //                   bool torque)
  //      : BasicFeedback(command, map, position, velocity, torque)
  //  {
  //  }

  BasicOperationalEuler(mwoibn::robot_class::State& command,
                        mwoibn::robot_class::BiMap map, YAML::Node config)
      : BasicFeedback(command, map, config)
  {

    if (!config["output_angles"])
      throw(std::invalid_argument("Missing required argument output angels."));
    if (!config["output_angles"]["angle_1"])
      throw(std::invalid_argument(
          "Missing required argument [output angels][angle_1]."));
    if (!config["output_angles"]["angle_2"])
      throw(std::invalid_argument(
          "Missing required argument [output angels][angle_2]."));
    if (!config["output_angles"]["angle_3"])
      throw(std::invalid_argument(
          "Missing required argument [output angels][angle_3]."));

    _angels << config["output_angles"]["angle_1"].as<double>(),
        config["output_angles"]["angle_2"].as<double>(),
        config["output_angles"]["angle_3"].as<double>();

    if (!config["offset_orientation"])
    {
      std::cout << "Argument [offset_orientation] has not been defined assume "
                   "no offset." << std::endl;
      _offset_orientation.setIdentity();
    }
    else
    {
      if (!config["offset_orientation"]["xx"])
        throw(std::invalid_argument(
            "Missing required argument [offset_orientation][xx]."));
      if (!config["offset_orientation"]["yy"])
        throw(std::invalid_argument(
            "Missing required argument [offset_orientation][yy]."));
      if (!config["offset_orientation"]["zz"])
        throw(std::invalid_argument(
            "Missing required argument [offset_orientation][zz]."));
      if (!config["offset_orientation"]["xy"])
        throw(std::invalid_argument(
            "Missing required argument [offset_orientation][xy]."));
      if (!config["offset_orientation"]["yz"])
        throw(std::invalid_argument(
            "Missing required argument [offset_orientation][yz]."));
      if (!config["offset_orientation"]["xz"])
        throw(std::invalid_argument(
            "Missing required argument [offset_orientation][xz]."));
      if (!config["offset_orientation"]["yx"])
        throw(std::invalid_argument(
            "Missing required argument [offset_orientation][yx]."));
      if (!config["offset_orientation"]["zy"])
        throw(std::invalid_argument(
            "Missing required argument [offset_orientation][zy]."));
      if (!config["offset_orientation"]["zx"])
        throw(std::invalid_argument(
            "Missing required argument [offset_orientation][zx]."));

      YAML::Node q = config["offset_orientation"];
      _offset_orientation << q["xx"].as<double>(), q["xy"].as<double>(),
          q["xz"].as<double>(), q["yx"].as<double>(), q["yy"].as<double>(),
          q["yz"].as<double>(), q["zx"].as<double>(), q["zy"].as<double>(),
          q["zz"].as<double>();
    }

    if (!config["offset_position"])
    {
      std::cout << "Argument [offset_position] has not been defined assume "
                   "no offset." << std::endl;
      _offset_position << 0, 0, 0;
    }
    else
    {
      if (!config["offset_position"]["x"])
        throw(std::invalid_argument(
            "Missing required argument [offset_position][x]."));
      if (!config["offset_position"]["y"])
        throw(std::invalid_argument(
            "Missing required argument [offset_position][y]."));
      if (!config["offset_position"]["z"])
        throw(std::invalid_argument(
            "Missing required argument [offset_position][z]."));

      YAML::Node q = config["offset_position"];
      _offset_position << q["x"].as<double>(), q["y"].as<double>(),
          q["z"].as<double>();
    }

    mwoibn::VectorInt init_map = _map.get();
    _map_dofs.resize(_size);
    int k = 0;
    for(int i = 0; i < init_map.size(); i++){
      if(init_map[i] != robot_class::NON_EXISTING){
          if(k > _size) throw(std::invalid_argument(
                "Received wrong map, only " + std::to_string(_size) + " arguments can be deifned"));
          _map_dofs[k] = i;
          k++;
    }
    }

  }

  virtual ~BasicOperationalEuler() {}

  virtual bool initialized() { return false;}
  virtual bool get() { return false;}

  // this recomputation is correct for points descrbied by euler angels
  virtual void getPosition(mwoibn::Quaternion orientation,
                           mwoibn::Vector3 position)
  {
    getPosition(orientation.toMatrix(), position);
  }

  virtual void getPosition(mwoibn::Matrix3 orientation,
                           mwoibn::Vector3 position)
  {
    mwoibn::VectorN base(_size);
    //    mwoibn::Quaternion orientation(msg->pose[_base_ref].orientation.x,
    //                                   msg->pose[_base_ref].orientation.y,
    //                                   msg->pose[_base_ref].orientation.z,
    //                                   -msg->pose[_base_ref].orientation.w);

    //    base.head(3) << msg->pose[_base_ref].position.x,
    //        msg->pose[_base_ref].position.y, msg->pose[_base_ref].position.z;
    base.head(3) = position - _offset_position;
    base.tail(3) =
        (_offset_orientation * orientation)
            .eulerAngles(_angels[0], _angels[1],
                         _angels[2]); // Check if the convention is met here


    _command.set(base, _map_dofs, mwoibn::robot_class::INTERFACE::POSITION);
  }

protected:
  mwoibn::Vector3 _offset_position;
  mwoibn::Matrix3 _offset_orientation;
  mwoibn::Vector3 _angels;
  mwoibn::VectorInt _map_dofs;
  int _size = 6;
};
}
}

#endif // BASIC_CONTROLLER_H
