/*
 * Copyright 2016 <copyright holder> <email>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "mwoibn/robot_class/robot.h"

#ifndef RBDL_BUILD_ADDON_URDFREADER
#error "Error: RBDL addon URDFReader not enabled."
#endif

#include <rbdl/addons/urdfreader/urdfreader.h>

#include <urdf/model.h>
#include <srdfdom/model.h>

mwoibn::robot_class::Robot::Robot(std::string urdf_description,
                                  std::string srdf_description, bool from_file)
{
  _init(urdf_description, srdf_description, from_file);
}

void mwoibn::robot_class::Robot::_init(std::string urdf_description,
                                       std::string srdf_description,
                                       bool from_file)
{
  std::stringstream errMsg;

  urdf::Model urdf;
  srdf::Model srdf;

  if (from_file)
  {
    if (!urdf.initFile(urdf_description))
    {
      errMsg << "Could not load urdf description";
      throw(std::invalid_argument(errMsg.str().c_str()));
    }

    if (srdf_description != "")
    {
      if (!srdf.initFile(urdf, srdf_description))
      {
        errMsg << "Could not load srdf description";
        throw(std::invalid_argument(errMsg.str().c_str()));
      }
    }
  }
  else
  {
    if (!urdf.initString(urdf_description))
    {
      errMsg << "Could not load urdf description";
      throw(std::invalid_argument(errMsg.str().c_str()));
    }

    if (srdf_description != "")
    {
      if (!srdf.initString(urdf, srdf_description))
      {
        errMsg << "Could not load srdf description";
        throw(std::invalid_argument(errMsg.str().c_str()));
      }
    }
  }

  _is_static = (urdf.getRoot()->child_joints[0]->type == urdf::Joint::FLOATING)
                   ? false
                   : true;

  try
  {
    if (from_file)
    {
      if (!RigidBodyDynamics::Addons::URDFReadFromFile(
              urdf_description.c_str(), &_model, !_is_static, false))
        throw std::invalid_argument(
            std::string("Error loading model from file"));
    }
    else
    {

      if (!RigidBodyDynamics::Addons::URDFReadFromString(
              urdf_description.c_str(), &_model, !_is_static, false))
        throw std::invalid_argument(
            std::string("Error loading model from string"));
    }
  }
  catch (...)
  {
    throw std::invalid_argument(std::string("Error loading model"));
  }

  _initMapNames(urdf.getRoot());

  state.restart(getDofs());
  command.restart(getDofs());
  _actuation = mwoibn::VectorInt::Ones(getDofs());

  _center_of_mass.reset(new CenterOfMass(_model,
                                         state.state(INTERFACE::POSITION),
                                         state.state(INTERFACE::VELOCITY)));
  _contacts.reset(new Contacts(getDofs()));

  // set to unactuated all dofs not in the model
  if (!_is_static)
    _actuation.head(6) << 0, 0, 0, 0, 0, 0;

  for (int i = 0; i < _model.mBodies.size(); i++)
  {
    if (_model.IsFixedBodyId(i))
    {
      continue;
    }
    std::string link_name = _model.GetBodyName(i);

    if (urdf.getLink(link_name) == NULL)
    {
      //      RigidBodyDynamics::Joint joint =
      //          _model.mJoints[_model.GetBodyId(link_name.c_str())];
      mwoibn::VectorInt dofs = getDof(link_name);
      for (int j = 0; j < dofs.size(); j++)
      {
        _actuation[dofs[j]] = 0;
      }
    }
  }

  // start simples mapping
  mwoibn::VectorInt rbdl_map(getDofs());
  for (int i = 0; i < getDofs(); i++)
    rbdl_map[i] = i;

  biMaps().addMap(BiMap("RBDL", rbdl_map));

  _zeroVec.setZero(getDofs());

}

void mwoibn::robot_class::Robot::_initMapNames(
    boost::shared_ptr<const urdf::Link> link)
{
  for (auto& child_link : link->child_links)
  {
    _map_names.left.insert(boost_map::left_value_type(
        child_link->parent_joint->name, child_link->name));
    _initMapNames(child_link);
  }
}

std::string mwoibn::robot_class::Robot::getLink(std::string joint_name)
{

  try
  {
    return _map_names.left.at(joint_name);
  }
  catch (const std::out_of_range& e)
  {
    throw;
  }
}

std::vector<std::string> mwoibn::robot_class::Robot::getLinks(
    const std::vector<std::string>& joint_names)
{
  std::vector<std::string> link_names;
  for (auto joint : joint_names)
    try
    {
      link_names.push_back(getLink(joint));
    }
    catch (const std::out_of_range& e)
    {
      link_names.push_back("");
    }

  return link_names;
}

std::string mwoibn::robot_class::Robot::getJoint(std::string link_name)
{

  try
  {
    return _map_names.right.at(link_name);
  }
  catch (const std::out_of_range& e)
  {
    throw;
  }
}

std::vector<std::string> mwoibn::robot_class::Robot::getJoints(
    const std::vector<std::string>& link_names)
{
  std::vector<std::string> joint_names;
  for (auto link : link_names)
    try
    {
      joint_names.push_back(getJoint(link));
    }
    catch (const std::out_of_range& e)
    {
      joint_names.push_back("");
    }

  return joint_names;
}

mwoibn::VectorInt mwoibn::robot_class::Robot::getDof(std::string link_name)
{
  int size;
  int init;

  if (_model.GetBodyId(link_name.c_str()) == RBDL_NON_EXISTING)
  {
    return {};
  }

  RigidBodyDynamics::Joint joint =
      _model.mJoints[_model.GetBodyId(link_name.c_str())]; // here it should
                                                           // try to catch and
                                                           // rethrow an
                                                           // exception for
                                                           // non-exisiting
                                                           // joints
  if (!_is_static && joint.q_index < 6 && joint.q_index + joint.mDoFCount == 6)
  {
    size = 6;
    init = 0;
  }
  else if (!_is_static && joint.q_index < 6)
  {
    return {};
  }
  else
  {
    size = _model.mJoints[_model.GetBodyId(link_name.c_str())].mDoFCount;
    init = _model.mJoints[_model.GetBodyId(link_name.c_str())].q_index;
  }

  mwoibn::VectorInt dofs(size);
  for (int i = 0; i < size; i++)
    dofs[i] = init + i;
  //  std::iota(dofs.begin(), dofs.end(),
  //            init); // Fill with init, init+1, ..., init_size.
  return dofs;
}

mwoibn::VectorInt
mwoibn::robot_class::Robot::getDof(std::vector<std::string> link_names)
{
  mwoibn::VectorInt dofs;
  int size = 0;
  for (auto& link : link_names)
  {
    mwoibn::VectorInt link_dofs = getDof(link);
    size += link_dofs.size();
    dofs.resize(size);
    dofs.tail(link_dofs.size()) = link_dofs;
  }

  return dofs;
}

YAML::Node
mwoibn::robot_class::Robot::_getConfig(const std::string config_file,
                                       const std::string secondary_file)
{
  YAML::Node config;

  try
  {
    config = YAML::LoadFile(config_file);
  }
  catch (const YAML::BadFile& e)
  {
    throw std::invalid_argument(
        std::string("Couldn\t find the configuration file: ") + config_file);
  }
  catch (...)
  {
    throw;
  }

  if (secondary_file.empty())
    return config["mwoibn"];

  YAML::Node config_2;
  try
  {
    config_2 = YAML::LoadFile(secondary_file);
  }
  catch (const YAML::BadFile& e)
  {
    throw std::invalid_argument(
        std::string("Couldn\t find the secondary configuration file: ") +
        config_file);
  }
  catch (...)
  {
    throw std::invalid_argument(
        std::string("Unkown error reading the secondary configuration file: ") +
        config_file);
  }

  config = config["mwoibn"];
  config_2 = config_2["mwoibn"];

  _compareEntry(config, config_2);
  return config;
}

void mwoibn::robot_class::Robot::_compareEntry(YAML::Node entry_main,
                                               YAML::Node entry_second)
{
  for (auto entry : entry_second)
  {
    if (!entry_main[entry.first.as<std::string>()])
    {
      entry_main[entry.first.as<std::string>()] = entry.second;
    }
    else if (entry.second.IsMap())
    {
      _compareEntry(entry_main[entry.first.as<std::string>()], entry.second);
    }
    else
      entry_main[entry.first.as<std::string>()] = entry.second;
  }
}

void mwoibn::robot_class::Robot::_loadContacts(YAML::Node contacts_config)
{
  //  for (auto contact : contacts_config)
  for (int i = 0; i < contacts_config.size() - 1; i++)
  {
    YAML::Node contact = contacts_config["contact" + std::to_string(i)];

    if (!contact["type"])
      throw std::invalid_argument(
          std::string("Please specify a contact types"));
    try
    {
      //      contact.second["name"] = contact.first.as<std::string>();
      std::string type = contact["type"].as<std::string>();

      if (type.compare("point_foot") == 0)
      {
        _contacts->add(std::unique_ptr<mwoibn::robot_class::ContactV2>(
            new mwoibn::robot_class::ContactV2(
                _model, state.state(robot_class::INTERFACE::POSITION),
                contact)));
        continue;
      }
      if (type.compare("wheel") == 0)
      {
        _contacts->add(std::unique_ptr<mwoibn::robot_class::ContactV2>(
            new mwoibn::robot_class::WheelContactV2(
                _model, state.state(robot_class::INTERFACE::POSITION),
                contact)));
        continue;
      }
      if (type.compare("wheel_locked") == 0)
      {
        _contacts->add(std::unique_ptr<mwoibn::robot_class::ContactV2>(
            new mwoibn::robot_class::WheelContact(
                _model, state.state(robot_class::INTERFACE::POSITION),
                contact)));
        continue;
      }

      throw std::invalid_argument(std::string("Uknown contacy type: ") + type);
    }
    catch (std::invalid_argument& e)
    {
      std::cout << e.what() << std::endl;
    }
  }
}

void mwoibn::robot_class::Robot::_loadActuators(YAML::Node actuators_config)
{

  if (!actuators_config.IsMap())
  {
    std::cout << "Wrong actuator type, expected struct" << std::endl;
    std::cout << "Continue woithout actuators loaded" << std::endl;
    return;
  }

  std::vector<RigidBodyDynamics::Body> bodies = _model.mBodies;

  for (int i = 0; i < bodies.size(); i++)
  {
    if (_model.IsFixedBodyId(i))
    {
      //      std::cout << "fixed body" << std::endl;
      continue;
    }

    std::string link_name = _model.GetBodyName(i);

    RigidBodyDynamics::Joint joint =
        _model.mJoints[_model.GetBodyId(link_name.c_str())];

    mwoibn::VectorInt dofs = getDof(link_name);
    if (!dofs.size())
      continue;

    if (_actuation[dofs[0]] == 0)
    {
      for (int dof = 0; dof < dofs.size(); dof++)
      {
        actuators().add(std::unique_ptr<mwoibn::robot_class::Actuator>(
            new Actuator(ACTUATOR_TYPE::UNACTUATED, 0, 0, link_name)));
      }
      continue;
    }

    std::string joint_name;

    try
    {
      joint_name = getJoint(link_name);
    }
    catch (const std::out_of_range& e)
    {
      continue;
    }

    if (!actuators_config[joint_name])
    {

      for (int k = 0; k < joint.mDoFCount; k++)
      {
        actuators().add(std::unique_ptr<mwoibn::robot_class::Actuator>(
            new Actuator(ACTUATOR_TYPE::UNACTUATED, 0, 0, link_name)));
        //        std::cout << "notactuated" << std::endl;
      }
      continue;
    }

    YAML::Node actuator_data = actuators_config[joint_name];

    actuator_data["name"] = joint_name;

    if (!actuator_data["type"])
    {
      std::stringstream errMsg;
      errMsg << "Unspecified actuator type for joint '" << joint_name;
      throw(std::invalid_argument(errMsg.str().c_str()));
    }

    if (actuator_data["type"].as<std::string>() == "SEA")
    {
      actuators().add(std::unique_ptr<mwoibn::robot_class::Actuator>(
          new SeriesElasticActuator(actuator_data)));
    }
    else
    {
      std::stringstream errMsg;
      errMsg << "Unexpected actuator type: " << actuator_data["type"]
             << " for joint " << joint_name;
      throw(std::invalid_argument(errMsg.str().c_str()));
    }
  }
}

mwoibn::robot_class::BiMap
mwoibn::robot_class::Robot::makeBiMap(std::vector<std::string> link_names,
                                      std::string map_name)
{

  mwoibn::VectorInt map = mwoibn::VectorInt::Constant(getDofs(), NON_EXISTING);

  for (int i = 0; i < link_names.size(); i++)
  {

    try
    {
      mwoibn::VectorInt dofs = getDof(link_names[i]);
      if (dofs.size() > 1)
        throw(std::invalid_argument(link_names[i] + " has " +
                                    std::to_string(dofs.size()) +
                                    " unique mapping is not defined"));
      if (dofs.size() == 1)
        map[dofs[0]] = i;
    }
    catch (const std::out_of_range& e)
    {
      //        map[i] = robot_class::NON_EXISTING;
    }
  }

  //      std::cout << "new\n" << map << std::endl;
  return mwoibn::robot_class::BiMap(map_name, map);
}

mwoibn::robot_class::BiMap
mwoibn::robot_class::Robot::readBiMap(YAML::Node config)
{

  if (!config["chain"])
    throw(
        std::invalid_argument("Required element chain has not been defined."));

  if (!config["type"])
    throw(std::invalid_argument("Required element type has not been defined."));

  mwoibn::VectorBool chain = mwoibn::VectorBool::Constant(getDofs(), false);

  if (config["chain"].as<std::string>() == "all")
    chain.setConstant(true);
  else
    throw(std::invalid_argument(
        "Support for chains has not been implemented yet"));

  mwoibn::VectorBool type = mwoibn::VectorBool::Constant(getDofs(), false);
  if (config["type"].as<std::string>() == "all")
    type.setConstant(true);
  else if (config["type"].as<std::string>() == "actuated")
  {
    type = actuators().getActuationTypes({ACTUATOR_TYPE::UNACTUATED});
    type = eigen_utils::flip(type);
  }

  else if (config["type"].as<std::string>() == "body")
  {
    if (!config["name"])
      throw(std::invalid_argument(
          "Couldn't initialize mapping to the body, no [body][name] defined."));
    mwoibn::VectorInt temp = getDof(config["name"].as<std::string>());

    for (int i = 0; i < temp.size(); i++)
      type[temp[i]] = true;
  }

  if (!biMaps().isDefined(config["mapping"].as<std::string>()))
  {
    // try to initialize from ros and/or config file, I need to add initialize
    // mappings function later (weekend?)
    throw(std::invalid_argument(
        "Required mapping " + config["mapping"].as<std::string>() +
        " has not been defined in the robot instance."));
  }

  mwoibn::VectorInt map =
      biMaps().get(config["mapping"].as<std::string>()).get();

  for (int i = 0; i < getDofs(); i++)
    map[i] = (chain[i] && type[i]) ? map[i] : NON_EXISTING;

  return mwoibn::robot_class::BiMap(config["mapping"].as<std::string>(), map);
}

void mwoibn::robot_class::Robot::_loadConfig(YAML::Node config,
                                             YAML::Node robot)
{

  for (auto entry : robot)
  {
    if (config[entry.first.as<std::string>()])
      _compareEntry(entry.second, config[entry.first.as<std::string>()]);
  }
}

YAML::Node
mwoibn::robot_class::Robot::_readRobotConfig(const YAML::Node full_config,
                                             std::string config_name)
{

  // get specific configuration
  if (!full_config["robot"][config_name])
    throw std::invalid_argument(std::string("Unknown configuration: ") +
                                config_name);

  // in config_robot all the data that are used later should be stored
  YAML::Node config = full_config["robot"][config_name];

  // extend config_robot by robot name
  config["name"] = (full_config["robot"]["name"])
                       ? full_config["robot"]["name"].as<std::string>()
                       : "";

  try
  {
    config["feedback"] = _readConfig(
        full_config["feedbacks"], full_config["feedback"], config["feedback"]);
  }
  catch (const std::invalid_argument& e)
  {
    throw(std::invalid_argument("config_name: " + config_name + "\n" +
                                "\tfeedback\n" + e.what()));
  }

  try
  {
    config["controller"] =
        _readConfig(full_config["controllers"], full_config["controller"],
                    config["controller"]);
  }
  catch (const std::invalid_argument& e)
  {
    throw(std::invalid_argument("config_name: " + config_name + "\n" +
                                "\tcontroller\n" + e.what()));
  }
  return config;
}

YAML::Node mwoibn::robot_class::Robot::_readConfig(const YAML::Node lists,
                                                   const YAML::Node defined,
                                                   YAML::Node config)
{
  YAML::Node reduced;
  //  //load expected feedbacks
  if (!config || !lists || !defined)
    throw std::invalid_argument(std::string("Received an empty argument. "));

  // get zero options
  if (!config["layer"] && !config["list"])
    return reduced;

  if (config["layer"] && config["list"])
    throw(std::invalid_argument(
        std::string("Both layer and list arguments are defined.")));

  std::vector<std::string> list;
  if (config["layer"])
  {
    std::string name = config["layer"].as<std::string>();
    if (!lists[name]){
      std::cout << "No feedback has been defined for this configuration" << std::endl;
      return reduced;
    }
    list = lists[name].as<std::vector<std::string>>();
  }
  if (config["list"])
    list = config["list"].as<std::vector<std::string>>();

  if (list.size() == 1 && list[0] == "all")
    return config;

  for (auto entry : list)
  {

    if (!defined[entry])
      throw(std::invalid_argument("Expected argument " + entry +
                                  " has not been defined in the robot"));
    reduced[entry] = defined[entry];
  }

  return reduced;
}
void mwoibn::robot_class::Robot::_getDefaultPosition(YAML::Node config,
                                                     bool position,
                                                     bool orientation,
                                                     bool angels)
{

  mwoibn::Vector3 offset_position = mwoibn::Vector3::Zero();
  mwoibn::Matrix3 offset_orientation = mwoibn::Matrix3::Identity();

  for (int i = 0;
       i < _model.GetBodyId(config["dofs"]["name"].as<std::string>().c_str());
       i++)
  {
    offset_position += _model.GetJointFrame(i).r;
    offset_orientation =
        offset_orientation * _model.GetJointFrame(i).E.transpose();
  }

  if (orientation)
  {
    config["offset_orientation"]["xx"] = offset_orientation(0, 0);
    config["offset_orientation"]["yy"] = offset_orientation(1, 1);
    config["offset_orientation"]["zz"] = offset_orientation(2, 2);
    config["offset_orientation"]["xy"] = offset_orientation(0, 1);
    config["offset_orientation"]["xz"] = offset_orientation(0, 2);
    config["offset_orientation"]["yz"] = offset_orientation(1, 2);
    config["offset_orientation"]["yx"] = offset_orientation(1, 0);
    config["offset_orientation"]["zx"] = offset_orientation(2, 0);
    config["offset_orientation"]["zy"] = offset_orientation(2, 1);
  }
  if (position)
  {
    config["offset_position"]["x"] = offset_position[0];
    config["offset_position"]["y"] = offset_position[1];
    config["offset_position"]["z"] = offset_position[2];
  }
  if(angels){
  config["output_angles"]["angle_1"] = 0;
  config["output_angles"]["angle_2"] = 1;
  config["output_angles"]["angle_3"] = 2;
  }
}

bool mwoibn::robot_class::Robot::_loadFeedback(YAML::Node entry,
                                               std::string name)
{

  if (!entry["space"])
    throw(std::invalid_argument("Couldn't find a space arument in feedback " +
                                name));

  if (entry["space"].as<std::string>() == "SKIP")
    return false;

  entry["name"] = name;

  return true;
}
