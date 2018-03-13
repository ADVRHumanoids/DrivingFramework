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
#include "mwoibn/point_handling/raw_positions_handler.h"

mwoibn::robot_class::Robot::Robot(std::string urdf_description,
                                  std::string srdf_description)
{
  _init(urdf_description, srdf_description);
}

void mwoibn::robot_class::Robot::_init(std::string urdf_description,
                                       std::string srdf_description)
{
  urdf::Model urdf;
  _is_static = _initUrdf(urdf_description, urdf);

  srdf::Model srdf = _initSrdf(srdf_description, urdf);

  // Init RBDL model
  try
  {
    _initModel(_is_static, urdf_description, _model);
  }
  catch (...)
  {
    throw std::invalid_argument(std::string("Error loading model"));
  }

  _initMapNames(urdf.getRoot());

  // init chains from srdf model
  for (const auto& group : srdf.getGroups())
  {
    if (selectors().isDefined(group.name_))
      continue; // maybe some warning

    mwoibn::VectorInt map =
        mwoibn::VectorInt::Zero(getDofs()); // I need the bool maps
    _loadGroup(group, map, srdf.getGroups());

    selectors().add(SelectorMap(group.name_, map));
  }

  state.restart(getDofs());
  command.restart(getDofs());
  lower_limits.restart(getDofs());
  upper_limits.restart(getDofs());

  // read joint limits from urdf
  _readJointLimits(urdf);
  _readGroupStates(srdf);

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

  // start simple mappings
  mwoibn::VectorInt rbdl_map(getDofs());
  for (int i = 0; i < getDofs(); i++)
    rbdl_map[i] = i;

  biMaps().add(BiMap("RBDL", rbdl_map));
  mwoibn::VectorInt all =
      mwoibn::VectorInt::Ones(getDofs()); // I need the bool maps
  selectors().add(SelectorMap("all", all));

  _zeroVec.setZero(getDofs());
}
void mwoibn::robot_class::Robot::_readGroupStates(srdf::Model& srdf)
{
  mwoibn::VectorN state =
      mwoibn::VectorN::Constant(getDofs(), mwoibn::NON_EXISTING);
  mwoibn::VectorInt dofs;
  bool update;
  std::cout << "read group states " << std::endl;

  for (const auto& group : srdf.getGroupStates())
  {
    std::cout << "\t" << group.name_;

    if (groupStates().isDefined(group.name_))
    {
      std::cout << "\tUPDATE" << std::endl;

      state = groupStates().get(group.name_).get();
      update = true;
    }
    else
    {
      std::cout << std::endl;

      state.setConstant(mwoibn::NON_EXISTING);
      update = false;
    }

    for (auto& joint : group.joint_values_)
    {
      dofs = getDof(getLink(joint.first));

      if (dofs.size() == 0)
        std::cout << "WARNING: unknow joint while reading a group state "
                  << group.name_ << std::endl;

      for (int i = 0; i < dofs.size(); i++)
        state[dofs[i]] = joint.second[i];
    }

    if (update)
      groupStates().update(MapState(group.name_, state));
    else
      groupStates().add(MapState(group.name_, state));
  }
}

void mwoibn::robot_class::Robot::_readJointLimits(urdf::Model& urdf)
{

  mwoibn::VectorN limits =
      mwoibn::VectorN::Constant(getDofs(), mwoibn::NON_EXISTING);

  lower_limits.set(limits, INTERFACE::POSITION);
  lower_limits.set(limits, INTERFACE::VELOCITY);
  lower_limits.set(limits, INTERFACE::TORQUE);
  upper_limits.set(limits, INTERFACE::POSITION);
  upper_limits.set(limits, INTERFACE::VELOCITY);
  upper_limits.set(limits, INTERFACE::TORQUE);

  for (int i = 0; i < _model.mBodies.size(); i++)
  {
    if (_model.IsFixedBodyId(i))
    {
      continue;
    }
    std::string link_name = _model.GetBodyName(i);

    if (urdf.getLink(link_name) == NULL)
      continue;

    boost::shared_ptr<const urdf::Joint> joint =
        urdf.getJoint(getJoint(link_name));

    if ((joint->limits) == NULL)
      continue;
    mwoibn::VectorInt dof = getDof(link_name);

    if (joint->limits->lower)
    {
      limits.setConstant(dof.size(), joint->limits->lower);
      lower_limits.set(limits, dof, INTERFACE::POSITION);
    }
    if (joint->limits->upper)
    {
      limits.setConstant(dof.size(), joint->limits->upper);
      upper_limits.set(limits, dof, INTERFACE::POSITION);
    }
    if (joint->limits->velocity)
    {
      limits.setConstant(dof.size(), joint->limits->velocity);
      upper_limits.set(limits, dof, INTERFACE::VELOCITY);
      limits.setConstant(dof.size(), -joint->limits->velocity);
      lower_limits.set(limits, dof, INTERFACE::VELOCITY);
    }
    if (joint->limits->effort)
    {
      limits.setConstant(dof.size(), joint->limits->effort);
      upper_limits.set(limits, dof, INTERFACE::TORQUE);
      limits.setConstant(dof.size(), -joint->limits->effort);
      lower_limits.set(limits, dof, INTERFACE::TORQUE);
    }
  }
}

void mwoibn::robot_class::Robot::_loadGroup(
    const srdf::Model::Group& group, mwoibn::VectorInt& map,
    const std::vector<srdf::Model::Group>& groups)
{

  mwoibn::VectorInt links = getDof(group.links_);
  mwoibn::VectorInt joints = getDof(getLinks(group.joints_));

  for (int i = 0; i < links.size(); i++)
    map[links[i]] = 1;
  for (int i = 0; i < joints.size(); i++)
    map[joints[i]] = 1;

  for (const auto& chain : group.chains_)
  {
    mwoibn::point_handling::RawPositionsHandler my_chain(
        chain.first, _model, std::vector<std::string>{chain.second});
    for (int i = 0; i < my_chain.getChain().size(); i++)
      map[my_chain.getChain()[i]] = 1;
  }

  for (const auto& sub : group.subgroups_)
  {
    // if subgroup is already initialized load dofs
    if (selectors().isDefined(sub))
    {
      mwoibn::VectorInt chainMap = selectors().get(sub).get();
      for (int i = 0; i < chainMap.size(); i++)
      {
        if (chainMap[i])
          map[i] = 1;
      }
    }
    // initialize subgroup
    else
    {
      // check if the subgroup is defined in the srdf
      auto it = std::find_if(groups.begin(), groups.end(),
                             [sub](const srdf::Model::Group& group)
                             {
                               return group.name_ == sub;
                             });
      if (it == groups.end())
      {
        throw(std::invalid_argument(
            "Error loading srdf file: couldn't find the subgroup \"" + sub +
            " \" in the group \"" + group.name_));
      }

      _loadGroup(*it, map, groups);
    }
  }
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

std::vector<std::string> mwoibn::robot_class::Robot::getLinks(std::string chain,
                                                              bool unique)
{
  return getLinks(selectors().get(chain).which(), unique);
}

std::vector<std::string>
mwoibn::robot_class::Robot::getLinks(mwoibn::VectorInt dofs, bool unique)
{
  std::vector<std::string> links;
  bool found;
  for (int i = 0; i < dofs.size(); i++)
  {
    found = false;
    for (int k = 0; k < _model.mJoints.size(); k++)
    {
      if (dofs[i] >= _model.mJoints[k].q_index &&
          dofs[i] < (_model.mJoints[k].q_index + _model.mJoints[k].mDoFCount))
      {
        if (!_is_static && _model.mJoints[k].q_index < 6 &&
            _model.mJoints[k].q_index + _model.mJoints[k].mDoFCount != 6)
        {
          for (int l = k; l < _model.mJoints.size(); l++)
          {
            if (_model.mJoints[l].q_index < 6 &&
                _model.mJoints[l].q_index + _model.mJoints[l].mDoFCount == 6)
            {

              links.push_back(_model.GetBodyName(l));
              found = true;
              break;
            }
          }
        }
        else
        {
          links.push_back(_model.GetBodyName(k));
          found = true;
          break;
        }
      }
    }

    if (!found)
      throw std::invalid_argument(
          std::string("No link is associated with dof ") +
          std::to_string(dofs[i]));
  }

  // ensure uniquness
  if (unique) // return repetitions
    links.erase(std::unique(links.begin(), links.end()), links.end());

  return links;
}

mwoibn::VectorInt mwoibn::robot_class::Robot::getDof(std::string link_name)
{
  int size;
  int init;

  if (_model.GetBodyId(link_name.c_str()) == RBDL_NON_EXISTING)
  {
    return {};
  }

  int nr = _model.GetBodyId(link_name.c_str());

  if (_model.IsFixedBodyId(nr))
  {
    return {};
  }

  RigidBodyDynamics::Joint joint = _model.mJoints[nr];

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
    size = _model.mJoints[nr].mDoFCount;
    init = _model.mJoints[nr].q_index;
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
    dofs.conservativeResize(size);
    dofs.tail(link_dofs.size()) = link_dofs;
  }

  return dofs;
}

YAML::Node
mwoibn::robot_class::Robot::getConfig(const std::string config_file,
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
        secondary_file);
  }
  catch (...)
  {
    throw std::invalid_argument(
        std::string("Unkown error reading the secondary configuration file: ") +
        secondary_file);
  }

  config = config["mwoibn"];
  config_2 = config_2["mwoibn"];

  compareEntry(config, config_2);
  return config;
}

void mwoibn::robot_class::Robot::compareEntry(YAML::Node entry_main,
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
      compareEntry(entry_main[entry.first.as<std::string>()], entry.second);
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
                                      std::string map_name,
                                      std::vector<std::string> names)
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
  return mwoibn::robot_class::BiMap(map_name, map, names);
}

void mwoibn::robot_class::Robot::addBiMap(robot_class::Robot& other,
                                          std::string map_name,
                                          std::vector<std::string> names)
{

  mwoibn::VectorInt map = mwoibn::VectorInt::Constant(getDofs(), NON_EXISTING);

  std::cout << "Robot Handshake: " << map_name << std::endl;

  mwoibn::VectorInt my_dofs, other_dofs;

  for (auto& link : other.getLinks(other.biMaps().get("RBDL").get(), true))
  {

    try
    {
      my_dofs = getDof(link);
      other_dofs = other.getDof(link);

      if (my_dofs.size() && my_dofs.size() != other_dofs.size())
        throw(std::invalid_argument(
            "size of " + link +
            " is different for two models unique mapping is not defined"));
    }
    catch (const std::out_of_range& e)
    {
      //        map[i] = robot_class::NON_EXISTING;
    }

    for (int k = 0; k < my_dofs.size(); k++)
      map[my_dofs[k]] = other_dofs[k];
  }

  std::cout << map.transpose() << std::endl;
  biMaps().add(BiMap(map_name, map, names));
}

mwoibn::robot_class::BiMap
mwoibn::robot_class::Robot::readBiMap(YAML::Node config)
{
  if (!config["chain"])
    throw(
        std::invalid_argument("Required element chain has not been defined."));

  if (!config["type"])
    throw(std::invalid_argument("Required element type has not been defined."));

  if (!selectors().isDefined(config["chain"].as<std::string>()))
    throw(std::invalid_argument("Chain." + config["chain"].as<std::string>() +
                                " has not been defined in the robot model"));

  mwoibn::VectorBool chain = mwoibn::VectorBool::Constant(getDofs(), false);
  mwoibn::VectorInt chainInt =
      selectors().get(config["chain"].as<std::string>()).get();

  for (int i = 0; i < chainInt.size(); i++)
    if (chainInt[i])
      chain[i] = true;

  //  if (config["chain"].as<std::string>() == "all")
  //    chain.setConstant(true);
  //  else
  //    throw(std::invalid_argument(
  //        "Support for chains has not been implemented yet"));

  mwoibn::VectorBool type = mwoibn::VectorBool::Constant(getDofs(), false);
  if (config["type"].as<std::string>() == "all")
    type.setConstant(true);
  else if (config["type"].as<std::string>() == "actuated")
  {
    type = actuators().getActuationTypes({ACTUATOR_TYPE::UNACTUATED});
    type = eigen_utils::flip(type);

    if (type.size() != getDofs())
      throw(std::invalid_argument("Couldn't initialize mapping for actuated "
                                  "dofs, actuation data has not been loaded."));
  }

  else if (config["type"].as<std::string>() == "body")
  {
    if (!config["name"])
      throw(std::invalid_argument("Couldn't initialize mapping to the body, "
                                  "no [body][name] defined."));
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
      compareEntry(entry.second, config[entry.first.as<std::string>()]);
  }
}

YAML::Node mwoibn::robot_class::Robot::readFullConfig(YAML::Node full_config, std::string config_name){
  if (!full_config["robot"][config_name])
    throw std::invalid_argument(std::string("Unknown configuration: ") +
                                config_name);
  if (!full_config["robot"]["systems"])
    throw std::invalid_argument(std::string(
        "Systems configuration has not been defined for the robot " +
        config_name));
  if (!full_config["robot"]["systems"][full_config["system"].as<std::string>()])
    throw std::invalid_argument(
        full_config["system"].as<std::string>() +
        std::string(" configuration has not been defined for the robot ") +
        config_name);
  if (!full_config["robot"]["layer"])
    throw std::invalid_argument(std::string("Please specify robot layer."));
  if (!full_config["layers"])
    throw std::invalid_argument(
        std::string("Layers have not been configured for the robot."));
  if (!full_config["layers"][full_config["robot"]["layer"]
                                          .as<std::string>()])
    throw std::invalid_argument(
        std::string("Layers have not been configured for the robot."));
  if (!full_config["robot"]["mode"])
    throw std::invalid_argument(
        std::string("Please specify robot operational mode."));

  // get specific configuration
  std::cout << "\t system\t" << full_config["system"].as<std::string>() << std::endl;
  std::cout << "\t layer\t" << full_config["robot"]["layer"].as<std::string>() << std::endl;
  std::cout << "\t mode\t" << full_config["robot"]["mode"].as<std::string>() << std::endl;

  std::string system_file = readPath(
      full_config["robot"]["systems"][full_config["system"].as<std::string>()]);
  YAML::Node system_config = getConfig(system_file);

  std::string layer_file = readPath(
      full_config["layers"][full_config["robot"]["layer"].as<std::string>()]);
  YAML::Node layer_config = getConfig(layer_file);

  mwoibn::robot_class::Robot::compareEntry(full_config, system_config);
  mwoibn::robot_class::Robot::compareEntry(full_config, layer_config);


  if (full_config["robot"]["rate"]){
    full_config["robot"][config_name]["rate"] = full_config["robot"]["rate"];
    std::cout << "\t rate\t" << full_config["robot"]["rate"].as<double>() << " Hz" << std::endl;
  }
  else
    std::cout << "\t rate\t undefiend" << std::endl;
  return full_config;
}

YAML::Node
mwoibn::robot_class::Robot::_readRobotConfig(YAML::Node full_config,
                                             std::string config_name)
{
  full_config = readFullConfig(full_config, config_name);
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
    config["controller"]["mode"] = full_config["robot"]["mode"];
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
    if (!lists[name])
    {
      std::cout << "No feedback has been defined for this configuration"
                << std::endl;
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

  int ref = _model.GetBodyId(config["dofs"]["name"].as<std::string>().c_str());

  while (ref != 0)
  {
    offset_position += _model.GetJointFrame(ref).r;
    offset_orientation =
        offset_orientation * _model.GetJointFrame(ref).E.transpose();
    ref = _model.GetParentBodyId(ref);
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
  if (angels)
  {

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

void mwoibn::robot_class::Robot::_loadMappings(YAML::Node config)
{

  //  std::cout << "mappings" << std::endl;
  for (auto entry : config)
  {
    //    std::cout << entry.first << std::endl;

    if (biMaps().isDefined(entry.first.as<std::string>()))
    {
      std::cout << "Map " + entry.first.as<std::string>() +
                       " has been previously initilized, skip this map"
                << std::endl;
      continue;
    }

    entry.second["name"] = entry.first.as<std::string>();
    _loadMap(entry.second);
  }
}

void mwoibn::robot_class::Robot::_loadMap(YAML::Node config)
{

  //  std::cout << "load map" << std::endl;
  if (!config["name"])
    throw(std::invalid_argument("Please define a mapping name"));

  if (biMaps().isDefined(config["name"].as<std::string>()))
    return;

  if (!config["loading"])
    throw(std::invalid_argument("Please defined a mapping loading method."));

  if (config["loading"].as<std::string>() == "model")
    _loadMapFromModel(config);

  else
    throw(std::invalid_argument("Unknown loading method for mapping " +
                                config["name"].as<std::string>()));

  std::cout << "Map " << config["name"].as<std::string>()
            << " has been sucesfully loaded." << std::endl;
}

std::string mwoibn::robot_class::Robot::_readUrdf(YAML::Node config)
{

  if (!config["urdf"])
    throw std::invalid_argument(
        std::string("Please define an urdf source in the yaml file."));

  if (!config["urdf"]["file"])
    throw(std::invalid_argument(
        "Please define an urdf source in the yaml file.\n"));

  /*std::string file = "";
  if (config["urdf"]["path"])
    file = config["urdf"]["path"].as<std::string>();

  file += config["urdf"]["file"].as<std::string>();

  std::cout << "from file:\t" << file << std::endl;

  return file;
  */
  return readPath(config["urdf"]);
}

std::string mwoibn::robot_class::Robot::readPath(YAML::Node config)
{

  std::string file = "";
  if (config["path"])
    file = config["path"].as<std::string>();

  file += config["file"].as<std::string>();

  std::cout << "from file:\t" << file << std::endl;

  return file;
}

std::string mwoibn::robot_class::Robot::_readSrdf(YAML::Node config)
{

  if (!config["srdf"])
    return "";
  if (config["srdf"]["file"].as<std::string>() == "")
    return "";

  return readPath(config["srdf"]);
}

bool mwoibn::robot_class::Robot::_initUrdf(std::string& urdf_description,
                                           urdf::Model& urdf)
{

  if (!urdf.initFile(urdf_description))
  {
    throw(
        std::invalid_argument(std::string("Could not load urdf description")));
  }

  return (urdf.getRoot()->child_joints[0]->type == urdf::Joint::FLOATING)
             ? false
             : true;
}

srdf::Model mwoibn::robot_class::Robot::_initSrdf(std::string& srdf_description,
                                                  urdf::Model& urdf)
{
  srdf::Model srdf;

  if (srdf_description == "")
  {
    std::cout << " WARNING: srdf file has not been defined, it will not be "
                 "initialized" << std::endl;
    return srdf;
  }

  if (!srdf.initFile(urdf, srdf_description))
  {
    throw(std::invalid_argument("Could not load srdf description"));
  }

  return srdf;
}

void mwoibn::robot_class::Robot::_initModel(bool is_static,
                                            const std::string& source,
                                            RigidBodyDynamics::Model& model)
{
  // RigidBodyDynamics::Model model;

  if (!RigidBodyDynamics::Addons::URDFReadFromFile(source.c_str(), &model,
                                                   !is_static, false))
    throw std::invalid_argument(
        std::string("Error loading model from file  for mapping "));
}

void mwoibn::robot_class::Robot::_loadMapFromModel(YAML::Node config)
{

  if (!config["urdf"])
    throw(std::invalid_argument("Please define a urdf source from mapping." +
                                config["name"].as<std::string>()));

  urdf::Model urdf;

  std::string source = _readUrdf(config);

  bool is_static;
  try
  {
    is_static = _initUrdf(source, urdf);
  }
  catch (const std::invalid_argument& e)
  {
    throw(std::invalid_argument(e.what() + std::string(" for mapping: ") +
                                config["name"].as<std::string>()));
  }

  RigidBodyDynamics::Model model;
  // Init RBDL model
  try
  {
    _initModel(is_static, source, model);
  }
  catch (...)
  {
    throw std::invalid_argument(
        std::string("Error loading model  for mapping " +
                    config["name"].as<std::string>()));
  }

  mwoibn::VectorInt map =
      mwoibn::VectorInt::Constant(model.dof_count, mwoibn::NON_EXISTING);

  for (const auto& link : model.mBodyNameMap)
  {
    std::string name = link.first;

    mwoibn::VectorInt dofs = getDof(name);

    if (model.IsFixedBodyId(link.second))
      continue;
    RigidBodyDynamics::Joint joint = model.mJoints[link.second];

    if (!_is_static && joint.q_index < 6 &&
        joint.q_index + joint.mDoFCount == 6)
    {

      map.segment(0, 6) = dofs;
      continue;
    }

    if (joint.mDoFCount != dofs.size())
    {
      std::cout << "size of body " << name << " is different for both models. "
                                              "It cannot be controlled using "
                                              "model mapping" << std::endl;
      continue;
    }

    map.segment(joint.q_index, joint.mDoFCount) = dofs;
  }

  mwoibn::VectorInt mapNew =
      mwoibn::VectorInt::Constant(getDofs(), mwoibn::NON_EXISTING);

  for (int i = 0; i < map.size(); i++)
  {
    if (map[i] < getDofs())
      mapNew[map[i]] = i;
  }

  biMaps().add(BiMap(config["name"].as<std::string>(), mapNew));
}
