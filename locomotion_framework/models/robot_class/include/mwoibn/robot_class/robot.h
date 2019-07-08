#ifndef ROBOT_CLASS_ROBOT_H
#define ROBOT_CLASS_ROBOT_H

//#include <rbdl/rbdl.h>
#include "mwoibn/robot_class/robot_class.h"
#include "mwoibn/robot_points/contact.h"
#include "mwoibn/robot_points/wheel_contact.h"
#include "mwoibn/robot_points/wheel_contact_v3.h"
#include "mwoibn/robot_points/point_contact.h"

#include "mwoibn/robot_points/contact_v2.h"

#include "mwoibn/robot_class/contacts_2.h"

#include "mwoibn/communication_modules/communications.h"
#include "mwoibn/robot_class/actuators.h"
#include "mwoibn/robot_points/center_of_mass.h"

#include "mwoibn/robot_points/center_of_pressure.h"

#include "mwoibn/communication_modules/shared.h"
#include "mwoibn/communication_modules/shared_all.h"
#include "mwoibn/common/stack.h"

#include "mwoibn/common/all.h"
#include "mwoibn/robot_class/mappings.h"
#include "mwoibn/robot_class/map.h"
#include <memory>
#include <boost/bimap.hpp>
#include <urdf/model.h>
#include <srdfdom/model.h>

namespace mwoibn
{

namespace robot_class
{

//! This class provides the robot abstaraction for a robot characterisitcs.
/**
 * It provides a bridge between
 * the simulation/hardware and the robot software for the rbdl library
 * Child classes are designed to implement any freamwork dependent
 * functions.
 * Inheriting classes shall be implemented in the separated
 * files/wrapped
 * The class can be used through the basic class providing model updates within
 * the written code, alternativelly robot updates can be provided by the
 * inheriting classes
 */
class Robot
{
public:
//! Initializer of the base robot class
/**
 * \param[in] modelPtr initialized RBDL model
 * \param[in] is_static information about the model type is required as there
 * are inconsistencies in the output from certain RBDL functions between two
 * types of models
 */
Robot(std::string urdf_description, std::string srdf_description = "");

virtual ~Robot() {
}

//! Returnes whether the robot is floating base or not
bool isStatic() {
        return _is_static;
}

//! Returns number of actuated(?) DOFs
int getDofs() {
        return _model.dof_count;
}

virtual double rate() {
        return _rate;
}

virtual void setRate(double rate){
        _rate = rate;
}

//! Returns pointer to the robot model
RigidBodyDynamics::Model& getModel() {
        return _model;
}

virtual void update() {
        send();

        get();
        updateKinematics();
        wait();
}

void updateKinematics()
{
//    RigidBodyDynamics::UpdateKinematicsCustom(
//        _model, &state.state("POSITION"),
//        &state.state("VELOCITY"), nullptr);
        if(!kinematics_update.get()) return;
        RigidBodyDynamics::UpdateKinematics(
                _model, state.position.get(), state.velocity.get(), _zeroVec);
        kinematics_update.set(false);
}

virtual bool isRunning() {
        return true;
}
virtual bool get(){
        return feedbacks.run();
}
virtual bool send(){
        return controllers.run();
}
virtual void wait(bool spin = true){
  kinematics_update.set(true);
}
///@}
///
/** @brief Actuation
 *  Functions releated to hendle robot actuators
 */
Actuators& actuators() {
        return _actuators;
}
Mappings<BiMap>& biMaps() {
        return _bi_maps;
}

Mappings<SelectorMap>& selectors() {
        return _selector_maps;
}
Mappings<MapState>& groupStates() {
        return _group_states;
}

const mwoibn::VectorInt& getActuationState() {
        return _actuation;
}

/** @brief CenterOfMass
 *  Functions releated to hendle robot's center of mass
 */
robot_points::CenterOfMass& centerOfMass() {
        return *(_center_of_mass.get());
}

robot_points::CenterOfPressure& centerOfPressure() {
        return *(_center_of_pressure.get());
}

Contacts2& contacts() {
        return *(_contacts.get());
}
//! Keeps robot states
robot_class::State state;
robot_class::State motor;

//! Keeps current commands to be send to the robot
robot_class::State command;

//! keep state limits
robot_class::State lower_limits;   // read from urdf?
robot_class::State upper_limits;
common::Stack<std::string, robot_class::State> states;
/** @brief Keeps pointers for all external controllers */
communication_modules::Communications controllers;
communication_modules::Communications feedbacks;

/** @brief return vector of links associated with given dofs**/
/** @param dofs - vector of model dofs
 *  @param unique - true: each link appears only ones, false: link is associated with each dof
 */
std::vector<std::string> getLinks(mwoibn::VectorInt dofs, bool unique = true);

std::vector<std::string> getLinks(std::string chain, bool unique = true);

std::string getLinks(unsigned int dof);

mwoibn::Vector3 gravity(){return _model.gravity;}
/** @brief return rbdl state vetor dofs associated with given links **/
/** @note For floating base system it gathers all flaoting base dofs in the
 * last
 * link, for previous joints it returns en empty vector
 */
mwoibn::VectorInt getDof(std::string link_name);

std::string getBodyName(int id){
    return _model.GetBodyName(id);
}

std::vector<std::string> getBodyName(mwoibn::VectorInt ids){
    std::vector<std::string> links;

    for(int i = 0; i < ids.size(); i++)
      links.push_back(getBodyName(ids[i]));

    return links;
}

unsigned int getBodyId(std::string link_name){
    return mwoibn::rbdl_utils::checkBody(link_name, _model);
}

mwoibn::VectorInt getBodyId(std::vector<std::string> link_names){
    mwoibn::VectorInt links(link_names.size());

    for(int i = 0; i < link_names.size(); i++)
      links[i] = mwoibn::rbdl_utils::checkBody(link_names[i], _model);

    return links;
}

/** @brief return rbdl state vetor dofs associated with given links **/
mwoibn::VectorInt getDof(std::vector<std::string> link_names);

/** @brief returns a name of a link associated with the same dof as given
 * joint name
 * @return name of a link, if the link doesn't exist in a RBDL model an
 * std::out_of_range exceptioin is thrown
 */
std::string getLink(std::string joint_name);
/** @brief returns name for link in joint_names vector. Returns an empty
 * string if joint does not exist in a model **/
std::vector<std::string>
getLinks(const std::vector<std::string>& joint_names);
/** @brief returns a name of a joint associated with the same dof as given
 * link name
 * @return name of a joint, if the joint doesn't exist in a RBDL model an
 * std::out_of_range exceptioin is thrown
 */
std::string getJoint(std::string link_name);
/** @brief returns name for joints in link_names vector. Returns an empty
 * string if joint does not exist in a model **/
std::vector<std::string>
getJoints(const std::vector<std::string>& link_names);

robot_class::BiMap readBiMap(YAML::Node config);

static std::string readPath(YAML::Node config);
/** @brief creates a map from a vector of links in a new map
 */
robot_class::BiMap makeBiMap(
        std::vector<std::string> link_names,
        std::string map_name, std::vector<std::string> names = {}); // for making a map from joints I can use getDof

void addBiMap(
        robot_class::Robot& other,
        std::string map_name, std::vector<std::string> names = {}); // for making a map from joints I can use getDof

static YAML::Node getConfig(const std::string config_file,
                            const std::string secondary_file = "");
static void compareEntry(YAML::Node entry_main, YAML::Node entry_second);

static YAML::Node readFullConfig(YAML::Node full_config, std::string config_name);

virtual void loadControllers(std::string config_file, std::string config_name,
                     mwoibn::communication_modules::Shared& shared,
                     std::string controller_source, std::string secondary_file){
                       std::cout << "base loads controllers" << std::endl;
                     }


virtual void loadControllers(YAML::Node full_config, std::string config_name,
                     mwoibn::communication_modules::Shared& shared,  std::string controller_source){
                       std::cout << "base loads controllers" << std::endl;
                     }


std::string name(){return _name;}


virtual void srdfContactGroup(const std::string& srdf_group){
  if(contacts().hasGroup(srdf_group)) return;

  std::vector<std::string> names = getLinks(srdf_group);

  // add wheels to the contact group
  for(auto& name: names){
      auto contact = ranges::find_if(contacts(), [&](auto& contact)-> bool{return getBodyName(contact->wrench().getBodyId()) == name;});
      if ( contact != ranges::end(contacts()) )
          contacts().toGroup((*contact)->getName(), srdf_group);
  }

}

common::Flag  kinematics_update;



protected:
//! Alternative robot class initializer,
/**  it is provited as protected method as it should only be used by the
 * inheriting
 *  functions providing the rbdl model and _is_static paramters inside its
 * initializers
 */
Robot() {
}

//! Keeps pointer to the RBDL model
RigidBodyDynamics::Model _model;

//! Keeps information whether the robot is static or not
bool _is_static;
std::string _name;
double _rate = 0;
//! Keeps data about considered contacts
std::unique_ptr<Contacts2> _contacts;

//! Kepps center of mass data
std::unique_ptr<robot_points::CenterOfMass> _center_of_mass;
std::unique_ptr<robot_points::CenterOfPressure> _center_of_pressure;

//! Actuators Module
Actuators _actuators;

//! Mappings Module
Mappings<BiMap> _bi_maps;
Mappings<SelectorMap> _selector_maps;
Mappings<MapState> _group_states;

/** @brief Keeps information about robot actuation type
 *
 * the convention is:
 *  for acutated joint: positive number
 *  unactuated joints: 0
 *
 * More specialized classes may add more specific infomration (reference
 ***controller, broken joints)
 */
Eigen::VectorXi _actuation;

//! Internal initialization function
void _init(std::string urdf_description, std::string srdf_description);

//! keeps link - joint mapping
typedef boost::bimap<boost::bimaps::set_of<std::string>,
                     boost::bimaps::set_of<std::string> > boost_map;
boost_map _map_names;
void _initMapNames(boost::shared_ptr<const urdf::Link> link);

virtual std::string _readUrdf(YAML::Node config);
virtual std::string _readSrdf(YAML::Node config);

virtual bool _initUrdf( std::string& urdf_description, urdf::Model& urdf);
virtual srdf::Model _initSrdf( std::string& srdf_description, urdf::Model& urdf);

virtual void _initModel(bool is_static, const std::string& source, RigidBodyDynamics::Model& model);
virtual void _initContactsCallbacks(YAML::Node config, mwoibn::communication_modules::Shared& shared);
virtual void _initContactsCallbacks(YAML::Node config);



virtual void _loadContacts(YAML::Node contacts_config);
virtual void _loadActuators(YAML::Node actuators_config);
virtual void _loadConfig(YAML::Node config, YAML::Node robot);

virtual void _loadFeedbacks(YAML::Node config) {
}
virtual void _shareFeedbacks(YAML::Node config, mwoibn::communication_modules::Shared& shared);
virtual void _shareControllers(YAML::Node config, mwoibn::communication_modules::Shared& shared);

virtual void _loadControllers(YAML::Node config) {
}

virtual void _loadMappings(YAML::Node config);
virtual void _loadMap(YAML::Node config);
virtual void _loadMapFromModel(YAML::Node config);


YAML::Node _readRobotConfig(YAML::Node full_config,
                            std::string config_name, std::string controller_source);
YAML::Node _readConfig(const YAML::Node lists, const YAML::Node defined,
                       YAML::Node config);
void _readJointLimits(urdf::Model& urdf);
void _readGroupStates(srdf::Model& srdf);



void _getDefaultPosition(YAML::Node config, bool position, bool orientation,
                         bool angels);
bool _loadFeedback(YAML::Node entry, std::string name);

mwoibn::VectorN _zeroVec;

void _loadGroup(const srdf::Model::Group& group, mwoibn::VectorInt& map, const std::vector<srdf::Model::Group>& groups);
virtual std::unique_ptr<mwoibn::communication_modules::CommunicationBase> _generateContactCallback(mwoibn::robot_points::Contact& contact, YAML::Node config, mwoibn::communication_modules::Shared& shared);
virtual std::unique_ptr<mwoibn::communication_modules::CommunicationBase> _generateContactCallback(mwoibn::robot_points::Contact& contact, YAML::Node config){return nullptr;}
};
} // namespace package
} // namespace library

#endif
