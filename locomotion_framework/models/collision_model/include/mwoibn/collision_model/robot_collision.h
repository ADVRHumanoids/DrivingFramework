#ifndef COLLISION_MODEL_UNIVERSE_H
#define COLLISION_MODEL_UNIVERSE_H
#include "mwoibn/collision_model/collision_model.h"

#include <sch/S_Object/S_Sphere.h>
#include <sch/S_Object/S_Box.h>
#include <sch/S_Object/S_Superellipsoid.h>
#include <sch/STP-BV/STP_BV.h>
#include <sch/CD/CD_Pair.h>
#include <sch/CD/CD_Scene.h>
#include <sch/S_Polyhedron/S_Polyhedron.h>
#include <sch/STP-BV/STP_BV_P.h>

#include <rbdl/rbdl.h>
#include <cmath>

#include "mwoibn/point_handling/robot_points_handler.h"
#include "mwoibn/robot_class/robot.h"

namespace mwoibn
{
  namespace collision_model
  {

    //! Keeps the collision model of a robot. It realise on the sch-core library
    /**
     *
     * \todo add an option of adding/removing a collision pair from the model
     *(To support external objects)
     * \todo add an option of adding/removing an object from the model (To
     *support external objects)
     * \todo add custom model param (as an imput parameter) + throw error if
     *neccessary (what?)
     *
     */
    class RobotCollision
    {
    public:
      //! In this structure basic properties of a collision model of a single
      //link is kept
      struct Object
      {
        std::string link; //!< name of the link in the rbdl/urdf
        sch::S_Object*
            objectPtr; //!< pointer to the colision model in the sch library
        mwoibn::Quaternion orientation; //!< default
                                                         //orientation of the
                                                         //link with respect to
                                                         //the reference frame
                                                         //origin
      };

      //! Keeps the data specific to the collision pair
      struct Pair
      {
        sch::CD_Pair* pair;   //!< pointer to the pair model in the sch library
        sch::Scalar distance; //!< last computed distance between the pairs
        int link_1;           //!< id number of a first link in _objects
        int link_2;           //!< id number of a second link in _objects
        sch::Point3 witness_point_1; //!< last computed position of a witness
                                     //point on first link
        sch::Point3 witness_point_2; //!< last computed position of a witness
                                     //point on second link
      };

      RobotCollision(mwoibn::robot_class::Robot& robot, std::string urdf_file,
                     std::string srdf_file = "", std::string package_path = "")
          : _robot(robot)
      {

        //! read robot model/configuration from urdf/srdf
        /*
         * @param[in] urdf_file URDF description of the robot parsed in string
         * @param[in] srdf_file SRDF description of the robot parsed in string.
         * SRDF file is used to disable not considered collision pairs
         * @param[in] package_path optional argument, providing path to the ros
         *package
         * from which the paths to mesh files are defined in URDF file
         *
         * @note to eliminate all direct ros dependencies the _readMesh function
         * uses predefined path to the ros package. As a consequence all paths
         * to the mesh files have to be placed in the same package, even though
         * in the URDF format they can be placed in different packages.
         * Alternatively, the absolute paths can be provided.
         *
         *
         */
        _readMesh(urdf_file, srdf_file, package_path);

        _dof = _objects.size();
        _pairs_number = _pairs.size();

        //! updates placements of the collision model based on
        updatePositions();
      }

      virtual ~RobotCollision()
      {
        for (auto& pair : _pairs)
        {
          delete pair.pair;
        }

        for (auto& object : _objects)
        {
          delete object.objectPtr;
        }
      }

      //! Initialize an instance of the Object struct
      /**
       * @param[in] obj pointer to the sch object
       * @param[in] name link rbdl name
       */
      void addObject(sch::S_Object* obj, std::string name);

      //! Initialize an instance of the Object struct with non-zero initial
      //rotation
      /**
       * @param[in] obj pointer to the sch object
       * @param[in] name link rbdl name
       * @param[in] orientation fixed orientation of the Object w.r.t. to the
       * reference frame expressed in quaternion
       */
      void addObject(sch::S_Object* obj, std::string name,
                     mwoibn::Quaternion orientation);

      //! Initialize an instance of the Pair struct
      /**
       * @param link1 rbdl name of a DOF associated with the first link
       * @param link2 rbdl name of a DOF associated with the second link
       */
      void addPair(std::string link1, std::string link2);

      //! Updates position of all objects and recomputes the distances for all
      //pairs
      int updateCollisions();

      //! Recomputes the distance for a specific pair
      /**
       * @return number of collisions detected
       *
       * @todo remove return
       * @note This function does not update the forward kinematics before
       *recomputing the distance
       */
      bool updateCollision(int i);

      //! returnes the copy of a given Obejct
      Object getObject(int i) { return _objects[i]; }

      //! returnes the copy of a given Pair
      Pair getPair(int i) { return _pairs[i]; }

      //! Returnes the vector of last known distances for all pairs
      /**
       * The data are returned in the same oreder as pairs are defined
       */
      mwoibn::VectorN getDistances();

      //! returnes number of consider objects with the collision model
      int getObjectsNumber() { return _dof; }

      //! returnes number of considered collision pairs
      int getPairsNumber() { return _pairs_number; }

      std::unique_ptr<mwoibn::point_handling::PositionsHandler> object_ik_ptr;
      std::unique_ptr<mwoibn::point_handling::PositionsHandler> pair_ik_ptr;

      //! returns number of real robot dofs
      /**
       * this function provides an access to mwoibn::robot_class::Robot::getDofs()
       * function from the Eobot class
       */
      int getRobotDofs() { return _robot.getDofs(); }

      //! returns real robot joint_states
      /**
       * this function just provides the access to
       * mwoibn::robot_class::Robot::getJointStates() function from the Robot class
       */
      mwoibn::VectorN getJointStates()
      {
        return _robot.state.get(mwoibn::robot_class::INTERFACE::POSITION);
      }

      //! updates position of all objects in the collision model
      virtual void updatePositions();

      //! updates position of a objects in the collision model refered by a
      //respective number of a DOF in rbdl
      virtual void updatePosition(int i);

      void updatePointHendler(int i);

    protected:
      //! Basic class initialization
      /**
       */
      RobotCollision(mwoibn::robot_class::Robot& robot) : _robot(robot) {}

      std::vector<Object> _objects; //!< Keeps all the Objects associated with
                                    //the collision model
      std::vector<Pair>
          _pairs; //!< Keeps all the Pairs considered in the collision model

      mwoibn::robot_class::Robot& _robot;
      //! Number of Objects in the collision model
      /**
       * \sa getObjectsNumber()
       */
      int _dof = 0;

      //! Keeps the name of the model reference frame
      /**
       * @note it is needed as rbdl differently computes the base position for
       * the static and floating_base models
       */
      std::string _reference;
      //! Number of Pairs considered in the collision model
      /**
       * \sa getPairsNumber()
       */
      int _pairs_number = 0;

      //! Initialize robot model
      bool _readMesh(std::string urdf_file, std::string srdf_file = "",
                     std::string package_path = "");
    };
  } // namespace package
} // namespace library
#endif
