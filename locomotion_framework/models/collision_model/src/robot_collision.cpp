//#define TIME_CHECK
#include "mwoibn/collision_model/robot_collision.h"
#include <iostream>
#include <algorithm>
#include <urdf/model.h>
#include <srdfdom/model.h>

#ifdef TIME_CHECK
#include <chrono>
#endif

using namespace sch;

bool mwoibn::collision_model::RobotCollision::_readMesh(std::string urdf_file,
                                                        std::string srdf_file,
                                                        std::string package_path)
{

        urdf::Model urdf;

        if (!urdf.initString(urdf_file))
        {
                throw(std::invalid_argument("Error, given urdf file is not valid."));
        }

        typedef std::vector<boost::shared_ptr<urdf::Link> > V_Link;
        V_Link links;
        urdf.getLinks(links);

//  if (!_robot.isStatic())
//  {
        _reference = "ROOT";
/*  }
   else
   {
    auto it = std::find_if(links.begin(), links.end(),
                           [](boost::shared_ptr<urdf::Link>& Link)
                           {
                             return Link->getParent()->name == "world";
                           });
    if (it == links.end())
      LOG_INFO << "robot base not found, trying to continue";
    else
    {
      _reference = (*it)->name;
    }
   }*/

        std::vector<std::string> body_names;
        std::vector<mwoibn::Vector3> P;
        mwoibn::Vector3 rbdl_position;

        //mwoibn::Matrix P;
        int j = 0;

        for (int i = 0; i < links.size(); i++)
        {
                if (links[i]->collision != nullptr)
                {
                        boost::shared_ptr<urdf::Collision> vs = links[i]->collision;
                        LOG_INFO << links[i]->name << "\n";
                        boost::shared_ptr<urdf::Geometry> gm = vs->geometry;
                        urdf::Vector3 u_position = vs->origin.position;
                        urdf::Rotation u_orientation = vs->origin.rotation;
                        rbdl_position << u_position.x, u_position.y, u_position.z;
                        P.push_back(rbdl_position);
                        body_names.push_back(links[i]->name);

                        mwoibn::Quaternion quaternion(u_orientation.x, u_orientation.y, u_orientation.z,
                                                      u_orientation.w);

                        switch (gm->type)
                        {
                        case urdf::Geometry::BOX:
                        {
                                boost::shared_ptr<urdf::Box> p =
                                        boost::static_pointer_cast<urdf::Box>(gm);
                                urdf::Vector3 vc = p->dim;
                                //					S_Box * box = new S_Box
                                //(vc.x,vc.y,vc.z);
                                S_Superellipsoid* box =
                                        new S_Superellipsoid(0.5 * vc.x, 0.5 * vc.y, 0.5 * vc.z, 0.1, 0.1);
                                addObject(box, links[i]->name, quaternion);
                                std::cout << "x " << vc.x << ", y " << vc.y << ", z " << vc.z
                                          << std::endl; // verbose
                                j++;
                                break;
                        }
                        case urdf::Geometry::CYLINDER:
                        {
                                boost::shared_ptr<urdf::Cylinder> p =
                                        boost::static_pointer_cast<urdf::Cylinder>(gm);
                                //					S_Box * box = new S_Box
                                //(vc.x,vc.y,vc.z);
                                S_Superellipsoid* cylinder = new S_Superellipsoid(
                                        p->radius, p->radius, 0.5 * p->length + p->radius, 0.75, 1);

                                addObject(cylinder, links[i]->name, quaternion);
                                std::cout << "radious " << p->radius << ", length " << p->length
                                          << std::endl; // verbose
                                j++;
                                break;
                        }
                        case urdf::Geometry::SPHERE:
                                LOG_INFO << links[i]->name
                                         << "is a sphere, adding spheres objects is not implemented yet"
                                         << "\n";
                                body_names.pop_back();
                                P.pop_back();

                                break;

                        case urdf::Geometry::MESH:
                        {
                                boost::shared_ptr<urdf::Mesh> p =
                                        boost::static_pointer_cast<urdf::Mesh>(gm);
                                STP_BV* mesh = new STP_BV();
                                std::string resource = p->filename;
                                resource.replace(resource.end() - 3, resource.end(), "txt");
                                std::string pkg = "";

                                std::string identifier = "package://";
                                if (resource.substr(0, identifier.size()) == identifier)
                                {
                                        pkg = resource.substr(identifier.size());
                                        resource = resource.substr(identifier.size());
                                        std::size_t pos = resource.find_first_of('/');
                                        resource = resource.substr(pos);

                                        //         std::cout << "resource: " << resource << std::endl;
                                        pkg = package_path; // this means in urdf file all collision mesh
                                                            // files have to be kept in the same package or
                                                            // refered by absolute paths
                                                            //         pos = pkg.find_first_of('/');
                                                            //         pkg = pkg.substr(0, pos);
                                                            //         try
                                                            //         {
                                                            //           pkg = ros::package::getPath(pkg);
                                                            //         }
                                                            //         catch (...)
                                                            //         {
                                                            //           throw;
                                                            //         }
                                }

                                try
                                {
                                        if (p->scale.y < 0)
                                        {
                                                try
                                                {
                                                        mesh->constructFromFile(
                                                                pkg + resource.substr(0, resource.size() - 4) + "_r.txt");
                                                }
                                                catch (std::exception& e)
                                                {
                                                        mesh->constructFromFile(pkg + resource);
                                                }
                                        }
                                        else
                                                mesh->constructFromFile(pkg + resource);
                                }
                                catch (std::exception& e)
                                {
                                        throw(std::invalid_argument("Error, file: " + resource +
                                                                    " does not exist."));
                                }

                                addObject(mesh, links[i]->name, quaternion);
                                LOG_INFO << links[i]->name << std::endl;
                                j++;
                                break;
                        }

                        default:
                                LOG_INFO << "unknown type" << links[i]->name << "\n";
                                body_names.pop_back();
                                P.pop_back();
                                break;
                        }
                }
                else
                {
                        LOG_INFO << links[i]->name << " doesn't have a collision element"
                                 << "\n";
                }
        }

        LOG_INFO << "objects, making objects" << std::endl;

        object_ik_ptr.reset(new mwoibn::point_handling::PositionsHandler(_reference, _robot, body_names, P));

        srdf::Model srdf;
        std::vector<srdf::Model::DisabledCollision> collisions;

        if (srdf_file.empty())
                LOG_INFO
                << "Recived an empty srdf file, consider all collision pairs valid";
        else
        {
                if (!srdf.initString(urdf, srdf_file))
                        LOG_INFO << "Couldn't read srdf file, consider all collision pairs valid";
                else
                {
                        collisions = srdf.getDisabledCollisionPairs();
                }
        }
        for (auto link1_it = _objects.begin(); link1_it != _objects.end(); ++link1_it)
        {
                std::string link1 = (*link1_it).link;
                for (auto link2_it = link1_it + 1; link2_it != _objects.end(); ++link2_it)
                {
                        std::string link2 = (*link2_it).link;
                        if (link1 != link2)
                        {
                                auto check_collision_ptr = std::find_if(
                                        collisions.begin(), collisions.end(),
                                        [&](srdf::Model::DisabledCollision collision)
                                        {
                                                return (collision.link1_ == link1 && collision.link2_ == link2) ||
                                                (collision.link1_ == link2 && collision.link2_ == link1);
                                        });
                                if (check_collision_ptr == collisions.end())
                                {
                                        addPair(link1, link2);
                                }
                        }
                }
        }

        body_names.clear();

        for (int i = 0; i < _pairs.size(); i++)
        {
/*  if (_robot.isStatic() && _objects[_pairs[i].link_1].link == _reference &&
      _robot.getModel().GetBodyId(_reference.c_str()) > _dof)
    body_names.push_back("ROOT");
   else */
                body_names.push_back(_objects[_pairs[i].link_1].link);

/*  if (_robot.isStatic() && _objects[_pairs[i].link_2].link == _reference &&
      _robot.getModel().GetBodyId(_reference.c_str()) > _dof)
    body_names.push_back("ROOT");
   else */
                body_names.push_back(_objects[_pairs[i].link_2].link);
        }

        pair_ik_ptr.reset(new mwoibn::point_handling::PositionsHandler(_reference, _robot, body_names));

        return true;
}

void mwoibn::collision_model::RobotCollision::addObject(S_Object* obj, std::string name)
{

        //	scene.addObject(obj);
        Object object;
        object.objectPtr = obj;
        object.link = name;

        mwoibn::Quaternion quaternion(0, 0, 0, 1);

        object.orientation = quaternion;

        _objects.push_back(object);
}

void mwoibn::collision_model::RobotCollision::addObject(
        S_Object* obj, std::string name,
        mwoibn::Quaternion orientation)
{

        //	scene.addObject(obj);

        Object object;

        object.objectPtr = obj;
        object.link = name;
        object.orientation = orientation;

        _objects.push_back(object);
}

void mwoibn::collision_model::RobotCollision::addPair(std::string link1,
                                                      std::string link2)
{

        LOG_INFO << "Adding a pair between " << link1 << ", " << link2 << "\n";
        auto iPtr = find_if(_objects.begin(), _objects.end(), [&](Object object)
                            {
                                    return object.link == link1;
                            });
        auto jPtr = find_if(_objects.begin(), _objects.end(), [&](Object object)
                            {
                                    return object.link == link2;
                            });

        if (iPtr == _objects.end() || jPtr == _objects.end())
        {
                LOG_INFO << "Could not find a link. Escape pair creation";
                return;
        }
        int i = std::distance(_objects.begin(), iPtr);
        int j = std::distance(_objects.begin(), jPtr);

        auto is_defined =
                std::find_if(_pairs.begin(), _pairs.end(), [&](Pair pair)
                             {
                                     return (pair.link_1 == i && pair.link_2 == j) ||
                                     (pair.link_1 == j && pair.link_2 == i);
                             });

        if (is_defined == _pairs.end())
        {
                Pair pair;

                pair.pair = new sch::CD_Pair(_objects[i].objectPtr, _objects[j].objectPtr);

                pair.link_1 = i;
                pair.link_2 = j;

                pair.witness_point_1.Set(0,0,0);
                pair.witness_point_2.Set(0,0,0);

                _pairs.push_back(pair);
        }
        else
                LOG_INFO << "The pair " << link1 << ", " << link2
                         << " has already been defined";
}

void mwoibn::collision_model::RobotCollision::updatePositions()
{
        for (int i = 0; i < _dof; i++)
                updatePosition(i);
}

void mwoibn::collision_model::RobotCollision::updatePosition(int i)
{
//  if (_robot.isStatic() && _objects.at(i).link == _reference)
//    return;

        mwoibn::Matrix3 rotation =
                RigidBodyDynamics::CalcBodyWorldOrientation(
                        _robot.getModel(), _robot.state.position.get(),
                        _robot.getModel().GetBodyId(_objects.at(i).link.c_str()), false);
        mwoibn::Quaternion quaternion =
                mwoibn::Quaternion::fromMatrix(rotation);
        mwoibn::Vector3 position =
                object_ik_ptr->getPointStateWorld(i);

        _objects[i].objectPtr->setPosition(0, 0, 0);

        _objects[i].objectPtr->setOrientation(
                _objects[i].orientation.x(), _objects[i].orientation.y(),
                _objects[i].orientation.z(), _objects[i].orientation.w());

        _objects[i].objectPtr->addRotation(quaternion.x(), quaternion.y(),
                                           quaternion.z(), quaternion.w());

        _objects[i].objectPtr->setPosition(position[0], position[1], position[2]);
}

bool mwoibn::collision_model::RobotCollision::updateCollision(int i)
{
        /*
           #ifdef TIME_CHECK
           std::chrono::steady_clock::time_point begin =
              std::chrono::steady_clock::now();
           #endif
         */
        bool collision = 0;
//  sch::Point3 p1;
//  sch::Point3 p2;

        if ((_pairs[i].distance = _pairs[i].pair->getClosestPoints(_pairs[i].witness_point_1, _pairs[i].witness_point_2)) <= 0)
        {
                collision = true;
        }

        //updatePointHendler(i);
        /*
           #ifdef TIME_CHECK
           std::chrono::steady_clock::time_point end =
           std::chrono::steady_clock::now();
           std::chrono::duration<double> elapsed_seconds = end - begin;
           std::cout << i << "\t" << elapsed_seconds.count() << "\t";
           begin =  std::chrono::steady_clock::now();
           #endif
         */


        /*
           #ifdef TIME_CHECK
           end = std::chrono::steady_clock::now();
           elapsed_seconds = end - begin;
           std::cout << elapsed_seconds.count() << "\t";
           #endif
         */
        return collision;
}

void mwoibn::collision_model::RobotCollision::updatePointHendler(int i){
        mwoibn::Vector3 wp;

        wp << _pairs[i].witness_point_1[0], _pairs[i].witness_point_1[1], _pairs[i].witness_point_1[2];
        pair_ik_ptr->setPointStateWorld(2 * i, wp);

        wp << _pairs[i].witness_point_2[0], _pairs[i].witness_point_2[1], _pairs[i].witness_point_2[2];
        pair_ik_ptr->setPointStateWorld(2 * i + 1, wp);
}

int mwoibn::collision_model::RobotCollision::updateCollisions()
{

        int collisions = 0;
#ifdef TIME_CHECK
        std::chrono::steady_clock::time_point begin =
                std::chrono::steady_clock::now();
#endif
        updatePositions();
#ifdef TIME_CHECK
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - begin;
        std::cout << "Time\t" << elapsed_seconds.count() << "\t";
        begin = std::chrono::steady_clock::now();
#endif

        for (int i = 0; i < _pairs_number; i++)
                collisions += updateCollision(i);

#ifdef TIME_CHECK
        end = std::chrono::steady_clock::now();
        elapsed_seconds = end - begin;
        std::cout << elapsed_seconds.count() << std::endl;
#endif

        //  std::cout << std::endl;
        return collisions;
}

mwoibn::VectorN
mwoibn::collision_model::RobotCollision::getDistances()
{
        mwoibn::VectorN distances(_pairs.size());
        int i = 0;
        for (auto& pair : _pairs)
        {
                distances[i] = sqrt(pair.distance);
                i++;
        }
        return distances;
}
