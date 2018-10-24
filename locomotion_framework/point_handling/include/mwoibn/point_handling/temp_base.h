#ifndef __MWOIBN__POINT_HANDLING__TEMP_BASE_H
#define __MWOIBN__POINT_HANDLING__TEMP_BASE_H

#include "mwoibn/point_handling/point_handling.h"

namespace mwoibn
{

namespace point_handling
{

template<typename Type>
class TempBase
{

public:
  TempBase(unsigned int body_id, RigidBodyDynamics::Model& model,
       const mwoibn::robot_class::State& state, unsigned int size, std::string name = "")
      : _body_id(body_id), _name(name), _model(model), _state(state), _size(size)
  {
  }

  TempBase(std::string body_name, RigidBodyDynamics::Model& model,
       const mwoibn::robot_class::State& state, unsigned int size, std::string name = "")
      : _name(name), _model(model), _body_id(_checkBody(body_name, model)), _state(state), _size(size)
  {
  }

  TempBase(const TempBase&& other)
      : _name(other._name), _model(other._model), _body_id(other._body_id), _state(other._state), _size(other._size)
  {  }

  TempBase(const TempBase& other)
      : _name(other._name), _model(other._model), _body_id(other._body_id), _state(other._state), _size(other._size)
  {  }



    template<typename Source>
    TempBase(const Source&& other, int size, std::string name = "")
        : _name(name), _model(other.getModel()), _state(other.getState()), _body_id(other.getBodyId()), _size(size)
    {  }

    template<typename Source>
    TempBase(const Source& other, int size, std::string name = "")
        : _name(name), _model(other.getModel()), _state(other.getState()), _body_id(other.getBodyId()), _size(size)
    {  }

  virtual ~TempBase() {}

  /** returns human-readable name of a point*/
  std::string getName() const { return _name; }

  /** returns the RBDL id of a point fixed frame */
  unsigned int getBodyId() const { return _body_id; }

  virtual int size() const { return _size; }
  virtual int dofs() const { return _state.size(); }

  RigidBodyDynamics::Model& getModel() const{
    return _model;
  }

  const mwoibn::robot_class::State& getState() const{
    return _state;
  }


protected:
  /** human-readable name of a point*/
  const std::string _name;
  /** RBDL id number of a point fixed frame */
  const unsigned _body_id;
  /** reference to a RBDL model */
  RigidBodyDynamics::Model& _model;
  const mwoibn::robot_class::State& _state;
  unsigned int _size;

  /** helping function to provide functinoality of calling functions through
   * frames RBDL names */
  unsigned int _checkBody(std::string body_name) const
  {
    return _checkBody(body_name, _model);
  }

  /** helping function to provide functinoality of calling functions through
   * frames RBDL names */
  unsigned int _checkBody(std::string body_name,
                          RigidBodyDynamics::Model model) const
  {

    unsigned int body_id = model.GetBodyId(body_name.c_str());

    if (body_id == std::numeric_limits<unsigned int>::max())
    {

      throw(std::invalid_argument("unknown body, " + body_name +
                                  " couldn't find it in a RBDL model"));
    }
    return body_id;
  }
};

} // namespace package
} // namespace library

#endif
