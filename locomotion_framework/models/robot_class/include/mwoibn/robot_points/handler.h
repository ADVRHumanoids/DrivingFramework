#ifndef __MWOIBN__ROBOT_POINTS__HANDLER_PTRS_H
#define __MWOIBN__ROBOT_POINTS__HANDLER_PTRS_H

#include "mwoibn/robot_points/point.h"

#include "mwoibn/point_handling/position.h"

namespace mwoibn
{
namespace robot_points
{

/** @brief Keeps all the contact data and probides acces to individual contacts
 * **/
 template<typename Type>
class Handler
{

public:
    Handler(unsigned int dofs) : _dofs(dofs) {}
    Handler(int dofs) : _dofs(dofs) {}

    template<typename Other>
    Handler(Other&& other)
          : _positions(other.getState()), _jacobian(other.getJacobian()), _dofs(other._dofs)
    {
        for (auto& point : other)
            _points.push_back(std::move(point));

        other._points.clear();

        resize();
    }

    virtual ~Handler() { }

    template<typename Other>
    int add(std::unique_ptr<Other> point)
    {
            _points.push_back(std::move(point));
            resize();
            return _points.size()-1;
    }

    // template<typename Other>
    // void add(Other&& point)
    // {
    //        _points.push_back(std::unique_ptr<Other>(new Other(point)));
    //        resize();
    // }

    template<typename Other>
    int add(Other point)
    {
           _points.push_back(std::unique_ptr<Type>(new Other(point)));
           resize();
           return _points.size()-1;
    }

    // template<typename Other>
    // void add(const Other& point)
    // {
    //        _points.push_back(std::unique_ptr<Other>(new Other(point)));
    //        resize();
    // }

    virtual void resize(){
          	double size = 0, rows = 0;
          	for(auto& point: _points){
              		  size += point->size();
          		      rows += point->rows();
          	}
          	_jacobian = mwoibn::Matrix::Zero(size, _points[0]->cols());
            _positions = mwoibn::VectorN::Zero(rows);
    }


    /**
     * @brief Gives number of all considered contact points
     *
     */
    int size() const {
            return _points.size();
    }                                               // RT?

    void update(bool jacobian){
    	for(auto& point: _points)
    		point->update(jacobian);
    }

    bool remove(int i)
    {
      if (i < 0 || i >= _points.size())
        return false;

      _points.erase(_points.begin() + i);
      resize();
      return true;
    }


    Type& point(unsigned int id)
    {
      if (id < _points.size())
        return *_points.at(id);
      else
        throw std::out_of_range("Given ID is beyond a vector scope");
    }

    const mwoibn::Matrix& getJacobian()
    {
      _jacobian.setZero();

      unsigned int i = 0;

      for (auto& point : _points)
      {
        _jacobian.block(i, 0, point->rows(), point->cols()) = point->getJacobian();
        i += point->rows();
      }

      return _jacobian;
    }


    std::vector<mwoibn::Matrix> getJacobians()
    {

      std::vector<mwoibn::Matrix> jacobians;
      for (auto& point : _points)
      {
        jacobians.push_back(point->getJacobian());
      }

      return jacobians;
    }

    const mwoibn::VectorN& getState()
    {
      _positions.setZero();

      unsigned int i = 0;

      for (auto& point : _points)
      {
        _positions.segment(i, point->size()) = point->get();
        i += point->size();
      }

      return _positions;
    }

    std::vector<mwoibn::VectorN> getStates()
    {
      std::vector<mwoibn::VectorN> positions;

      for (auto& point : _points)
      {
        positions.push_back(point->get());
      }
      return positions;
    }


    int rows() const {
            int i = 0;
            for (auto& point : _points)
                    i+= point->rows();
            return i;
    }

    int cols() const {
            return _dofs;
    }

    typename std::vector<std::unique_ptr<Type>>::iterator begin(){return _points.begin();}
    typename std::vector<std::unique_ptr<Type>>::iterator end(){return _points.end();}

    typename std::vector<std::unique_ptr<Type>>::const_iterator begin() const {return _points.begin();}
    typename std::vector<std::unique_ptr<Type>>::const_iterator end() const {return _points.end();}

    virtual Type& operator[](int i) {
            return *_points[i];
    }

protected:
    std::vector<std::unique_ptr<Type> > _points;
    mwoibn::Matrix _jacobian;
    mwoibn::VectorN _positions;
    unsigned int _dofs;
};


} // namespace package
} // namespace library
#endif // CONTACTS_H
