#ifndef __MWOIBN__ROBOT_POINTS__HANDLER_PTRS_H
#define __MWOIBN__ROBOT_POINTS__HANDLER_PTRS_H

#include "mwoibn/robot_points/point.h"

#include "mwoibn/point_handling/position.h"

namespace mwoibn
{
namespace robot_points
{

//! This class provides a methods to menage multiple Robot Points.
/**
 *  This class provides a convinent and real-time safe way to compute the concatenatd vectors and Jacobians for multiple robot points.
 *  This class provides iterators to provide access to the Type interface for all the elements
 *  This class allows to menage updates for multiple robot points
 */
 template<typename Type>
class Handler
{

public:
    Handler(unsigned int dofs) : _dofs(dofs) {}
    Handler(int dofs) : _dofs(dofs) {}
    Handler() : _dofs(0) {}


    // A move constructor
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

    //! add a point to the Handler by moving a unique pointer
    template<typename Other>
    int add(std::unique_ptr<Other> point)
    {
            _points.push_back(std::move(point));
            resize();
            return _points.size()-1;
    }

    //! add a point to the Handler with a copy constructor

    template<typename Other>
    int add(Other point)
    {
           _points.push_back(std::unique_ptr<Type>(new Other(std::move(point))));
           resize();
           return _points.size()-1;
    }

    //! allocate the memmory according to the current handler state

    virtual void resize(){
          	double size = 0, rows = 0, cols = 0;
          	for(auto& point: _points){
              		  size += point->size();
          		      rows += point->rows();
             cols = point->cols(); // the number of columns has to be equal for all the points
          	}
          	_jacobian = mwoibn::Matrix::Zero(size, cols);
            _positions = mwoibn::VectorN::Zero(rows);
            _dofs = cols;
    }


    //! return the number of points in the Handler
    int size() const {
            return _points.size();
    }                                             

    //! update all the points
    /** @param[in] - should the jacobians (mappings) be updated as well */
    void update(bool jacobian){
    	for(auto& point: _points)
    		point->update(jacobian);
    }

    //! remove an i_th point from the Handler
    bool remove(int i)
    {

      if(i < 0 && std::abs(i) <= _points.size()){
          _points.erase(_points.end() - i);
          resize();
          return true;
      }

      if(i >= 0 && i < _points.size()){
      _points.erase(_points.begin() + i);
      resize();
      return true;
      }

      return false;
    }

    //! remove all the points from the Handler
    void clear(){
      _points.clear();
    }

    //! provide an access to the n-th robot point, a zero-based 
    Type& point(unsigned int id)
    {
      if (id < _points.size())
        return *_points.at(id);
      else
        throw std::out_of_range("Given ID is beyond a vector scope");
    }

    //! Computes and returns a concatenated matrix of jacobians of all the points in the Handler. This method is RT safe
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

    //! Returns the concatenated matrix of jacobians of all the points in the Handler. This method is RT safe
    const mwoibn::Matrix& jacobian() const {return _jacobian;}
    //! Returns the concatenated vector of vectors of all the points in the Handler. This method is RT safe
    const mwoibn::VectorN& state() const {return _positions;}


    //! Returns the vector of jacobians of all the points. WARNING: This method is not RT safe
    std::vector<mwoibn::Matrix> getJacobians()
    {

      std::vector<mwoibn::Matrix> jacobians;
      for (auto& point : _points)
      {
        jacobians.push_back(point->getJacobian());
      }

      return jacobians;
    }

    //! Computes and returns the concatenated vector  of vectors of all the points in the Handler. This method is RT safe
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

    //! Returns the vector of vectors of all the points in the Handler. WARNING: This method is not RT safe
    std::vector<mwoibn::VectorN> getStates()
    {
      std::vector<mwoibn::VectorN> positions;

      for (auto& point : _points)
      {
        positions.push_back(point->get());
      }
      return positions;
    }



    //! returns the number of rows in the concatenated jacobian
    int rows() const {
            int i = 0;
            for (auto& point : _points)
                    i+= point->rows();
            return i;
    }

    //! returns the number of columns in the concatenated jacobian
    int cols() const {
            return _dofs;
    }

    //! An iterator to the beginning of the robot points container
    typename std::vector<std::unique_ptr<Type>>::iterator begin(){return _points.begin();}
    //! An iterator to the end of the the robot points container
    typename std::vector<std::unique_ptr<Type>>::iterator end(){return _points.end();}

    //! A constant iterator to the beginning of the robot points container
    typename std::vector<std::unique_ptr<Type>>::const_iterator begin() const {return _points.begin();}
    //! A constant iterator to the end of the robot points container
    typename std::vector<std::unique_ptr<Type>>::const_iterator end() const {return _points.end();}

    //! A method to provide a zero-based access to the n-th last robot point
    virtual Type& end(unsigned int i) {
            int idx = -i - 1;
            return *(_points.end()[idx]);
    }

    //! A method to provide a zero-based access to the n-th robot point
    virtual Type& operator[](int i) {
            return *_points[i];
    }

protected:
    std::vector<std::unique_ptr<Type> > _points; // container to keep the robot_points
    mwoibn::Matrix _jacobian; // concatenation of all the jacobians
    mwoibn::VectorN _positions; // concatenation of all the vectors
    unsigned int _dofs; // number of decalred columns
};


} // namespace package
} // namespace library
#endif // CONTACTS_H
