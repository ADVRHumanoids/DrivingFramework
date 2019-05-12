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
    Handler() : _dofs(0) {}


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
           _points.push_back(std::unique_ptr<Type>(new Other(std::move(point))));
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
          	double size = 0, rows = 0, cols = 0;
          	for(auto& point: _points){
              		  size += point->size();
          		      rows += point->rows();
                    cols = point->cols();
          	}
          	_jacobian = mwoibn::Matrix::Zero(size, cols);
            _positions = mwoibn::VectorN::Zero(rows);
            _dofs = cols;
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
      // std::cout << __PRETTY_FUNCTION__ << std::endl;
      // std::cout << i << std::endl;
      // std::cout << _points.size() << std::endl;
      // std::cout << (std::abs(i) <= _points.size()) << std::endl;
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

    void clear(){
      _points.clear();
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

    const mwoibn::Matrix& jacobian() const {return _jacobian;}
    const mwoibn::VectorN& state() const {return _positions;}


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

    virtual Type& end(unsigned int i) {
            int idx = -i - 1;
            // std::cout << "idx\t" << idx << std::endl;
            return *(_points.end()[idx]);
    }

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
