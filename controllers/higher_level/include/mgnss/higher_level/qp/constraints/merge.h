#ifndef __MGNSS__HIGHER_LEVEL__MERGE_H
#define __MGNSS__HIGHER_LEVEL__MERGE_H

#include "mgnss/higher_level/qp/constraints/constraint.h"
#include "mwoibn/robot_points/handler.h"
#include "mwoibn/robot_points/point.h"

namespace mgnss
{

namespace higher_level
{

  namespace constraints
  {

// assumes that at least one constraint is active at all time
// assumes all constraint jacobians are equal
class Merge: public mgnss::higher_level::Constraint{

  public:
    Merge(): Constraint(){

    }

    Merge(const Merge& other): Constraint(other){
        for(auto& constraint: other._constraints)
          _constraints.add(constraint->clone());
    }

    Merge(Merge&& other): Constraint(other){
        for(auto& constraint: other._constraints)
          _constraints.add(constraint->clone());
    }

    template<typename Other>
    bool add(Other constraint)
    {
           _constraints.add(std::move(constraint));
           _checkConstraint();
           return true;
    }

    void init() override {
      resize(active.count(), _constraints[0].rows());
    }

    virtual void update();

    typename std::vector<std::unique_ptr<Constraint>>::iterator begin(){return _constraints.begin();}
    typename std::vector<std::unique_ptr<Constraint>>::iterator end(){return _constraints.end();}

    typename std::vector<std::unique_ptr<Constraint>>::const_iterator begin() const {return _constraints.begin();}
    typename std::vector<std::unique_ptr<Constraint>>::const_iterator end() const {return _constraints.end();}

    virtual Constraint& end(unsigned int i) {
            int idx = -i - 1;
            // std::cout << "idx\t" << idx << std::endl;
            return *(_constraints.end()[idx]);
    }

    virtual Constraint& operator[](int i) {
            return _constraints[i];
    }

  protected:
    virtual Merge* clone_impl() const override {return new Merge(*this);}
    mwoibn::robot_points::Handler<mgnss::higher_level::Constraint> _constraints;

    void _checkConstraint();
};

}
}
}
#endif
