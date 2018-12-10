#ifndef __MWOIBN__COMMON__UPDATE_MANAGER_H
#define __MWOIBN__COMMON__UPDATE_MANAGER_H


#include "mwoibn/common/types.h"

namespace mwoibn
{

namespace common
{
  template<typename Interface, typename Class>
  class UpdateManager {

  public:
    UpdateManager(Class& member, std::map<Interface, void (Class::*)() > map): _member(member){
        for(auto& entry: map)
          _update_map[entry.first] = {entry.second, 0};
    }


    void subscribe(std::vector<Interface> interfaces){
      for(auto& interface: interfaces)
        subscribe(interface);
    }

    void subscribe(Interface update){
      _update_map[update].second++;
    }


    void update() {
      for(auto& interface: _update_map){


          if(interface.second.second) (_member.*interface.second.first)();
      }
    }

    bool defined(Interface update) const {
      return _update_map.count(update);
    }

    bool is(Interface update) const {
      return defined(update) && _update_map.at(update).second;
    }

    bool unsubscribe(Interface update){
      if(!_update_map[update].second) return false;

      _update_map[update].second--;
      return true;
    }



  protected:
    Class& _member;
    std::map<Interface, std::pair<void (Class::*)(), int> > _update_map;
  };


}
}
#endif
