#ifndef __MWOIBN__COMMUNICATION_MODULES__COMMUNICATIONS_H
#define __MWOIBN__COMMUNICATION_MODULES__COMMUNICATIONS_H

#include "mwoibn/communication_modules/communication_base.h"
#include <memory>

namespace mwoibn {

namespace communication_modules {

class Communications{


public:
   Communications(){}
   Communications(Communications&& other) {
     for(auto& module: other._modules)
       _modules[module.first] = std::move(module.second);
   }

   virtual ~Communications(){}


   template<typename Type>
   void add(std::unique_ptr<Type> module, std::string name){
     if(_modules.count("name"))
     throw std::invalid_argument(std::string(__PRETTY_FUNCTION__) + std::string(": Communication module ") + std::string(name) + " has already been registered in the stack." );

      _modules[name] = std::move(module);
   }

   template<typename Type>
   void add(Type&& module, std::string name){
     if(_modules.count("name"))
     throw std::invalid_argument(std::string(__PRETTY_FUNCTION__) + std::string(": Communication module ") + std::string(name) + " has already been registered in the stack." );

      _modules[name] = std::unique_ptr<CommunicationBase>(new Type(module));
   }

   template<typename Type>
   void add(Type& module, std::string name){
     if(_modules.count("name"))
     throw std::invalid_argument(std::string(__PRETTY_FUNCTION__) + std::string(": Communication module ") + std::string(name) + " has already been registered in the stack." );
      _modules[name] = std::unique_ptr<CommunicationBase>(new Type(module));
   }

   bool run()
   {

     bool success = true;
     for (auto& module : _modules){
       // std::cout << "success\t" << module.first << "\t" << module.second->run() << std::endl;
       success = module.second->run() && success;
     }
     return success;
   }

   bool initialized(){
     bool success = true;
     for (auto& module : _modules)
       success = module.second->initialized() && success;

     return success;
   }

   bool reset(){
     bool success = true;

     for (auto& module : _modules)
       success = module.second->reset() && success;

     return success;
   }

   mwoibn::communication_modules::CommunicationBase& module(std::string name)
   {
     return *_modules[name];
   }

   void remove(std::string name) { _modules.erase(name); }

   bool has(std::string name){
     return _modules.count(name);
   }

   bool startsWith(std::string start){
     auto i = _modules.lower_bound(start);
     if (i != _modules.end())
            return (i->first.compare(0, start.size(), start) == 0); // Really a prefix?

     return false;
   }

   std::map<std::string, std::unique_ptr<CommunicationBase>>::iterator begin(){return _modules.begin();}
   std::map<std::string, std::unique_ptr<CommunicationBase>>::iterator end(){return _modules.end();}

   std::map<std::string, std::unique_ptr<CommunicationBase>>::const_iterator begin() const {return _modules.begin();}
   std::map<std::string, std::unique_ptr<CommunicationBase>>::const_iterator end() const {return _modules.end();}

   virtual CommunicationBase& operator[](std::string name) {
           return *_modules[name];
   }


protected:
   std::map<std::string, std::unique_ptr<mwoibn::communication_modules::CommunicationBase>> _modules;
};
}
}

#endif // FEEDBACKS_H
