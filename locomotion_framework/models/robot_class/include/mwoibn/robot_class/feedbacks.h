#ifndef ROBOT_CLASS_FEEDBACKS_H
#define ROBOT_CLASS_FEEDBACKS_H

#include "mwoibn/communication_modules/basic_feedback.h"
#include <memory>

namespace mwoibn {

namespace robot_class {

class Feedbacks{

public:
   Feedbacks(){}
   virtual ~Feedbacks(){}

   virtual void add(std::unique_ptr<mwoibn::communication_modules::BasicFeedback> feedback){
      _feedbacks.push_back(std::move(feedback));
   }

   virtual bool remove(int i){
       if (i < 0 || i >= _feedbacks.size())
         return false;

       _feedbacks.erase(_feedbacks.begin() + i);
       return true;
   }

   bool get()
   {

     bool success = true;
     for (auto& feedback : _feedbacks)
       success = feedback->get() && success;

     return success;
   }

   bool initialized(){
     bool success = true;
     for (auto& feedback : _feedbacks)
       success = feedback->initialized() && success;

     return success;
   }

   mwoibn::communication_modules::BasicFeedback& feedback(unsigned int id){
     if (id < _feedbacks.size())
       return *_feedbacks.at(id);
     else
       throw std::out_of_range("Given ID is beyond a vector scope");
   }


protected:
   std::vector<std::unique_ptr<mwoibn::communication_modules::BasicFeedback>>
       _feedbacks;
};
}
}

#endif // FEEDBACKS_H
