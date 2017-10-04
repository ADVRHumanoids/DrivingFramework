#ifndef HIERARCHICAL_CONTROL_HIERARCHICAL_CONTROLLER_CONTINOUS_H
#define HIERARCHICAL_CONTROL_HIERARCHICAL_CONTROLLER_CONTINOUS_H

#include "mwoibn/hierarchical_control/hierarchical_controller.h"
#include "mwoibn/robot_class/robot.h"

namespace mwoibn{

namespace hierarchical_control
{
	
	class HierarchicalControllerContinous: public HierarchicalController{
		public:
      HierarchicalControllerContinous(mwoibn::robot_class::Robot* robot, double mu = 10000000): HierarchicalController(), _robot(robot), _mu(mu){
        _joint_states = _robot->state.get(mwoibn::robot_class::POSITION);
        _previous_joint_states = mwoibn::VectorN::Zero(_robot->getDofs());
  }
			~HierarchicalControllerContinous(){}
      void addTask(ControllerTask* new_task, mwoibn::VectorN gain, int i = -1);
			void removeTask(int i);

      //! updates states of all of the controllers and computes the control law with compute()
			/**
       * \see HierarchicalController#compute()
			 * 
			 */
      virtual mwoibn::VectorN update();
			//! computes the controll law without updating the controllers states 
			/** 
			 * \see HierarchicalController#updateController()
			 * 
			 */
      virtual void compute();

		protected:
			double _last_stack_change = 0;
			std::vector<double> _last_tasks_ranks;
      mwoibn::VectorN _e;
      mwoibn::Matrix _g;
      mwoibn::robot_class::Robot* _robot;
			void _resetCorrection();
      mwoibn::VectorN _previous_joint_states;
      mwoibn::VectorN _joint_states;
			double _mu;
			
			
			
	};
	
} // namespace package
} // namespace library

#endif
