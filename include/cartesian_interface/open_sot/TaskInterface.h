#ifndef __XBOT_CARTESIAN_OPENSOT_TASK_INTERFACE_H__
#define __XBOT_CARTESIAN_OPENSOT_TASK_INTERFACE_H__

#include <cartesian_interface/problem/Task.h>
#include <cartesian_interface/problem/Constraint.h>
#include <cartesian_interface/CartesianInterface.h>

#include <OpenSoT/tasks/Aggregated.h>
#include <OpenSoT/constraints/Aggregated.h>

namespace XBot { namespace Cartesian { namespace SoT {
    
    using TaskPtr = OpenSoT::tasks::Aggregated::TaskPtr;
    using ConstraintPtr = OpenSoT::constraints::Aggregated::ConstraintPtr;
 
    class TaskInterface
    {
        
    public:
        
        typedef std::shared_ptr<TaskInterface> Ptr;
        
        TaskInterface(TaskDescription::Ptr task_desc, 
                      ModelInterface::ConstPtr model
                      ){}
        
        virtual TaskPtr getTaskPtr() const = 0;
        
        virtual bool update(const CartesianInterface * ci,
                            double time, 
                            double period) = 0;
                            
        virtual bool setControlMode(const std::string& ee_name,
                                    ControlType ctrl_mode) = 0;
                                    
        virtual bool setBaseLink(const std::string& ee_name,
                                 const std::string& base_link) = 0;
        
        virtual ~TaskInterface() = default;
        
    };
    
    class ConstraintInterface
    {
        
    public:
        
        typedef std::shared_ptr<ConstraintInterface> Ptr;
        
        ConstraintInterface(ConstraintDescription::Ptr task_desc, 
                      ModelInterface::ConstPtr model
                      ){}
        
        virtual ConstraintPtr getConstraintPtr() const = 0;
        
        virtual bool update(const CartesianInterface * ci,
                            double time, 
                            double period) = 0;
                            
        virtual bool setControlMode(const std::string& ee_name,
                                    ControlType ctrl_mode) = 0;
                                    
        virtual bool setBaseLink(const std::string& ee_name,
                                 const std::string& base_link) = 0;
        
        virtual ~ConstraintInterface() = default;
        
    };
    

} } }

#endif


