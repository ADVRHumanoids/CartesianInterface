#ifndef __XBOT_CARTESIAN_PROBLEM_INTERACTION_H__
#define __XBOT_CARTESIAN_PROBLEM_INTERACTION_H__

#include <cartesian_interface/problem/Cartesian.h>

namespace Eigen 
{
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
}

namespace XBot { namespace Cartesian {
    
    /**
     * @brief Description of an interaction task (extends CartesianTask)
     */
    struct InteractionTask : CartesianTask {
        
        /**
        * @brief Interaction parameters for the task (linear, then angular)
        */
        Eigen::Vector6d stiffness, damping;
        
        /**
        * @brief Chains to be used for force estimation
        */
        std::vector<std::string> force_estimation_chains;

        typedef std::shared_ptr<InteractionTask> Ptr;
        typedef std::shared_ptr<const InteractionTask> ConstPtr;
        
        InteractionTask() = default;
        InteractionTask(std::string distal_link, 
                        std::string base_link = "world", 
                        int size = 6,
                        std::string type = "Interaction");

        static TaskDescription::Ptr yaml_parse_interaction(YAML::Node node, ModelInterface::ConstPtr model);
        
        
    };
    
    /**
     * @brief Make a cartesian task and return a shared pointer
     */
    InteractionTask::Ptr MakeInteraction(std::string distal_link, std::string base_link = "world");

    /**
     * @brief Dynamic cast a generic task to an InteractionTask
     * 
     * @return A null pointer if the cast is unsuccessful (i.e. task is not an InteractionTask)
     */
    InteractionTask::Ptr GetAsInteraction(TaskDescription::Ptr task);
    
    
    
  
    
} }




#endif
