#ifndef __XBOT_CARTESIAN_PROBLEM_CARTESIAN_H__
#define __XBOT_CARTESIAN_PROBLEM_CARTESIAN_H__

#include <cartesian_interface/problem/Task.h>

namespace XBot { namespace Cartesian {
    
    /**
     * @brief Description of a cartesian task
     */
    struct CartesianTask : TaskDescription {
        
        /**
         * @brief The task controls the relative motion of distal_link w.r.t base_link
         */
        std::string base_link, distal_link;

        /**
         * @brief Set Cartesian task in local (body) frame
         */
        bool is_body_jacobian;

        /**
         * @brief Parameter that weights orientation errors w.r.t. position errors.
         * For example, setting it to orientation_gain = 2.0 means that an error of
         * 2 rad is recovered in the same time as an error of 1 meter.
         */
        double orientation_gain;
        
        typedef std::shared_ptr<CartesianTask> Ptr;
        typedef std::shared_ptr<const CartesianTask> ConstPtr;
        
        CartesianTask() = default;
        CartesianTask(std::string distal_link, 
                      std::string base_link = "world", 
                      int size = 6,
                      std::string type = "Cartesian");        

        static TaskDescription::Ptr yaml_parse_cartesian(YAML::Node node, ModelInterface::ConstPtr model);
        
    };
    
    /**
     * @brief Make a cartesian task and return a shared pointer
     */
    CartesianTask::Ptr MakeCartesian(std::string distal_link, std::string base_link = "world");

    /**
     * @brief Dynamic cast a generic task to a CartesianTask
     * 
     * @return A null pointer if the cast is unsuccessful (i.e. task is not a CartesianTask)
     */
    CartesianTask::Ptr GetAsCartesian(TaskDescription::Ptr task);


    
} }


#endif
