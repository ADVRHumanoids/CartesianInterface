#ifndef __XBOT_CARTESIAN_PROBLEM_POSTURAL_H__
#define __XBOT_CARTESIAN_PROBLEM_POSTURAL_H__

#include <cartesian_interface/problem/Task.h>

namespace XBot { namespace Cartesian {
    
    
    /**
     * @brief Description of a postural (i.e. joint space) task
     * 
     */
    struct PosturalTask : TaskDescription {
        
        typedef std::shared_ptr<PosturalTask> Ptr;
        typedef std::shared_ptr<const PosturalTask> ConstPtr;

        /**
         * @brief use_inertia_matrix if true the postural task is weighted with the inertia matrix
         */
        bool use_inertia_matrix;
        
        /**
         * @brief Construct a postural task from the number of robot dofs (including 6 virtual joints)
         * 
         * @param ndof The number of robot dofs (including 6 virtual joints)
         */
        PosturalTask(int ndof);
        
    };
    
    /**
    * @brief Construct a postural task and return a shared pointer
    * 
    * @param ndof The number of robot dofs (including 6 virtual joints)
    */
    PosturalTask::Ptr MakePostural(int ndof);
    
    /**
     * @brief Dynamic cast a generic task to a PosturalTask
     * 
     * @return A null pointer if the cast is unsuccessful (i.e. task is not a PosturalTask)
     */
    PosturalTask::Ptr GetAsPostural(TaskDescription::Ptr task);
  
    
} }


#endif
