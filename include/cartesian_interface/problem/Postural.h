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
         * @brief use_inertia_matrix if true the postural task is weighted with the inertia matrix.
         * To use it set the tag:
         *                          use_inertia: true
         */
        bool use_inertia_matrix;
        
        /**
         * @brief Construct a postural task from the number of robot dofs (including 6 virtual joints)
         * 
         * @param ndof The number of robot dofs (including 6 virtual joints)
         */
        PosturalTask(ModelInterface::ConstPtr model, int ndof);

        PosturalTask(YAML::Node node, ModelInterface::ConstPtr model);

        void getReferencePosture(Eigen::VectorXd& qref) const;

        void getReferencePosture(JointNameMap& qref) const;
        
        void run(double time, double period) override;

        void reset() override;

    private:

        Eigen::VectorXd _qref;
    };

} }


#endif
