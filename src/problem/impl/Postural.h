#ifndef __XBOT_CARTESIAN_PROBLEM_POSTURAL_IMPL_H__
#define __XBOT_CARTESIAN_PROBLEM_POSTURAL_IMPL_H__

#include <cartesian_interface/problem/Postural.h>

#include "Task.h"

namespace XBot { namespace Cartesian {
    
    
    /**
     * @brief Description of a postural (i.e. joint space) task
     * 
     */
    struct PosturalTaskImpl : public TaskDescriptionImpl,
            public virtual PosturalTask
    {
        


        
        /**
         * @brief Construct a postural task from the number of robot dofs (including 6 virtual joints)
         * 
         * @param ndof The number of robot dofs (including 6 virtual joints)
         */
        PosturalTaskImpl(ModelInterface::ConstPtr model, int ndof);

        PosturalTaskImpl(YAML::Node node, ModelInterface::ConstPtr model);

        /**
         * @brief use_inertia_matrix if true the postural task is weighted with the inertia matrix.
         * To use it set the tag:
         *                          use_inertia: true
         */
        bool useInertiaMatrixWeight() const override;

        void getReferencePosture(Eigen::VectorXd& qref) const override;

        void getReferencePosture(JointNameMap& qref) const override;

        void setReferencePosture(const JointNameMap& qref) override;
        
        void update(double time, double period) override;

        void reset() override;

    private:

        Eigen::VectorXd _qref;
        bool _use_inertia_matrix;
    };

} }


#endif