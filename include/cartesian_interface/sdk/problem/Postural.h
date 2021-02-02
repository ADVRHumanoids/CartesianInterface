#ifndef __XBOT_CARTESIAN_PROBLEM_POSTURAL_IMPL_H__
#define __XBOT_CARTESIAN_PROBLEM_POSTURAL_IMPL_H__

#include <cartesian_interface/problem/Postural.h>
#include <cartesian_interface/sdk/problem/Task.h>

namespace XBot { namespace Cartesian {
    
    
    /**
     * @brief Description of a postural (i.e. joint space) task
     * 
     */
    struct PosturalTaskImpl : public TaskDescriptionImpl,
            public virtual PosturalTask
    {
        
        CARTESIO_DECLARE_SMART_PTR(PosturalTaskImpl)

        
        /**
         * @brief Construct a postural task from the number of robot dofs (including 6 virtual joints)
         * 
         * @param ndof The number of robot dofs (including 6 virtual joints)
         */
        PosturalTaskImpl(Context::ConstPtr context, int ndof);

        PosturalTaskImpl(YAML::Node node, Context::ConstPtr contextl);

        /**
         * @brief use_inertia_matrix if true the postural task is weighted with the inertia matrix.
         * To use it set the tag:
         *                          use_inertia: true
         */
        bool useInertiaMatrixWeight() const override;

        void getReferencePosture(Eigen::VectorXd& qref) const override;

        void getReferencePosture(JointNameMap& qref) const override;

        void getReferenceVelocity(Eigen::VectorXd& qdotref) const override;

        void setReferencePosture(const JointNameMap& qref) override;

        void setReferenceVelocity(const JointNameMap& qdotref) override;

        void setReferenceVelocity(const Eigen::VectorXd& qdotref) override;
        
        void update(double time, double period) override;

        void reset() override;

        void setIndices(const std::vector<int>& value) override;

        void setDisabledJoints(const std::vector<std::string>& value) override;

    private:

        Eigen::VectorXd _qref, _qdotref;
        bool _use_inertia_matrix;

    };

} }


#endif
