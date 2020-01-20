#ifndef __XBOT_CARTESIAN_PROBLEM_POSTURAL_H__
#define __XBOT_CARTESIAN_PROBLEM_POSTURAL_H__

#include <cartesian_interface/problem/Task.h>

namespace XBot { namespace Cartesian {
    
    
    /**
     * @brief Description of a postural (i.e. joint space) task
     * 
     */
    struct PosturalTask : virtual TaskDescription {
        
        typedef std::shared_ptr<PosturalTask> Ptr;
        typedef std::shared_ptr<const PosturalTask> ConstPtr;

        virtual bool useInertiaMatrixWeight() const = 0;
        
        virtual void getReferencePosture(Eigen::VectorXd& qref) const = 0;

        virtual void getReferencePosture(JointNameMap& qref) const = 0;

        virtual void setReferencePosture(const JointNameMap& qref) = 0;

    };

} }


#endif
