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

        /**
         * @brief getReferencePosture
         * @param qref
         */
        virtual void getReferencePosture(Eigen::VectorXd& qref) const = 0;

        /**
         * @brief getReferenceVelocity
         * @param qdotref
         */
        virtual void getReferenceVelocity(Eigen::VectorXd& qdotref) const = 0;

        /**
         * @brief getReferencePosture
         * @param qref
         */
        virtual void getReferencePosture(JointNameMap& qref) const = 0;

        /**
         * @brief setReferencePosture
         * @param qref
         */
        virtual void setReferencePosture(const JointNameMap& qref) = 0;

        /**
         * @brief setReferenceVelocity
         * @param qdotref
         */
        virtual void setReferenceVelocity(const JointNameMap& qdotref) = 0;

        /**
         * @brief setReferenceVelocity
         * @param qdotref
         */
        virtual void setReferenceVelocity(const Eigen::VectorXd& qdotref) = 0;

    };

} }


#endif
