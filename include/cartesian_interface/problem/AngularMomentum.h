#ifndef __XBOT_CARTESIAN_PROBLEM_ANGULAR_MOMENTUM_H__
#define __XBOT_CARTESIAN_PROBLEM_ANGULAR_MOMENTUM_H__

#include <cartesian_interface/problem/Task.h>

namespace XBot { namespace Cartesian {


    /**
     * @brief Description of a angular moemtum task
     *
     */
    struct AngularMomentumTask : virtual TaskDescription {

        typedef std::shared_ptr<AngularMomentumTask> Ptr;
        typedef std::shared_ptr<const AngularMomentumTask> ConstPtr;


        /**
         * @brief getAngularMomentumReference
         * @param href
         */
        virtual void getAngularMomentumReference(Eigen::Vector3d& href) const = 0;

        /**
         * @brief setAngularMomentumReference
         * @param href
         */
        virtual void setAngularMomentumReference(const Eigen::Vector3d& href) = 0;

    };

} }


#endif
