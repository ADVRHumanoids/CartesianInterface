#ifndef __CARTESIAN_INTERFACE_UTILS_POSE_INTERPOLATION_H__
#define __CARTESIAN_INTERFACE_UTILS_POSE_INTERPOLATION_H__

#include <ReflexxesTypeII/Wrappers/TrajectoryGenerator.h>

namespace XBot { namespace Cartesian { namespace Utils {
    
    class PoseInterpolation 
    {
        
    public:
        
        PoseInterpolation(double period = -1.0);
        PoseInterpolation(const Eigen::Affine3d& T, double period);
        
        void setReference(const Eigen::Affine3d& T);
        
        void update();
        
        Eigen::Affine3d getPose() const;
        
        void reset(const Eigen::Affine3d& T);
        
        
    private:
        
        typedef Eigen::Matrix<double, 7, 1> Vector7d;
        
        static Vector7d get_identity_pose();
        
        Reflexxes::Utils::TrajectoryGenerator _traj;
        double _period;
        
    };
    
} } }

#endif

XBot::Cartesian::Utils::PoseInterpolation::PoseInterpolation(double period):
    _period(period),
    _traj(7, period, get_identity_pose())
{

}

XBot::Cartesian::Utils::PoseInterpolation::PoseInterpolation(const Eigen::Affine3d& T, double period)
{

}
