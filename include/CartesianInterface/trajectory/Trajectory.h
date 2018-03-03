#ifndef __XBOT_CARTESIAN_TRAJ_INTERPOLATION_H__
#define __XBOT_CARTESIAN_TRAJ_INTERPOLATION_H__

#include <XBotInterface/ModelInterface.h>

namespace XBot { namespace Cartesian {
   
    class Trajectory {
        
    public:
        
        typedef std::shared_ptr<Trajectory> Ptr;
        typedef std::shared_ptr<const Trajectory> ConstPtr;
        
        struct WayPoint {
            
            Eigen::Affine3d frame;
            Eigen::Vector6d vel, acc;
            double time;
            
            WayPoint();
        };
        
        Trajectory();
        
        void addWayPoint(double time, 
                         const Eigen::Affine3d& frame);
        
        void addWayPoint(const WayPoint& way_point);
        
        void setVelocityLimit(const Eigen::Vector6d& vel_max);
        void setAccelerationLimit(const Eigen::Vector6d& acc_max);
        
        void clear();
        
        const std::vector<WayPoint>& getWayPoints() const;
        
        virtual Eigen::Affine3d evaluate(double time, 
                                 Eigen::Vector6d * const vel = nullptr, 
                                 Eigen::Vector6d * const acc = nullptr);
        
        bool isTrajectoryEnded(double time);
        
        
    protected:
        
    private:
        
        void sort_frames();
        
        std::vector<WayPoint> _frames;
        Eigen::Vector6d _vel_max, _acc_max;
        
    };
    
    
} }

#endif