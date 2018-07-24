/*
 * Copyright (C) 2018 IIT-ADVR
 * Author: Arturo Laurenzi
 * email:  arturo.laurenzi@iit.it
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/

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
        
        typedef std::vector<WayPoint> WayPointVector;
        
        Trajectory();
        
        void addWayPoint(double time, 
                         const Eigen::Affine3d& frame);
        
        void addWayPoint(const WayPoint& way_point, double time_offset = 0.0);

        void clear();
        
        const std::vector<WayPoint>& getWayPoints() const;
        
        virtual void compute();
        
        virtual Eigen::Affine3d evaluate(double time, 
                                 Eigen::Vector6d * const vel = nullptr, 
                                 Eigen::Vector6d * const acc = nullptr);
        
        bool isTrajectoryEnded(double time);
        
        
    protected:
        
    private:
        
        void sort_frames();
        
        std::vector<WayPoint> _frames;
        
    };
    
    
} }

#endif