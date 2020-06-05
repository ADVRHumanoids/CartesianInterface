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

#ifndef __XBOT_CARTESIAN_TRAJ_SPLINE_H__
#define __XBOT_CARTESIAN_TRAJ_SPLINE_H__

#include <cartesian_interface/trajectory/Trajectory.h>

namespace alglib
{
class spline1dinterpolant;
}

namespace XBot { namespace Cartesian {
    
    class Spline : public Trajectory
    {
        
    public:
        
        Spline();
        
        virtual void compute();
        
        virtual Eigen::Affine3d evaluate(double time, 
                                         Eigen::Vector6d * const vel = nullptr, 
                                         Eigen::Vector6d * const acc = nullptr);

        ~Spline();
        
    private:
        
        void clear_all();
        void add_knot(double t, const Eigen::Affine3d& T);
        
        std::vector<alglib::spline1dinterpolant> _splines_pos, _splines_rot;
        std::vector<std::vector<double>> _values_pos, _values_rot;
        std::vector<double> _knots;
    };
    
} }

#endif


