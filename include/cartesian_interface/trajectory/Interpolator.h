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

#ifndef __XBOT_CARTESIAN_INTERPOLATOR_H__
#define __XBOT_CARTESIAN_INTERPOLATOR_H__

#include <XBotInterface/ModelInterface.h>

/* fi: this class is just a draft of a generic interpolator thet we will use for
 * any object with an arithmetic... Now it is just a copy-paste from Trajectory
 * class. One day... */

namespace XBot { namespace Cartesian {
	
	template <typename T>
	class Interpolator {

    public:

        typedef std::shared_ptr<Interpolator<T>>       Ptr     ;
        typedef std::shared_ptr<const Interpolator<T>> ConstPtr;
        
        struct WayPoint {
            
            T      value;
            double time ;
            
            WayPoint () = default;
			
            WayPoint (T      v,
					  double t)
				: value (v)
				, time  (t)
			{}
        };
        
        typedef std::vector<WayPoint> WayPointVector;
        
        Interpolator ()
		{
			_points.reserve(10);
		}
        
        void addWayPoint (double    time ,
                          const T & value)
		{
			WayPoint wp(value, time);
			
			addWayPoint(wp);
		}
        
        void addWayPoint (const WayPoint & way_point        ,
						  double           time_offset = 0.0)
		{
			_points.push_back(way_point);
			_points.back().time += time_offset;
			
			sort_points();
		}

        void clear ()
		{
		    _points.clear();
		}
        
        const std::vector<WayPoint> & getWayPoints() const
        {
			return _points;
		}
        
        virtual void compute ()
		{
			/**/
		}
        
        virtual T evaluate (double time)
		{
			/* Find relevant segment (first frame after time) */
			auto it = std::find_if (_points.begin(),
									_points.end()  ,
									[&time](const WayPoint& wp){ return wp.time > time; });
			
			if (it == _points.begin()) // trajectory yet to start
			{
				return it->value;
			}
			
			if(it == _points.end()) // no frames after time (traj is finished)
			{
				return (--it)->value;
			}
			
			T end   =  it   ->value;
			T start = (it-1)->value;
			
			double t_end   =  it   ->time;
			double t_start = (it-1)->time;

			double tau, dtau, ddtau;
			XBot::Utils::FifthOrderPlanning(0, 0, 0, 1, t_start, t_end, time, tau, dtau, ddtau);
			
			T interpolated = (1 - tau)*start + tau*end;
			
			return interpolated;    
		}

        int getCurrentSegmentId (double time) const
        {
			if (_points.empty())
			{
				return -1;
			}

			/* Find relevant segment (first frame after time) */
			auto it = std::find_if(_points.begin(),
								   _points.end  (),
								   [&time](const WayPoint& wp){ return wp.time > time; });

			int id = std::distance(_points.begin(), it) - 1;

			return id;

		}
        
        bool isTrajectoryEnded (double time)
		{
			return time > _points.back().time;
		}
        
        
    private:
        
        void sort_points()
		{
			std::sort(_points.begin(), _points.end(), [](const WayPoint& w1, const WayPoint& w2){ return w1.time < w2.time; });
		}
        
        std::vector<WayPoint> _points;
        
    };
    
    
} }

#endif

