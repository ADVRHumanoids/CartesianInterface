#ifndef __XBOT_CARTESIAN_WITH_ROS_IO_H__
#define __XBOT_CARTESIAN_WITH_ROS_IO_H__

#include <ros/ros.h>
#include <memory>

namespace XBot { namespace Cartesian {

    class RosEnabled
    {
        
    public:
        
        typedef std::shared_ptr<RosEnabled> Ptr;
        
        virtual bool initRos(ros::NodeHandle nh) = 0;
        
        virtual void updateRos() = 0;
        
        virtual ~RosEnabled(){}
        
    };
    
} }


#endif