#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/TwistStamped.h>

namespace XBot { namespace Cartesian {
class JoyStick{
public:
    typedef boost::shared_ptr<JoyStick> Ptr;

    JoyStick(const std::vector<std::string>& distal_links);

    ~JoyStick();

    void sendVelRefs();

private:
    /**
     * @brief _nh
     */
    ros::NodeHandle _nh;

    std::vector<std::string> _distal_links;

    ros::Subscriber _joy_sub;

    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    void setVelocityCtrl();
    int _selected_task;

    std::vector<ros::ServiceClient> _set_properties_service_clients;
    std::vector<ros::Publisher> _ref_pose_pubs;

    double _linear_speed_sf;
    double _angular_speed_sf;

    geometry_msgs::TwistStamped _desired_twist;
};
}
}
