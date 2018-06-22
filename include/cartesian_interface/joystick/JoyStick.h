#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/TwistStamped.h>
#include <eigen3/Eigen/Dense>
#include <tf/transform_listener.h>

namespace XBot { namespace Cartesian {
class JoyStick{
public:
    typedef boost::shared_ptr<JoyStick> Ptr;

    JoyStick(const std::vector<std::string>& distal_links, std::string tf_prefix = "");

    ~JoyStick();

    void sendVelRefs();

    std::string Doc()
    {
        std::stringstream doc;
        doc<<
        "JoyStick Control:\n \n"
        "   Select button:                  Print this documentation\n"
        "   Start button:                   Enable velocity control in task\n"
        "   Left Analog:                    UP/DOWN    -> X Global Coordinates\n"
        "                                   LEFT/RIGHT -> Y Global Coordinates\n"
        "   Right Analog:                   UP/DOWN    -> PITCH Global Coordinates\n"
        "                                   LEFT/RIGHT -> YAW Global Coordinates\n"
        "   D-pad:                          UP/DOWN    -> Z Global Coordinates\n"
        "                                   LEFT/RIGHT -> ROLL Global Coordinates\n"
        "   A button:                       Set GLOBAL/LOCAL control\n"
        "   L2 + X/Y button:                Decrease/Increase linear speed\n"
        "   R2 + X/Y button:                Decrease/Increase angular speed\n"
        "   L1/R1:                          Previous/Next Task\n";

        return doc.str();
    }

private:
    /**
     * @brief _nh
     */
    ros::NodeHandle _nh;

    std::string _tf_prefix;

    std::vector<std::string> _distal_links;

    ros::Subscriber _joy_sub;

    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    void setVelocityCtrl();
    void localCtrl();
    int _selected_task;

    std::vector<ros::ServiceClient> _set_properties_service_clients;
    std::vector<ros::ServiceClient> _get_properties_service_clients;
    std::vector<ros::Publisher> _ref_pose_pubs;

    double _linear_speed_sf;
    double _angular_speed_sf;

    geometry_msgs::TwistStamped _desired_twist;
    Eigen::VectorXd _twist;

    tf::TransformListener _listener;
    tf::StampedTransform _transform;

    int _local_ctrl;
};
}
}
