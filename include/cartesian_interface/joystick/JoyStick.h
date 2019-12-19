#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Empty.h>
#include <eigen3/Eigen/Dense>
#include <XBotInterface/Utils.h>
#include <cartesian_interface/SetJoystickActiveTask.h>
#include <cartesian_interface/SetJoystickTaskMaxSpeed.h>
#include <cartesian_interface/SetJoystickTaskBaseFrame.h>
#include <cartesian_interface/ros/RosImpl.h>

namespace Eigen 
{
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    typedef Eigen::Matrix<int, 6, 1> Vector6i;
}

namespace XBot { namespace Cartesian {
    
/**
 * @brief The JoyStick class implements the logic behind the joystick control
 */
class JoyStick {
    
public:
    
    typedef std::shared_ptr<JoyStick> Ptr;

    JoyStick(std::shared_ptr<RosImpl> ci_ros,
             std::string tf_prefix = "");

    ~JoyStick();
    
    void setTwistMask(const Eigen::Vector6i& mask);

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
        "   Y button:                       Desired end-effector velocities in robot base_link ON/OFF\n"
/*        "   B button:                       Activate/Deactivate Task\n" */
        "   L2 + X/Y button:                Decrease/Increase linear speed\n"
        "   R2 + X/Y button:                Decrease/Increase angular speed\n"
        "   L1/R1:                          Previous/Next Task\n";

        return doc.str();
    }

    std::string getRobotBaseLinkCtrlFrame();
    void setRobotBaseLinkCtrlFrame(const std::string& robot_base_link);

    bool updateStatus();
    void broadcastStatus();

private:
    /**
     * @brief _nh
     */
    ros::NodeHandle _nh;

    std::string _tf_prefix;

    std::vector<std::string> _distal_links;
    std::vector<std::string> _base_links;

    ros::Publisher _joy_audio_pub;
    ros::Subscriber _joy_sub;

    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    void setVelocityCtrl();
    void localCtrl();
    void twistInBase();
//    void activateDeactivateTask();

    int _selected_task;

    std::shared_ptr<RosImpl> _ci;

    double _linear_speed_sf;
    double _angular_speed_sf;

    geometry_msgs::TwistStamped _desired_twist;
    Eigen::Vector6d _twist;
    Eigen::Vector6i _twist_mask;
    XBot::Utils::SecondOrderFilter<Eigen::Vector6d> _twist_filt;


    int _local_ctrl;
    int _base_ctrl;

    std::string _robot_base_link;

    ros::Publisher _joystick_status_pub;

    ros::ServiceServer _joystick_set_active_task_service;
    bool setActiveTask(cartesian_interface::SetJoystickActiveTaskRequest& req,
                       cartesian_interface::SetJoystickActiveTaskResponse& res);

    ros::ServiceServer _joystick_set_task_max_speed_service;
    bool setMaxSpeed(cartesian_interface::SetJoystickTaskMaxSpeedRequest& req,
                     cartesian_interface::SetJoystickTaskMaxSpeedResponse& res);

    ros::ServiceServer _joystick_set_task_base_frame_service;
    bool setBaseFrame(cartesian_interface::SetJoystickTaskBaseFrameRequest& req,
                     cartesian_interface::SetJoystickTaskBaseFrameResponse& res);

};
}
}
