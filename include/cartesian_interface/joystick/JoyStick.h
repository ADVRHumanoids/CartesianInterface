#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/TwistStamped.h>
#include <eigen3/Eigen/Dense>
#include <tf/transform_listener.h>
#include <initializer_list>


namespace XBot { namespace Cartesian {

class JoyStickRemap{
public:
    typedef std::vector<int> axes;
    typedef std::vector<int> buttons;
    typedef std::pair<axes, buttons> comand;
    typedef std::string action;
    typedef std::map<action, comand> map_comand;

    static const std::string buttons_string(){

        return "buttons";

    }


    static const std::vector<action> actions()
    {
        char* c[] = {
                         "IncrLinSpeed",
                         "DecrLinSpeed",
                         "IncrAngSpeed",
                         "DecrAngSpeed",
                         "NextTask",
                         "PrevTask",
                         "X",
                         "Y",
                         "Z",
                         "Roll",
                         "Pitch",
                         "Yaw",
                         "select",
                         "start",
                         "LocalGlobal"
                    };
        int s = sizeof(c) / sizeof(c[0]);
        return std::vector<action>(c, c+s);}


    comand getComand(const action act)
    {
        return _map_actions[act];
    }

    buttons getAxes(const action act)
    {
        return _map_actions[act].first;
    }

    buttons getButtons(const action act)
    {
        return _map_actions[act].second;
    }

    void setMapping(const axes& ax, const buttons& but, const action& act)
    {
        comand cmd(ax, but);
        _map_actions[act] = cmd;
    }

private:
    map_comand _map_actions;

};

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
        "   B button:                       Activate/Deactivate Task\n"
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
    void activateDeactivateTask();
    int _selected_task;

    std::vector<ros::ServiceClient> _set_properties_service_clients;
    std::vector<ros::ServiceClient> _get_properties_service_clients;
    std::vector<ros::Publisher> _ref_pose_pubs;
    std::vector<ros::ServiceClient> _task_active_service_client;

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
