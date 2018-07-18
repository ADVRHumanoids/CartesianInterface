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

    static const std::string buttons_string(){ return "buttons";}
    static const std::string axes_string(){ return "axes";}

    static comand getComandFromMsg(const sensor_msgs::Joy::ConstPtr& joy, const double sens = 0.99)
    {
        axes a;
        for(unsigned int i = 0; i < joy->axes.size(); ++i)
        {
            if(fabs(joy->axes[i]) >= sens)
                a.push_back(i);
        }

        buttons b;
        for(unsigned int i = 0; i < joy->buttons.size(); ++i)
        {
            if(joy->buttons[i] == 1)
                b.push_back(i);
        }

        return std::pair<axes,buttons>(a,b);
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

    bool compareComands(const comand& A, const comand& B)
    {
        axes Aa = A.first;
        buttons Ab = A.second;
        axes Ba = B.first;
        buttons Bb = B.second;

        return compare<axes>(Aa, Ba) && compare<buttons>(Ab, Bb);
    }

    template<typename T>
    bool compare(const T& A, const T& B)
    {
        if(A.size() != B.size())
            return false;

        //check if all the elments in A are in B
        for(unsigned int i = 0; i < A.size(); ++i)
        {
            if(!(std::find(B.begin(), B.end(), A[i]) != B.end()))
                return false;
        }
        return true;
    }

    action getAction(const comand& cmd)
    {
        action act = "";

        for(unsigned int i = 0; i < actions().size(); ++i)
        {
            if(compareComands(cmd, getComand(actions()[i]))){
                act = actions()[i];
                break;}
        }
        return act;
    }


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

    bool remapCommands(XmlRpc::XmlRpcValue& list)
    {
        for(unsigned int i = 0; i < JoyStickRemap::actions().size(); ++i)
        {
            if(!list.hasMember(JoyStickRemap::actions()[i]))
            {
                ROS_ERROR("%s is missing in /xbot/cartesian/joy/ param", JoyStickRemap::actions()[i].c_str());
                return false;
            }
            else
            {
                XmlRpc::XmlRpcValue sub_list = list[JoyStickRemap::actions()[i]];

                //ROS_INFO("%s mapping: ", JoyStickRemap::actions()[i].c_str());

                JoyStickRemap::buttons buttons;
                if(sub_list.hasMember(JoyStickRemap::buttons_string()))
                {
                    //ROS_INFO("  buttons: ");
                    XmlRpc::XmlRpcValue button_list = sub_list[JoyStickRemap::buttons_string()];

                    for(unsigned int i = 0; i < button_list.size(); ++i){
                        buttons.push_back(static_cast<int>(button_list[i]));
                        //ROS_INFO("      %i", buttons.back());
                    }
                }

                JoyStickRemap::axes axes;
                if(sub_list.hasMember(JoyStickRemap::axes_string()))
                {
                    //ROS_INFO("  axes: ");
                    XmlRpc::XmlRpcValue axes_list = sub_list[JoyStickRemap::axes_string()];

                    for(unsigned int i = 0; i < axes_list.size(); ++i){
                        axes.push_back(static_cast<int>(axes_list[i]));
                        //ROS_INFO("      %i", axes.back());
                    }
                }

                setMapping(axes,buttons,JoyStickRemap::actions()[i]);

            }
        }
        ROS_INFO(Doc().c_str());
        return true;
    }

    std::string Doc()
    {
        std::stringstream doc;
        for(unsigned int i = 0; i < JoyStickRemap::actions().size(); ++i)
        {
            doc<<"\n"<<JoyStickRemap::actions()[i]<<"\n";
            if(!(getButtons(JoyStickRemap::actions()[i]).empty()))
            {
                doc<<"  buttons: [ ";
                buttons b = getButtons(JoyStickRemap::actions()[i]);
                for(unsigned int j = 0; j < b.size(); ++j)
                    doc<<b[j]<<" ";
                doc<<"]\n";
            }

            if(!(getAxes(JoyStickRemap::actions()[i]).empty()))
            {
                doc<<"  axes: [ ";
                axes a = getAxes(JoyStickRemap::actions()[i]);
                for(unsigned int j = 0; j < a.size(); ++j)
                    doc<<a[j]<<" ";
                doc<<"]\n";
            }
        }
        return doc.str();
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

    bool remap(XmlRpc::XmlRpcValue list);

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

    JoyStickRemap _jremap;
    bool _remapped = false;

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


};
}
}
