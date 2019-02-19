#include <unistd.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/wait.h>

#include <vector>
#include <string>
#include <stdexcept>

#include <ros/ros.h>

#include <XBotInterface/Utils.h>

#include <sensor_msgs/JointState.h>
#include <cartesian_interface/GetTaskList.h>
#include <cartesian_interface/GetTaskInfo.h>
#include <cartesian_interface/SetTaskInfo.h>

#include <cartesian_interface/ros/RosImpl.h>

#include <gtest/gtest.h>

class Process
{
    
public:
    
    Process(std::vector<const char *> args);
    
    int wait();
    
    void kill(int signal = SIGTERM);
    
    ~Process();
    
private:
    
    std::string _name;
    pid_t _pid;
    
};

Process::Process(std::vector< const char* > args):
    _name(args[0])
{
    args.push_back(nullptr);
    char ** argv = (char**)args.data();
    
    _pid = ::fork();
    
    if(_pid == -1)
    {
        perror("fork");
        throw std::runtime_error("Unable to fork()");
    }
    
    if(_pid == 0)
    {
        ::execvp(argv[0], argv);
        perror("execvp");
        throw std::runtime_error("Unknown command");
    }
    
}

int Process::wait()
{
    int status;
    while(::waitpid(_pid, &status, 0) != _pid);
    printf("Child process '%s' exited with status %d\n", _name.c_str(), status);
    return status;
        
}

void Process::kill(int signal)
{
    ::kill(_pid, signal);
    printf("Killed process '%s' with signal %d\n", _name.c_str(), signal);
}

Process::~Process()
{
    kill(SIGINT);
    wait();
}


class TestRos: public ::testing::Test { };

TEST_F(TestRos, checkServices)
{
    std::map<std::string, std::string> tasks;
    tasks["wheel_1"] = "world";
    tasks["wheel_1"] = "world";
    tasks["wheel_2"] = "world";
    tasks["wheel_3"] = "world";
    tasks["wheel_4"] = "world";
    tasks["arm1_8"] = "pelvis";
    tasks["arm2_8"] = "world";
    tasks["com"] = "world";
    
    /* Get task list */
    {
        
        ASSERT_TRUE(ros::service::waitForService("cartesian/get_task_list", ros::Duration(10.0)));
        
        cartesian_interface::GetTaskListRequest req;
        cartesian_interface::GetTaskListResponse res;
        ASSERT_TRUE(ros::service::call("cartesian/get_task_list", req, res));
        
        for(auto db : tasks)
        {
            auto it = std::find(res.distal_links.begin(), res.distal_links.end(), db.first);
            ASSERT_TRUE(it != res.distal_links.end());
            ASSERT_TRUE(res.base_links.at(it-res.distal_links.begin()) == db.second);
        }
    
    }
    
    /* Get task info */
    {
        for(auto db : tasks)
        {
            ASSERT_TRUE(ros::service::waitForService("cartesian/" + db.first + "/get_task_properties", ros::Duration(1.0)));
            
            cartesian_interface::GetTaskInfoRequest req;
            cartesian_interface::GetTaskInfoResponse res;
            ASSERT_TRUE(ros::service::call("cartesian/" + db.first + "/get_task_properties", req, res));
            ASSERT_TRUE(res.distal_link == db.first);
            ASSERT_TRUE(res.base_link == db.second);
            ASSERT_TRUE(res.control_mode == "Position");
            ASSERT_TRUE(res.task_interface == "Cartesian");
            ASSERT_TRUE(res.task_state == "Online");
        }
        
    }
    
    /* Set task info */
    {
        tasks["wheel_1"] = "pelvis";
        
        {
        ASSERT_TRUE(ros::service::waitForService("cartesian/wheel_1/set_task_properties", ros::Duration(1.0)));
        cartesian_interface::SetTaskInfoRequest req;
        req.base_link = "pelvis";
        req.control_mode = "Velocity";
        cartesian_interface::SetTaskInfoResponse res;
        ASSERT_TRUE(ros::service::call("cartesian/wheel_1/set_task_properties", req, res));
        ASSERT_TRUE(res.success);
        }
        
        {
        ASSERT_TRUE(ros::service::waitForService("cartesian/com/set_task_properties", ros::Duration(1.0)));
        cartesian_interface::SetTaskInfoRequest req;
        req.base_link = "pelvis";
        cartesian_interface::SetTaskInfoResponse res;
        ASSERT_TRUE(ros::service::call("cartesian/com/set_task_properties", req, res));
        ASSERT_TRUE(!res.success);
        }
        
        {
        cartesian_interface::SetTaskInfoRequest req;
        req.control_mode = "Disabled";
        cartesian_interface::SetTaskInfoResponse res;
        ASSERT_TRUE(ros::service::call("cartesian/com/set_task_properties", req, res));
        ASSERT_TRUE(res.success);
        }
    }
    
    /* Topics */
    auto msg = ros::topic::waitForMessage<sensor_msgs::JointState>("cartesian/solution", ros::Duration(1.0));
    ASSERT_TRUE(bool(msg));
    ASSERT_TRUE(msg->name.size() > 0);
    
    for(auto db : tasks)
    {
        auto msg = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("cartesian/" + db.first + "/state", ros::Duration(1.0));
        ASSERT_TRUE(bool(msg));
        ASSERT_TRUE(msg->header.frame_id != "");
    }
    
}

TEST_F(TestRos, checkBindings)
{
    using namespace XBot::Cartesian;
    
    RosImpl * ci = nullptr;
    
    ASSERT_NO_THROW(ci = new RosImpl);
    
    ASSERT_TRUE(ci->getControlMode("com") == ControlType::Disabled);
    ASSERT_TRUE(ci->getBaseLink("wheel_1") == "pelvis");
    
    ASSERT_NO_THROW(ci->setControlMode("arm1_8", ControlType::Velocity));
    ASSERT_TRUE(ci->getControlMode("arm1_8") == ControlType::Velocity);
    
    ASSERT_NO_THROW(ci->setControlMode("arm1_8", ControlType::Position));
    ASSERT_TRUE(ci->getControlMode("arm1_8") == ControlType::Position);
    
    ASSERT_NO_THROW(ci->setControlMode("com", ControlType::Position));
    ASSERT_TRUE(ci->getControlMode("com") == ControlType::Position);
    
    ASSERT_NO_THROW(ci->setBaseLink("wheel_1", "world"));
    ASSERT_TRUE(ci->getBaseLink("wheel_1") == "world");
    
    
    
    delete ci;
    
}

int main(int argc, char ** argv)
{                    
    /* Run tests on an isolated roscore */
    if(setenv("ROS_MASTER_URI", "http://localhost:11322", 1) == -1)
    {
        perror("setenv");
        return 1;
    }
    
    Process roscore({"roscore", "-p", "11322"});
    
    
    /* Launch ros server node */
    ros::init(argc, argv, "cartesian_interface_ros_test_node");
    ros::NodeHandle  nh("cartesian");
    
    std::string cfg_path(CARTESIO_TEST_CONFIG_PATH), urdf, srdf, prob;
    XBot::Utils::ReadFile(cfg_path + "/centauro.urdf", urdf);
    XBot::Utils::ReadFile(cfg_path + "/centauro.srdf", srdf);
    XBot::Utils::ReadFile(cfg_path + "/centauro_test_stack.yaml", prob);
    
    ros::param::set("/robot_description", urdf);
    ros::param::set("/robot_description_semantic", srdf);
    ros::param::set("/cartesian/problem_description", prob);
    
    Process proc({"rosrun", 
        "cartesian_interface", 
        "ros_server_node", 
        "_is_model_floating_base:=true", 
        "_model_type:=RBDL",
        "_solver:=OpenSot"
        });
    
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
