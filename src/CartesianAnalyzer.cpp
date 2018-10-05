#include <cartesian_interface/utils/Manipulability.h>
#include <sensor_msgs/JointState.h>


using namespace XBot::Cartesian;

ManipulabilityAnalyzer * __g_manip;
XBot::ModelInterface::Ptr _model;


void solution_callback(const sensor_msgs::JointStateConstPtr& msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cartesian_analyzer");
    ros::NodeHandle nh("cartesian"), nh_priv("~");
    
    XBot::Logger::SetVerbosityLevel(XBot::Logger::Severity::HIGH);
    
    YAML::Node problem_yaml;
    auto xbot_cfg = Utils::LoadOptionsFromParamServer(problem_yaml);
    
    _model = XBot::ModelInterface::getModel(xbot_cfg);
    
    ProblemDescription ik_problem(problem_yaml, _model);
    
    auto solution_sub = nh.subscribe<sensor_msgs::JointState>("solution", 1, solution_callback);
    
    ManipulabilityAnalyzer manip(_model, ik_problem, "ci");
    __g_manip = &manip;
    
    ros::spin();

    return EXIT_SUCCESS;
    
}


void solution_callback(const sensor_msgs::JointStateConstPtr& msg)
{
    /* Sync from current ik solution */
    XBot::JointNameMap jnamemap;
    if(msg->name.size() != msg->position.size())
    {
        throw std::runtime_error("msg->name.size() != msg->position.size()");
    }
    
    for(int i = 0; i < msg->name.size(); i++)
    {
        jnamemap[msg->name.at(i)] = msg->position.at(i);
    }
    
    _model->setJointPosition(jnamemap);
    _model->update();
    
    
    __g_manip->compute();
    
}

