#include <cartesian_interface/utils/RobotStatePublisher.h>
#include <cartesian_interface/utils/LoadConfig.h>

using namespace XBot::Cartesian;

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "robot_state_publisher");
    ros::NodeHandle npr("~");

    auto opt = Utils::LoadOptionsFromParamServer();

    XBot::ModelInterface::Ptr model = XBot::ModelInterface::getModel(opt);

    Eigen::VectorXd qhome;
    model->getRobotState("home", qhome);
    model->setJointPosition(qhome);
    model->update();

    Utils::RobotStatePublisher rspub(model);

    auto tf_prefix = npr.param<std::string>("tf_prefix", "");
    double rate = npr.param("rate", 100);

    auto on_timer_event = [&rspub, tf_prefix](const ros::TimerEvent& ev)
    {
        rspub.publishTransforms(ev.current_real, tf_prefix);
    };

    auto timer = npr.createTimer(ros::Duration(1./rate), on_timer_event);

    ros::spin();

}
