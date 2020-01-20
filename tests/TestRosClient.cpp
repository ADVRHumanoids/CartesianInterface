#include "ros/client_api/CartesianRos.h"
#include "fmt/format.h"
#include "fmt/ostream.h"

int main(int argc, char ** argv)
{

    using namespace XBot::Cartesian;

    std::string name = "arm1_8";
    ros::init(argc, argv, "ros_client_test");

    auto b = new ClientApi::TaskRos(name,
                                    ros::NodeHandle("cartesian"));

    std::cout << fmt::format("Name {} \n"
                             "Type {} \n"
                             "Size {} \n"
                             "Lambda {} \n"
                             "Activation {} \n"
                             "Weight {} \n",
                             b->getName(),
                             b->getType(),
                             b->getSize(),
                             b->getLambda(),
                             EnumToString(b->getActivationState()),
                             b->getWeight());

    auto c = new ClientApi::CartesianRos(name,
                                         ros::NodeHandle("cartesian"));

    fmt::print(stdout,
               "Base link {} \n"
               "Distal link {} \n"
               "Control mode {} \n"
               "Task state {} \n",
               c->getBaseLink(),
               c->getDistalLink(),
               EnumToString(c->getControlMode()),
               EnumToString(c->getTaskState()));

}
