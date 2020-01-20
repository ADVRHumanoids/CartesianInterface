#include "ros/client_api/CartesianRos.h"
#include "fmt/format.h"
#include "fmt/ostream.h"

int main(int argc, char ** argv)
{

    using namespace XBot::Cartesian;

    std::string name = "arm1_8";
    ros::init(argc, argv, "ros_client_test");

    auto b = new ClientApi::TaskRos(name, ros::NodeHandle("cartesian"));

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

}
