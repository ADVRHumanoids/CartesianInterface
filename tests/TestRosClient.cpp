#include "ros/client_api/CartesianRos.h"

int main()
{

    using namespace XBot::Cartesian;

    auto ctx = Context::MakeContext(0);

    std::string name = "s";
    CartesianRos c(name);
}
