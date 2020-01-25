#include <gtest/gtest.h>

#include <cartesian_interface/CartesianInterfaceImpl.h>
#include "utils/DynamicLoading.h"
#include "ros/server_api/CartesianRos.h"
#include "fmt/format.h"
#include "fmt/ostream.h"

#include "testutils.h"

using namespace XBot::Cartesian;

class TestRos: public ::testing::Test
{
    void SetUp() override
    {
        _roscore.reset(new Process({"roscore", "-p", "11322"}));

        std::string path_to_cfg(CARTESIO_TEST_CONFIG_PATH);
        XBot::ConfigOptions opt;
        if(!opt.set_urdf_path(path_to_cfg + "centauro.urdf"))
        {
            throw std::runtime_error("unable to load urdf");
        }

        if(!opt.set_srdf_path(path_to_cfg + "centauro.srdf"))
        {
            throw std::runtime_error("unable to load srdf");
        }

        if(!opt.generate_jidmap())
        {
            throw std::runtime_error("unable to load jidmap");
        }

        opt.set_parameter("is_model_floating_base", true);
        opt.set_parameter<std::string>("model_type", "RBDL");

        auto model = XBot::ModelInterface::getModel(opt);
        Eigen::VectorXd qhome;
        model->getRobotState("home", qhome);
        model->setJointPosition(qhome);
        model->update();

        YAML::Node ik_yaml = YAML::LoadFile(path_to_cfg + "centauro_test_stack.yaml");

        auto ctx = Context::MakeContext(0.001);

        ProblemDescription ik_problem(ik_yaml, model);

        std::string impl_name = "OpenSot";
        std::string path_to_shared_lib = XBot::Utils::FindLib("libCartesian" + impl_name + ".so", "LD_LIBRARY_PATH");
        if (path_to_shared_lib == "")
        {
            throw std::runtime_error("libCartesian" + impl_name + ".so must be listed inside LD_LIBRARY_PATH");
        }

        ci.reset( CallFunction<CartesianInterfaceImpl*>(path_to_shared_lib,
                                                        "create_instance",
                                                        model, ik_problem) );

        _ctx = RosContext::MakeContext(ros::NodeHandle("cartesian"), "ci");
    }

    void TearDown() override
    {

    }

    std::unique_ptr<Process> _roscore;
    RosContext::Ptr _ctx;

protected:

    CartesianInterfaceImpl::Ptr ci;
};

TEST_F(TestRos, check1)
{
    auto larm = std::dynamic_pointer_cast<CartesianTask>(ci->getTask("arm1_8"));
    ASSERT_TRUE(bool(larm));

    auto larm_ros_srv = ServerApi::TaskRos::MakeInstance(larm, ci->getModel());

    larm_ros_srv.reset();

    ASSERT_EQ(larm_ros_srv.use_count(), 0);
}


int main(int argc, char ** argv)
{
    /* Run tests on an isolated roscore */
    if(setenv("ROS_MASTER_URI", "http://localhost:11322", 1) == -1)
    {
        perror("setenv");
        return 1;
    }

    ros::init(argc, argv, "ros_server_test");

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

