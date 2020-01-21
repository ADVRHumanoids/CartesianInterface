#include <gtest/gtest.h>

#include "ros/client_api/CartesianRos.h"
#include "fmt/format.h"
#include "fmt/ostream.h"

#include "testutils.h"

class TestRos: public ::testing::Test
{
    void SetUp() override
    {
        _roscore.reset(new Process({"roscore", "-p", "11322"}));

        /* Launch ros server node */
        ros::NodeHandle  nh("cartesian");

        std::string cfg_path(CARTESIO_TEST_CONFIG_PATH), urdf, srdf, prob;
        XBot::Utils::ReadFile(cfg_path + "/centauro.urdf", urdf);
        XBot::Utils::ReadFile(cfg_path + "/centauro.srdf", srdf);
        XBot::Utils::ReadFile(cfg_path + "/centauro_test_stack.yaml", prob);

        ros::param::set("/robot_description", urdf);
        ros::param::set("/robot_description_semantic", srdf);
        ros::param::set("/cartesian/problem_description", prob);

        _cartesio.reset(new Process({"rosrun",
                                     "cartesian_interface",
                                     "ros_server_node",
                                     "_is_model_floating_base:=true",
                                     "_model_type:=RBDL",
                                     "_solver:=OpenSot"
                                    }));
    }

    void TearDown() override
    {

    }

    std::unique_ptr<Process> _roscore, _cartesio;
};

TEST_F(TestRos, check1)
{
    using namespace XBot::Cartesian;

    ros::NodeHandle nh("cartesian");
    std::string name = "arm1_8";

    int attempts = 5;
    while(attempts-- && !ros::service::exists("/cartesian/" + name + "/get_task_properties", true))
    {
        sleep(1);
    }

    ClientApi::TaskRos task(name, nh);

    ros::spinOnce();

    EXPECT_EQ(task.getName(), "arm1_8");
    EXPECT_EQ(task.getType(), "Cartesian");
    EXPECT_EQ(task.getSize(), 6);
    EXPECT_NEAR(task.getLambda(), 0.1, 0.0001);
    EXPECT_EQ(task.getActivationState(), ActivationState::Enabled);
    EXPECT_NEAR((task.getWeight() - Eigen::MatrixXd::Identity(6,6)).norm(), 0, 0.0001);

    ClientApi::CartesianRos cart(name, nh);

    ros::spinOnce();

    EXPECT_EQ(cart.getBaseLink(), "pelvis");
    EXPECT_EQ(cart.getDistalLink(), "arm1_8");
    EXPECT_EQ(cart.getControlMode(), ControlType::Position);
    EXPECT_EQ(cart.getTaskState(), State::Online);

    EXPECT_TRUE(task.setActivationState(ActivationState::Disabled));
    EXPECT_EQ(task.getActivationState(), ActivationState::Disabled);

    EXPECT_TRUE(task.setActivationState(ActivationState::Enabled));
    EXPECT_EQ(task.getActivationState(), ActivationState::Enabled);

    task.setLambda(1.0);
    EXPECT_NEAR(task.getLambda(), 1.0, 0.0001);

    EXPECT_THROW(task.setLambda(-0.1), std::runtime_error);
    EXPECT_THROW(task.setLambda(1.1), std::runtime_error);

    Eigen::MatrixXf w;
    w.setRandom(6, 6);
    w = w.transpose()*w;
    EXPECT_TRUE(task.setWeight(w.cast<double>()));
    EXPECT_EQ(task.getWeight(), w.cast<double>());

    EXPECT_FALSE(cart.setBaseLink("cazzi"));
    EXPECT_TRUE(cart.setBaseLink("world"));
    EXPECT_EQ(cart.getBaseLink(), "world");
    EXPECT_TRUE(cart.setBaseLink("torso_2"));
    EXPECT_EQ(cart.getBaseLink(), "torso_2");

    EXPECT_TRUE(cart.setControlMode(ControlType::Velocity));
    EXPECT_EQ(cart.getControlMode(), ControlType::Velocity);


}

int main(int argc, char ** argv)
{
    /* Run tests on an isolated roscore */
    if(setenv("ROS_MASTER_URI", "http://localhost:11322", 1) == -1)
    {
        perror("setenv");
        return 1;
    }

    ros::init(argc, argv, "ros_client_test");

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

