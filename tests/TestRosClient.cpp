#include <gtest/gtest.h>

#include <cartesian_interface/ros/RosImpl.h>
#include "ros/client_api/CartesianRos.h"
#include "fmt/format.h"
#include "fmt/ostream.h"

#include "testutils.h"

using namespace XBot::Cartesian;

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

        std::string binary_dir(CARTESIO__TEST_BINARY_PATH);
        _cartesio.reset(new Process({binary_dir + "/ros_server_node",
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

TEST_F(TestRos, checkClient)
{
    RosClient cli;

    auto tlist = cli.getTaskList();

    EXPECT_FALSE(std::find(tlist.begin(), tlist.end(), "arm1_8") == tlist.end());
    EXPECT_FALSE(std::find(tlist.begin(), tlist.end(), "arm2_8") == tlist.end());
    EXPECT_FALSE(std::find(tlist.begin(), tlist.end(), "wheel_1") == tlist.end());
    EXPECT_FALSE(std::find(tlist.begin(), tlist.end(), "wheel_2") == tlist.end());
    EXPECT_FALSE(std::find(tlist.begin(), tlist.end(), "wheel_3") == tlist.end());
    EXPECT_FALSE(std::find(tlist.begin(), tlist.end(), "wheel_4") == tlist.end());
    EXPECT_FALSE(std::find(tlist.begin(), tlist.end(), "JointLimits") == tlist.end());
    EXPECT_FALSE(std::find(tlist.begin(), tlist.end(), "VelocityLimits") == tlist.end());
    EXPECT_FALSE(std::find(tlist.begin(), tlist.end(), "Com") == tlist.end());
    EXPECT_FALSE(std::find(tlist.begin(), tlist.end(), "ComXY") == tlist.end());
    EXPECT_FALSE(std::find(tlist.begin(), tlist.end(), "ComZ") == tlist.end());
    EXPECT_FALSE(std::find(tlist.begin(), tlist.end(), "Postural") == tlist.end());
    EXPECT_EQ(tlist.size(), 12);

    auto c = std::dynamic_pointer_cast<CartesianTask>(cli.getTask("arm1_8"));
    EXPECT_TRUE(bool(c));

    c = std::dynamic_pointer_cast<CartesianTask>(cli.getTask("arm2_8"));
    EXPECT_TRUE(bool(c));

    c = std::dynamic_pointer_cast<CartesianTask>(cli.getTask("wheel_1"));
    EXPECT_TRUE(bool(c));

    c = std::dynamic_pointer_cast<CartesianTask>(cli.getTask("wheel_2"));
    EXPECT_TRUE(bool(c));

    c = std::dynamic_pointer_cast<CartesianTask>(cli.getTask("wheel_3"));
    EXPECT_TRUE(bool(c));

    c = std::dynamic_pointer_cast<CartesianTask>(cli.getTask("wheel_4"));
    EXPECT_TRUE(bool(c));

    auto p = std::dynamic_pointer_cast<PosturalTask>(cli.getTask("Postural"));
    EXPECT_TRUE(bool(p));


}

TEST_F(TestRos, checkCartesian)
{

    RosClient cli;
    std::string name = "arm1_8";

    auto& task = *cli.getTask(name);

    ros::spinOnce();

    EXPECT_EQ(task.getName(), "arm1_8");
    EXPECT_EQ(task.getType(), "Cartesian");
    EXPECT_EQ(task.getSize(), 6);
    EXPECT_NEAR(task.getLambda(), 0.1, 0.0001);
    EXPECT_EQ(task.getActivationState(), ActivationState::Enabled);
    EXPECT_NEAR((task.getWeight() - Eigen::MatrixXd::Identity(6,6)).norm(), 0, 0.0001);

    auto& cart = dynamic_cast<CartesianTask&>(task);

    ros::spinOnce();

    EXPECT_EQ(cart.getBaseLink(), "pelvis");
    EXPECT_EQ(cart.getDistalLink(), "arm1_8");
    EXPECT_EQ(cart.getControlMode(), ControlType::Position);
    EXPECT_EQ(cart.getTaskState(), State::Online);

    EXPECT_TRUE(task.setActivationState(ActivationState::Disabled));
    EXPECT_EQ  (task.getActivationState(), ActivationState::Disabled);

    EXPECT_TRUE(task.setActivationState(ActivationState::Enabled));
    EXPECT_EQ  (task.getActivationState(), ActivationState::Enabled);

    task.setLambda(1.0);
    EXPECT_NEAR(task.getLambda(), 1.0, 0.0001);

    EXPECT_THROW(task.setLambda(-0.1), std::runtime_error);
    EXPECT_THROW(task.setLambda(1.1), std::runtime_error);

    Eigen::MatrixXf w, wrong;
    w.setRandom(6, 6);
    w = w.transpose()*w;
    EXPECT_TRUE(task.setWeight(w.cast<double>()));
    EXPECT_EQ  (task.getWeight(), w.cast<double>());

    wrong.setRandom(6, 1);
    EXPECT_FALSE(task.setWeight(wrong.cast<double>()));
    EXPECT_EQ  (task.getWeight(), w.cast<double>());

    EXPECT_FALSE(cart.setBaseLink("cazzi"));
    EXPECT_TRUE (cart.setBaseLink("world"));
    EXPECT_EQ   (cart.getBaseLink(), "world");
    EXPECT_TRUE (cart.setBaseLink("torso_2"));
    EXPECT_EQ   (cart.getBaseLink(), "torso_2");

    EXPECT_TRUE(cart.setControlMode(ControlType::Velocity));
    EXPECT_EQ  (cart.getControlMode(), ControlType::Velocity);


}

TEST_F(TestRos, checkPostural)
{
    RosClient cli;

    auto postural_task = std::dynamic_pointer_cast<PosturalTask>(cli.getTask("Postural"));

    ASSERT_TRUE(bool(postural_task));

    EXPECT_NEAR(postural_task->getLambda(), 0.01, 0.0001);

    Eigen::MatrixXd w;
    w.setIdentity(postural_task->getSize(), postural_task->getSize());
    w.diagonal().head<6>().setZero();
    EXPECT_EQ(postural_task->getWeight(), w);

    XBot::JointNameMap qref;
    postural_task->getReferencePosture(qref);

    EXPECT_EQ(qref.size(), postural_task->getSize());

    qref.clear();
    qref["j_arm1_4"] = 0.0;
    postural_task->setReferencePosture(qref);

    sleep(1);

    cli.update(0,0);

    postural_task->getReferencePosture(qref);

    EXPECT_EQ(qref.at("j_arm1_4"), 0.0);

}

//TEST_F(TestRos, checkLifetime)
//{
//}


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

