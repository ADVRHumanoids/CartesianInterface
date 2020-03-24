#include <algorithm>
#include <gtest/gtest.h>
#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <cartesian_interface/Context.h>

#include "cartesian_interface/sdk/problem/Task.h"

#include "testutils.h"

using namespace XBot::Cartesian;

TEST(TestTask, checkDefault)
{
    auto model = GetTestModel();
    auto ctx = std::make_shared<Context>(
                std::make_shared<Parameters>(.001),
                model);
    TaskDescriptionImpl t("Cartesian", "MyTask", 6, ctx);

    ASSERT_EQ(t.getName(), "MyTask");
    ASSERT_EQ(t.getType(), "Cartesian");
    ASSERT_EQ(t.getSize(), 6);
    ASSERT_EQ(t.getLambda(), 1.0);
    ASSERT_EQ(t.getIndices(), (std::vector<int>{0, 1, 2, 3, 4, 5}));
    ASSERT_EQ(t.getActivationState(), ActivationState::Enabled);
    ASSERT_EQ(t.getModel(), model);
    ASSERT_EQ(t.getWeight(), Eigen::MatrixXd::Identity(6,6));
    ASSERT_EQ(t.getLibName(), "");
    ASSERT_TRUE(t.getDisabledJoints().empty());

    ASSERT_TRUE(t.validate());

}

TEST(TestTask, checkDefaultYaml)
{
    std::string yaml_str = "type: Cartesian \n";
    auto yaml = YAML::Load(yaml_str);

    auto model = GetTestModel();
    auto ctx = std::make_shared<Context>(
                std::make_shared<Parameters>(.001),
                model);
    TaskDescriptionImpl t(yaml, ctx, "MyTask", 6);

    ASSERT_EQ(t.getName(), "MyTask");
    ASSERT_EQ(t.getType(), "Cartesian");
    ASSERT_EQ(t.getSize(), 6);
    ASSERT_EQ(t.getLambda(), 1.0);
    ASSERT_EQ(t.getIndices(), (std::vector<int>{0, 1, 2, 3, 4, 5}));
    ASSERT_EQ(t.getActivationState(), ActivationState::Enabled);
    ASSERT_EQ(t.getModel(), model);
    ASSERT_EQ(t.getWeight(), Eigen::MatrixXd::Identity(6,6));
    ASSERT_EQ(t.getLibName(), "");
    ASSERT_TRUE(t.getDisabledJoints().empty());

    ASSERT_TRUE(t.validate());
}

TEST(TestTask, checkDefaultYaml1)
{
    std::string yaml_str =
            "type: Cartesian                         \n"
            "lambda: 0.1                             \n"
            "weight: [1, 2, 3, 4, 5, 6]              \n"
            "indices: [0, 1, 2]                      \n"
            "lib_name: libCazzi.so                   \n"
            "disabled_joints: [j_arm1_7, torso_yaw]  \n";

    auto yaml = YAML::Load(yaml_str);

    auto model = GetTestModel();
    auto ctx = std::make_shared<Context>(
                std::make_shared<Parameters>(.001),
                model);
    TaskDescriptionImpl t(yaml, ctx, "MyTask", 6);

    Eigen::VectorXd expected_weight(6);
    expected_weight << 1, 2, 3, 4, 5, 6;

    ASSERT_EQ(t.getName(), "MyTask");
    ASSERT_EQ(t.getType(), "Cartesian");
    ASSERT_EQ(t.getSize(), 6);
    ASSERT_EQ(t.getLambda(), .1);
    ASSERT_EQ(t.getIndices(), (std::vector<int>{0, 1, 2}));
    ASSERT_EQ(t.getActivationState(), ActivationState::Enabled);
    ASSERT_EQ(t.getModel(), model);
    ASSERT_EQ(t.getWeight(), Eigen::MatrixXd(expected_weight.asDiagonal()));
    ASSERT_EQ(t.getLibName(), "libCazzi.so");
    ASSERT_EQ(t.getDisabledJoints(), (std::vector<std::string>{"j_arm1_7", "torso_yaw"}));

    ASSERT_TRUE(t.validate());

}

TEST(TestTask, checkDefaultYaml3)
{
    std::string yaml_str =
            "type: Cartesian                         \n"
            "active: false                           \n"
            "lambda: 0.1                             \n"
            "weight: [1, 2, 3, 4, 5, 6]              \n"
            "indices: [0, 1, 2]                      \n"
            "lib_name: libCazzi.so                   \n"
            "disabled_joints: [j_arm1_7, torso_yaw]  \n";

    auto yaml = YAML::Load(yaml_str);

    auto model = GetTestModel();
    auto ctx = std::make_shared<Context>(
                std::make_shared<Parameters>(.001),
                model);
    TaskDescriptionImpl t(yaml, ctx, "MyTask", 6);

    Eigen::VectorXd expected_weight(6);
    expected_weight << 1, 2, 3, 4, 5, 6;

    ASSERT_EQ(t.getName(), "MyTask");
    ASSERT_EQ(t.getType(), "Cartesian");
    ASSERT_EQ(t.getSize(), 6);
    ASSERT_EQ(t.getLambda(), .1);
    ASSERT_EQ(t.getIndices(), (std::vector<int>{0, 1, 2}));
    ASSERT_EQ(t.getActivationState(), ActivationState::Disabled);
    ASSERT_EQ(t.getModel(), model);
    ASSERT_EQ(t.getWeight(), Eigen::MatrixXd(expected_weight.asDiagonal()));
    ASSERT_EQ(t.getLibName(), "libCazzi.so");
    ASSERT_EQ(t.getDisabledJoints(), (std::vector<std::string>{"j_arm1_7", "torso_yaw"}));

    ASSERT_TRUE(t.validate());

}

TEST(TestTask, checkDefaultYaml2)
{
    std::string yaml_str =
            "type: Cartesian                         \n"
            "active: true                            \n"
            "weight: 10.0                            \n"
            "indices: [0, 1, 2]                      \n"
            "lib_name: libCazzi.so                   \n"
            "enabled_joints: [j_arm1_7, torso_yaw]   \n";

    auto yaml = YAML::Load(yaml_str);

    auto model = GetTestModel();
    auto ctx = std::make_shared<Context>(
                std::make_shared<Parameters>(.001),
                model);
    TaskDescriptionImpl t(yaml, ctx, "MyTask", 6);

    Eigen::VectorXd expected_weight(6);
    expected_weight << 10, 10, 10, 10, 10, 10;

    auto joints = model->getEnabledJointNames();
    joints.erase(std::remove(joints.begin(), joints.end(), "j_arm1_7"),
                 joints.end());
    joints.erase(std::remove(joints.begin(), joints.end(), "torso_yaw"),
                 joints.end());

    ASSERT_EQ(t.getName(), "MyTask");
    ASSERT_EQ(t.getType(), "Cartesian");
    ASSERT_EQ(t.getSize(), 6);
    ASSERT_EQ(t.getLambda(), 1.0);
    ASSERT_EQ(t.getIndices(), (std::vector<int>{0, 1, 2}));
    ASSERT_EQ(t.getActivationState(), ActivationState::Enabled);
    ASSERT_EQ(t.getModel(), model);
    ASSERT_EQ(t.getWeight(), Eigen::MatrixXd(expected_weight.asDiagonal()));
    ASSERT_EQ(t.getLibName(), "libCazzi.so");
    ASSERT_EQ(t.getDisabledJoints(), joints);

    ASSERT_TRUE(t.validate());

}

TEST(TestTask, checkGetSet)
{
    std::string yaml_str = "type: Cartesian \n";
    auto yaml = YAML::Load(yaml_str);

    auto model = GetTestModel();
    auto ctx = std::make_shared<Context>(
                std::make_shared<Parameters>(.001),
                model);
    TaskDescriptionImpl t(yaml, ctx, "MyTask", 6);

    Eigen::MatrixXd w = 2*Eigen::MatrixXd::Identity(t.getSize(), t.getSize());

    t.setLambda(0.5);
    t.setWeight(w);
    t.setIndices({0, 4, 5});
    t.setActivationState(ActivationState::Disabled);


    ASSERT_EQ(t.getName(), "MyTask");
    ASSERT_EQ(t.getType(), "Cartesian");
    ASSERT_EQ(t.getSize(), 6);
    ASSERT_EQ(t.getIndices(), (std::vector<int>{0, 4, 5}));
    ASSERT_EQ(t.getActivationState(), ActivationState::Disabled);
    ASSERT_EQ(t.getModel(), model);
    ASSERT_EQ(t.getWeight(), w);
    ASSERT_EQ(t.getLibName(), "");
    ASSERT_TRUE(t.getDisabledJoints().empty());

    ASSERT_TRUE(t.validate());
}

TEST(TestTask, checkObserver)
{
    std::string yaml_str = "type: Cartesian \n";
    auto yaml = YAML::Load(yaml_str);

    auto model = GetTestModel();
    auto ctx = std::make_shared<Context>(
                std::make_shared<Parameters>(.001),
                model);
    TaskDescriptionImpl t(yaml, ctx, "MyTask", 6);

    struct Obs : TaskObserver
    {
        virtual bool onWeightChanged(){return _on_w();}
        virtual bool onActivationStateChanged(){return _on_act();}

        std::function<bool(void)> _on_w, _on_act;
    };

    auto obs = std::make_shared<Obs>();

    bool w_cb_called = false;
    bool act_cb_called = false;

    auto on_w = [&w_cb_called]()
    {
        w_cb_called = true;
        return true;
    };

    auto on_act = [&act_cb_called]()
    {
        act_cb_called = true;
        return true;
    };

    obs->_on_w = on_w;
    obs->_on_act = on_act;

    t.registerObserver(obs);

    t.setWeight(Eigen::MatrixXd::Identity(6,6));
    ASSERT_TRUE(w_cb_called);
    ASSERT_FALSE(act_cb_called);

    t.setActivationState(ActivationState::Enabled);
    ASSERT_TRUE(w_cb_called);
    ASSERT_TRUE(act_cb_called);

}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
