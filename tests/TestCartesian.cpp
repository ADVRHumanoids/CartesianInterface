#include <algorithm>
#include <gtest/gtest.h>
#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <cartesian_interface/Context.h>

#include "testutils.h"

#include "cartesian_interface/sdk/problem/Interaction.h"

using namespace XBot::Cartesian;


TEST(TestCartesian, checkDefault)
{

    auto model = GetTestModel();
    auto ctx = std::make_shared<Context>(
                std::make_shared<Parameters>(.001),
                model);
    CartesianTaskImpl t(ctx, "MyTask", "arm1_8", "pelvis");

    ASSERT_TRUE(t.validate());

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

    ASSERT_EQ(t.getDistalLink(), "arm1_8");
    ASSERT_EQ(t.getBaseLink(), "pelvis");
    ASSERT_EQ(t.getTaskState(), State::Online);
    ASSERT_EQ(t.getControlMode(), ControlType::Position);
    ASSERT_EQ(t.getCurrentSegmentId(), -1);

    Eigen::Affine3d T, Ttarget, Tref, Traw;
    ASSERT_FALSE(t.getPoseTarget(Ttarget));
    ASSERT_TRUE(t.getCurrentPose(T));
    ASSERT_TRUE(t.getPoseReference(Tref));
    ASSERT_TRUE(t.getPoseReferenceRaw(Traw));

    ASSERT_EQ(T.matrix(), Tref.matrix());
    ASSERT_EQ(T.matrix(), Traw.matrix());





}



TEST(TestCartesian, checkDefaultYaml1)
{
    std::string yaml_str =
            "name: larm                         \n"
            "type: Cartesian                    \n"
            "distal_link: arm1_8                \n"
            "use_local_subtasks: true            \n"
            "orientation_gain: 5.0              \n";

    auto yaml = YAML::Load(yaml_str);

    auto model = GetTestModel();
    auto ctx = std::make_shared<Context>(
                std::make_shared<Parameters>(.001),
                model);
    CartesianTaskImpl t(yaml, ctx);

    ASSERT_EQ(t.getName(), "larm");
    ASSERT_EQ(t.getType(), "Cartesian");
    ASSERT_EQ(t.getSize(), 6);
    ASSERT_EQ(t.getIndices(), (std::vector<int>{0, 1, 2, 3, 4, 5}));
    ASSERT_EQ(t.getActivationState(), ActivationState::Enabled);
    ASSERT_EQ(t.getModel(), model);
    ASSERT_EQ(t.getWeight(), Eigen::MatrixXd::Identity(6,6));
    ASSERT_EQ(t.getDistalLink(), "arm1_8");
    ASSERT_EQ(t.getBaseLink(), "world");
    ASSERT_EQ(t.isSubtaskLocal(), true);

    ASSERT_TRUE(t.validate());

}

TEST(TestCartesian, checkDefaultYaml2)
{
    std::string yaml_str =
            "name: larm                         \n"
            "type: Cartesian                    \n"
            "distal_link: arm1_8                \n"
            "base_link: pelvis                  \n"
            "use_local_subtasks: true            \n"
            "orientation_gain: 5.0              \n";

    auto yaml = YAML::Load(yaml_str);

    auto model = GetTestModel();
    auto ctx = std::make_shared<Context>(
                std::make_shared<Parameters>(.001),
                model);
    CartesianTaskImpl t(yaml, ctx);

    ASSERT_EQ(t.getName(), "larm");
    ASSERT_EQ(t.getType(), "Cartesian");
    ASSERT_EQ(t.getSize(), 6);
    ASSERT_EQ(t.getIndices(), (std::vector<int>{0, 1, 2, 3, 4, 5}));
    ASSERT_EQ(t.getActivationState(), ActivationState::Enabled);
    ASSERT_EQ(t.getModel(), model);
    ASSERT_EQ(t.getWeight(), Eigen::MatrixXd::Identity(6,6));
    ASSERT_EQ(t.getDistalLink(), "arm1_8");
    ASSERT_EQ(t.getBaseLink(), "pelvis");
    ASSERT_EQ(t.isSubtaskLocal(), true);

    ASSERT_TRUE(t.validate());

}

TEST(TestCartesian, checkAdmittance)
{
    std::string yaml_str =
            "name: larm                             \n"
            "type: Admittance                       \n"
            "distal_link: arm1_8                    \n"
            "base_link: pelvis                      \n"
            "force_dead_zone: [10, 10, 10, 3, 3, 3] \n"
            "force_estimation_chains: [left_arm]    \n"
            "stiffness: [10, 10, 10, 3, 3, 3]       \n"
            "inertia: [10, 10, 10, 3, 3, 3]         \n"
            "damping: [10, 10, 10, 3, 3, 3]         \n";

    auto yaml = YAML::Load(yaml_str);

    auto model = GetTestModel();
    auto ctx = std::make_shared<Context>(
                std::make_shared<Parameters>(.001),
                model);
    AdmittanceTaskImpl t(yaml, ctx);

    Eigen::Vector6d vec;
    vec << 10, 10, 10, 3, 3, 3;

    Eigen::Matrix6d mat(vec.asDiagonal());

    ASSERT_EQ(t.getName(), "larm");
    ASSERT_EQ(t.getType(), "Admittance");
    ASSERT_EQ(t.getSize(), 6);
    ASSERT_EQ(t.getIndices(), (std::vector<int>{0, 1, 2, 3, 4, 5}));
    ASSERT_EQ(t.getActivationState(), ActivationState::Enabled);
    ASSERT_EQ(t.getModel(), model);
    ASSERT_EQ(t.getWeight(), Eigen::MatrixXd::Identity(6,6));
    ASSERT_EQ(t.getDistalLink(), "arm1_8");
    ASSERT_EQ(t.getBaseLink(), "pelvis");
    ASSERT_EQ(t.getForceDeadzone(), vec);
    ASSERT_EQ(t.getImpedance().stiffness, mat);
    ASSERT_EQ(t.getImpedance().damping, mat);
    ASSERT_EQ(t.getImpedance().mass, mat);
    ASSERT_EQ(t.getForceEstimationChains(), std::vector<std::string>{"left_arm"});


    ASSERT_TRUE(t.validate());

}


TEST(TestCartesian, checkObserver)
{
    std::string yaml_str =
            "name: larm                         \n"
            "type: Cartesian                    \n"
            "distal_link: arm1_8                \n"
            "base_link: pelvis                  \n"
            "use_local_subtasks: true            \n"
            "orientation_gain: 5.0              \n";

    auto yaml = YAML::Load(yaml_str);

    auto model = GetTestModel();
    auto ctx = std::make_shared<Context>(
                std::make_shared<Parameters>(.001),
                model);
    CartesianTaskImpl t(yaml, ctx);


    struct Obs : CartesianTaskObserver
    {
        virtual bool onWeightChanged(){return _on_w();}
        virtual bool onActivationStateChanged(){return _on_act();}
        virtual bool onBaseLinkChanged(){return _on_bl();}
        virtual bool onControlModeChanged(){return _on_cm();}
        virtual bool onSafetyLimitsChanged(){return _on_sl();}

        std::function<bool(void)> _on_w,
                _on_act,
                _on_bl,
                _on_cm,
                _on_sl;
    };

    auto obs = std::make_shared<Obs>();

    bool w_cb_called = false;
    bool act_cb_called = false;
    bool bl_cb_called = false;
    bool cm_cb_called = false;
    bool sl_cb_called = false;

    obs->_on_w = [&w_cb_called]()
    {
        w_cb_called = true;
        return true;
    };

    obs->_on_act = [&act_cb_called]()
    {
        act_cb_called = true;
        return true;
    };

    obs->_on_bl = [&bl_cb_called]()
    {
        bl_cb_called = true;
        return true;
    };

    obs->_on_cm = [&cm_cb_called]()
    {
        cm_cb_called = true;
        return true;
    };

    obs->_on_sl = [&sl_cb_called]()
    {
        sl_cb_called = true;
        return true;
    };


    t.registerObserver(obs);

    t.setWeight(Eigen::MatrixXd::Identity(6,6));
    ASSERT_TRUE(w_cb_called);
    w_cb_called = false;

    t.setActivationState(ActivationState::Enabled);
    ASSERT_TRUE(act_cb_called);
    act_cb_called = false;

    t.setBaseLink("cazzi");
    ASSERT_FALSE(bl_cb_called);
    t.setBaseLink("pelvis");
    ASSERT_TRUE(bl_cb_called);
    bl_cb_called = false;

    t.setControlMode(ControlType::Velocity);
    ASSERT_TRUE(cm_cb_called);
    cm_cb_called = false;

    t.setVelocityLimits(1, 1);
    ASSERT_TRUE(sl_cb_called);
    sl_cb_called = false;
    t.setAccelerationLimits(1, 1);
    ASSERT_TRUE(sl_cb_called);
    sl_cb_called = false;

    /* Check that obs lifetime is not extended by the task */
    obs.reset();
    ASSERT_EQ(obs.use_count(), 0);

    t.setWeight(Eigen::MatrixXd::Identity(6,6));
    ASSERT_FALSE(w_cb_called);
    w_cb_called = false;

    t.setActivationState(ActivationState::Enabled);
    ASSERT_FALSE(act_cb_called);
    act_cb_called = false;

    t.setBaseLink("cazzi");
    ASSERT_FALSE(bl_cb_called);
    t.setBaseLink("pelvis");
    ASSERT_FALSE(bl_cb_called);
    bl_cb_called = false;

    t.setControlMode(ControlType::Velocity);
    ASSERT_FALSE(cm_cb_called);
    cm_cb_called = false;

    t.setVelocityLimits(1, 1);
    ASSERT_FALSE(sl_cb_called);
    sl_cb_called = false;
    t.setAccelerationLimits(1, 1);
    ASSERT_FALSE(sl_cb_called);
    sl_cb_called = false;

}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
