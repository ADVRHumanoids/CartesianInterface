#include <algorithm>
#include <gtest/gtest.h>
#include <cartesian_interface/CartesianInterfaceImpl.h>
#include "utils/DynamicLoading.h"


using namespace XBot::Cartesian;

namespace {

Eigen::Affine3d GetRandomFrame()
{
    Eigen::Quaterniond q;
    q.coeffs().setRandom();
    q.normalize();

    Eigen::Affine3d T;
    T.setIdentity();
    T.linear() = q.toRotationMatrix();
    T.translation().setRandom();

    return T;
}

}

class TestOpensot: public ::testing::Test {


protected:

    TestOpensot(){

        XBot::Logger::SetVerbosityLevel(XBot::Logger::Severity::DEBUG);

#ifndef CARTESIO_TEST_CONFIG_PATH
        throw std::runtime_error("CARTESIO_TEST_CONFIG_PATH is not defined");
#endif

        std::string path_to_cfg(CARTESIO_TEST_CONFIG_PATH);
        std::cout << __func__ << " using path " << path_to_cfg << std::endl;

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

        auto ctx = std::make_shared<Context>(
                    std::make_shared<Parameters>(.001),
                    model);

        ProblemDescription ik_problem(ik_yaml, ctx);

        std::string impl_name = "OpenSot";
        ci = CartesianInterfaceImpl::MakeInstance(impl_name, ik_problem, ctx);

    }

    virtual ~TestOpensot() {
    }

    virtual void SetUp() {

    }

    virtual void TearDown() {
    }

    CartesianInterfaceImpl::Ptr ci;

};

TEST_F(TestOpensot, checkInitialReference)
{
    for(auto tname : ci->getTaskList())
    {
        auto t = ci->getTask(tname);

        if(auto tcart = std::dynamic_pointer_cast<CartesianTask>(t))
        {
            if(tcart->getDistalLink() == "com") continue;

            Eigen::Affine3d Tmodel, T, Tref;
            Eigen::Vector6d v, a;
            tcart->getCurrentPose(T);
            tcart->getPoseReference(Tref, &v, &a);
            ASSERT_TRUE(ci->getModel()->getPose(tcart->getDistalLink(),
                                    tcart->getBaseLink(),
                                    Tmodel));
            ASSERT_EQ(v.norm(), 0.0);
            ASSERT_EQ(a.norm(), 0.0);
            ASSERT_NEAR((T.matrix()-Tref.matrix()).norm(), 0, 0.0);
            ASSERT_NEAR((T.matrix()-Tmodel.matrix()).norm(), 0, 0.00);

        }

        if(auto tcom = std::dynamic_pointer_cast<ComTask>(t))
        {
            Eigen::Affine3d Tmodel, T, Tref;
            Eigen::Vector3d com_model;
            Eigen::Vector6d v, a;
            Tmodel.setIdentity();
            tcom->getCurrentPose(T);
            tcom->getPoseReference(Tref, &v, &a);
            ci->getModel()->getCOM(com_model);
            Tmodel.translation() = com_model;
            ASSERT_EQ(v.norm(), 0.0);
            ASSERT_EQ(a.norm(), 0.0);
            ASSERT_NEAR((T.matrix()-Tref.matrix()).norm(), 0, 0.00);
            ASSERT_NEAR((T.matrix()-Tmodel.matrix()).norm(), 0, 0.0);
        }

        if(auto tpostur = std::dynamic_pointer_cast<PosturalTask>(t))
        {
            Eigen::VectorXd qp;
            tpostur->getReferencePosture(qp);

            Eigen::VectorXd q0;
            ci->getModel()->getJointPosition(q0);

            ASSERT_EQ(qp, q0);
        }

    }

}

TEST_F(TestOpensot, checkNoInitialMotion)
{
    Eigen::VectorXd q0;
    ci->getModel()->getJointPosition(q0);

    double time = 0;
    double dt = ci->getContext()->params()->getControlPeriod();

    while(time < 1.0)
    {
        ASSERT_TRUE(ci->update(time, dt));

        time += dt;

        Eigen::VectorXd q, dq;
        ci->getModel()->getJointPosition(q);
        ci->getModel()->getJointVelocity(dq);

        q += dq * dt;
        ci->getModel()->setJointPosition(q);
        ci->getModel()->update();

        EXPECT_LE((q - q0).norm(), 0.001);
        EXPECT_LE(dq.norm(), 0.001);

    }
};

TEST_F(TestOpensot, checkMotion)
{
    Eigen::VectorXd q0;
    ci->getModel()->getJointPosition(q0);

    double time = 0;
    double dt = ci->getContext()->params()->getControlPeriod();

    Eigen::Affine3d Tref;
    ci->getCurrentPose("arm1_8", Tref);
    Tref.translation().x() += 0.1;

    ci->setTargetPose("arm1_8", Tref, 1.0);

    Eigen::VectorXd q, dq;
    while(ci->getTaskState("arm1_8") == State::Reaching)
    {
        ASSERT_TRUE(ci->update(time, dt));

        time += dt;

        ci->getModel()->getJointPosition(q);
        ci->getModel()->getJointVelocity(dq);

        q += dq * dt;
        ci->getModel()->setJointPosition(q);
        ci->getModel()->update();

        Eigen::Affine3d T;
        ci->getCurrentPose("arm1_8", T);

    }

    Eigen::Affine3d T;
    ci->getCurrentPose("arm1_8", T);

    EXPECT_TRUE(T.isApprox(Tref, 0.01));
};

TEST_F(TestOpensot, checkVelocityCtrl)
{
    auto t = ci->getTask("arm1_8");
    ASSERT_TRUE(t != nullptr);

    auto ct = std::dynamic_pointer_cast<CartesianTask>(t);
    ASSERT_TRUE(ct != nullptr);

    Eigen::Affine3d pose0;
    ct->getCurrentPose(pose0);

    ASSERT_TRUE(ct->setControlMode(ControlType::Velocity));
    EXPECT_EQ(ct->getControlMode(), ControlType::Velocity);

    Eigen::Vector6d vref;
    vref << 0.1, 0, 0, 0, 0, 0;

    double time = 0.0;
    double dt = ci->getContext()->params()->getControlPeriod();

    Eigen::VectorXd q, dq;
    for(int i = 0; i < 1.0/dt; i++)
    {
        EXPECT_TRUE(ct->setVelocityReference(vref));
        ci->update(time, dt);

        time += dt;

        ci->getModel()->getJointPosition(q);
        ci->getModel()->getJointVelocity(dq);
        q += dq * dt;
        ci->getModel()->setJointPosition(q);
        ci->getModel()->update();
    }

    Eigen::Affine3d pose1;
    ct->getCurrentPose(pose1);

    // we should've moved ~= 0.1 m
    EXPECT_NEAR(pose0.translation().x() + 0.1,
                pose1.translation().x(),
                0.01);

    ASSERT_TRUE(ct->setControlMode(ControlType::Position));
    EXPECT_EQ(ct->getControlMode(), ControlType::Position);

    for(int i = 0; i < 1.0/dt; i++)
    {
        ci->update(time, dt);

        ci->getModel()->getJointPosition(q);
        ci->getModel()->getJointVelocity(dq);
        q += dq * dt;
        ci->getModel()->setJointPosition(q);
        ci->getModel()->update();
    }

    Eigen::Affine3d pose2;
    ct->getCurrentPose(pose2);

    EXPECT_TRUE(pose2.isApprox(pose1));

};


int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
