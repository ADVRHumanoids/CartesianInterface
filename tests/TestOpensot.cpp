#include <algorithm>
#include <gtest/gtest.h>
#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <cartesian_interface/utils/LoadObject.hpp>

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

        YAML::Node ik_yaml = YAML::LoadFile(path_to_cfg + "centauro_test_stack.yaml");

        auto ctx = Context::MakeContext(0.001);

        ProblemDescription ik_problem(ik_yaml, model);

        std::string impl_name = "OpenSot";
        std::string path_to_shared_lib = XBot::Utils::FindLib("libCartesian" + impl_name + ".so", "LD_LIBRARY_PATH");
        if (path_to_shared_lib == "")
        {
            throw std::runtime_error("libCartesian" + impl_name + ".so must be listed inside LD_LIBRARY_PATH");
        }
        
        ci = Utils::LoadObject<CartesianInterfaceImpl>(path_to_shared_lib,
                                                       "create_instance",
                                                       model, ik_problem);

    }

    virtual ~TestOpensot() {
    }

    virtual void SetUp() {

    }

    virtual void TearDown() {
    }

    CartesianInterfaceImpl::Ptr ci;

};

TEST_F(TestOpensot, checkNoInitialMotion)
{
    Eigen::VectorXd q0;
    ci->getModel()->getJointPosition(q0);

    double time = 0;
    double dt = Context().getControlPeriod();

    Eigen::VectorXd q, dq;
    while(time < 1.0)
    {
        ci->update(time, dt);

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
    double dt = Context().getControlPeriod();

    Eigen::Affine3d Tref;
    ci->getCurrentPose("arm1_8", Tref);
    Tref.translation().x() += 0.1;

    ci->setTargetPose("arm1_8", Tref, 1.0);

    Eigen::VectorXd q, dq;
    while(ci->getTaskState("arm1_8") == State::Reaching)
    {
        ci->update(time, dt);

        time += dt;

        ci->getModel()->getJointPosition(q);
        ci->getModel()->getJointVelocity(dq);

        q += dq * dt;
        ci->getModel()->setJointPosition(q);
        ci->getModel()->update();

        Eigen::Affine3d T;
        ci->getCurrentPose("arm1_8", T);

        std::cout << T.matrix() << std::endl;

    }

    Eigen::Affine3d T;
    ci->getCurrentPose("arm1_8", T);

    EXPECT_TRUE(T.isApprox(Tref));
};


int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
