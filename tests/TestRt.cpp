#include <algorithm>
#include <gtest/gtest.h>

#include "utils/LockfreeBufferImpl.h"
#include "utils/DynamicLoading.h"

#include "testutils.h"

using namespace XBot::Cartesian;

class TestRt: public ::testing::Test
{
    void SetUp() override
    {
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

        auto ctx = Context::MakeContext(0.001);

        ProblemDescription ik_problem(ik_yaml, model);

        std::string impl_name = "OpenSot";
        std::string path_to_shared_lib = XBot::Utils::FindLib("libCartesian" + impl_name + ".so", "LD_LIBRARY_PATH");
        if (path_to_shared_lib == "")
        {
            throw std::runtime_error("libCartesian" + impl_name + ".so must be listed inside LD_LIBRARY_PATH");
        }

        ci_rt.reset( CallFunction<CartesianInterfaceImpl*>(path_to_shared_lib,
                                                           "create_instance",
                                                           model, ik_problem) );


        auto model_lf = XBot::ModelInterface::getModel(opt);
        ci_lf = std::make_shared<LockfreeBufferImpl>(ci_rt.get(), model_lf);

    }

    void TearDown() override
    {

    }

protected:

    LockfreeBufferImpl::Ptr ci_lf;
    CartesianInterfaceImpl::Ptr ci_rt;

};

TEST_F(TestRt, checkTaskList)
{
    EXPECT_TRUE(ci_lf->getTaskList() == ci_rt->getTaskList());

    for(auto tname : ci_lf->getTaskList())
    {
        auto tlf = ci_lf->getTask(tname);
        auto trt = ci_rt->getTask(tname);

        EXPECT_EQ(tlf->getName(), trt->getName());
        EXPECT_EQ(tlf->getSize(), trt->getSize());
        EXPECT_EQ(tlf->getType(), trt->getType());
        EXPECT_EQ(tlf->getLambda(), trt->getLambda());
        EXPECT_EQ(tlf->getWeight(), trt->getWeight());
        EXPECT_EQ(tlf->getIndices(), trt->getIndices());
        EXPECT_EQ(tlf->getActivationState(), trt->getActivationState());

        if(auto cartrt = std::dynamic_pointer_cast<CartesianTask>(trt))
        {
            auto cartlf = std::dynamic_pointer_cast<CartesianTask>(tlf);
            EXPECT_TRUE(bool(cartlf));

            EXPECT_EQ(cartlf->getBaseLink(), cartrt->getBaseLink());
            EXPECT_EQ(cartlf->getTaskState(), cartrt->getTaskState());
            EXPECT_EQ(cartlf->getDistalLink(), cartrt->getDistalLink());
            EXPECT_EQ(cartlf->getControlMode(), cartrt->getControlMode());

            Eigen::Affine3d Tlf, Trt;

            cartlf->getPoseReference(Tlf);
            cartrt->getPoseReference(Trt);
            EXPECT_EQ(Tlf.matrix(), Trt.matrix());

            cartlf->getCurrentPose(Tlf);
            cartrt->getCurrentPose(Trt);
            EXPECT_EQ(Tlf.matrix(), Trt.matrix());
        }
    }

    Eigen::VectorXd qlf, qrt;
    ci_lf->getModel()->getJointPosition(qlf);
    ci_rt->getModel()->getJointPosition(qrt);

    EXPECT_EQ(qlf, qrt);
}

TEST_F(TestRt, checkUsage)
{
    // nrt side
    ci_lf->setActivationState("arm1_8", ActivationState::Disabled);
    EXPECT_EQ(ci_lf->getActivationState("arm1_8"), ActivationState::Enabled);

    // rt side
    ci_lf->callAvailable(ci_rt.get());

    EXPECT_EQ(ci_rt->getActivationState("arm1_8"), ActivationState::Disabled);

    EXPECT_TRUE(ci_rt->setBaseLink("arm2_8", "torso_2"));

    ci_lf->pushState(ci_rt.get(), ci_rt->getModel().get());

    // nrt side
    ci_lf->updateState();
    EXPECT_EQ(ci_lf->getActivationState("arm1_8"), ActivationState::Disabled);
    EXPECT_EQ(ci_lf->getBaseLink("arm2_8"), "torso_2");
}

int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
