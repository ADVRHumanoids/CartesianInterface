#include <algorithm>
#include <gtest/gtest.h>
#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <cartesian_interface/Context.h>

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

class TestApi: public ::testing::Test {


protected:

     TestApi()
     {

         std::cout << __PRETTY_FUNCTION__ << std::endl;

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

         auto ctx = std::make_shared<Context>(
                     std::make_shared<Parameters>(.01),
                     model);

         ProblemDescription ik_problem(ik_yaml, ctx);

         ci = std::make_shared<CartesianInterfaceImpl>(ik_problem, ctx);

     }

     virtual ~TestApi()
     {
         std::cout << __PRETTY_FUNCTION__ << std::endl;
     }

     virtual void SetUp() {

     }

     virtual void TearDown() {
     }

     CartesianInterfaceImpl::Ptr ci;

};

TEST_F(TestApi, checkParsing)
{
    auto list = ci->getTaskList();

    std::copy(list.begin(), list.end(), std::ostream_iterator<std::string>(std::cout, " "));

    ASSERT_FALSE(std::find(list.begin(), list.end(), "wheel_1") == list.end());

    ASSERT_FALSE(std::find(list.begin(), list.end(), "wheel_2") == list.end());

    ASSERT_FALSE(std::find(list.begin(), list.end(), "wheel_3") == list.end());

    ASSERT_FALSE(std::find(list.begin(), list.end(), "wheel_4") == list.end());

    ASSERT_FALSE(std::find(list.begin(), list.end(), "arm1_8") == list.end());

    ASSERT_FALSE(std::find(list.begin(), list.end(), "arm2_8") == list.end());

    ASSERT_FALSE(std::find(list.begin(), list.end(), "com") == list.end());

    ASSERT_FALSE(std::find(list.begin(), list.end(), "Postural") == list.end());

    ASSERT_TRUE( ci->getBaseLink("wheel_1") == "world" );

    ASSERT_TRUE( ci->getBaseLink("wheel_2") == "world" );

    ASSERT_TRUE( ci->getBaseLink("wheel_3") == "world" );

    ASSERT_TRUE( ci->getBaseLink("wheel_4") == "world" );

    ASSERT_TRUE( ci->getBaseLink("arm1_8") == "pelvis" );

    ASSERT_TRUE( ci->getBaseLink("arm2_8") == "world" );

    ASSERT_TRUE( ci->getBaseLink("com") == "world" );

    EXPECT_THROW( ci->getBaseLink("undefined_link"), std::exception );

    Eigen::VectorXd qref;
    ASSERT_TRUE(ci->getReferencePosture(qref));
    Eigen::VectorXd qhome;
    ci->getModel()->getRobotState("home", qhome);
    ASSERT_TRUE( (qref - qhome).norm() == 0.0 );

};


TEST_F(TestApi, checkTaskProperties)
{
    ASSERT_TRUE(ci->setBaseLink("arm2_8", "pelvis"));
    ASSERT_TRUE(ci->getBaseLink("arm2_8") == "pelvis");
    ASSERT_FALSE(ci->setBaseLink("arm2_8", "undefined_link"));
    ASSERT_TRUE(ci->getBaseLink("arm2_8") == "pelvis");
    ASSERT_TRUE(ci->setBaseLink("arm2_8", "world"));
    ASSERT_TRUE(ci->getBaseLink("arm2_8") == "world");

    for(auto task : ci->getTaskList())
    {
        ASSERT_EQ(ci->getTaskState(task), State::Online);
    }

    for(auto task : ci->getTaskList())
    {
        ASSERT_EQ(ci->getActivationState(task), ActivationState::Enabled);
    }

    for(auto task : ci->getTaskList())
    {
        ASSERT_TRUE(ci->setActivationState(task, ActivationState::Disabled));
        ASSERT_EQ(ci->getActivationState(task), ActivationState::Disabled);
        ASSERT_TRUE(ci->setActivationState(task, ActivationState::Enabled));
        ASSERT_EQ(ci->getActivationState(task), ActivationState::Enabled);

        if(TaskDescription::HasType<CartesianTask>(ci->getTask(task)))
        {
            ASSERT_TRUE(ci->setControlMode(task, ControlType::Velocity));
            ASSERT_EQ(ci->getControlMode(task), ControlType::Velocity);
            ASSERT_TRUE(ci->setControlMode(task, ControlType::Position));
            ASSERT_EQ(ci->getControlMode(task), ControlType::Position);
        }

    }


}


TEST_F(TestApi, checkReferences)
{
    std::string ee = "arm1_8";

    {
        auto T = GetRandomFrame();
        Eigen::Vector6d vel, acc;
        vel.setRandom(); acc.setRandom();
        ASSERT_TRUE(ci->setPoseReference(ee, T));
        ASSERT_TRUE(ci->setVelocityReference(ee, vel));
        Eigen::Affine3d T1;
        Eigen::Vector6d vel1, acc1;
        ASSERT_TRUE(ci->getPoseReference(ee, T1, &vel1, &acc1));
        EXPECT_TRUE( T.isApprox(T1) );
        EXPECT_TRUE( vel.isApprox(vel1) );
    }

    {
        auto T = GetRandomFrame();
        Eigen::Vector6d vel, acc;
        vel.setRandom(); acc.setRandom();
        ASSERT_TRUE(ci->setPoseReferenceRaw(ee, T));
        Eigen::Affine3d T1;
        Eigen::Vector6d vel1, acc1;
        ASSERT_TRUE(ci->getPoseReferenceRaw(ee, T1, &vel1, &acc1));
        EXPECT_TRUE( T.isApprox(T1) );
    }

    Eigen::Vector3d pcom, vcom, acom;
    pcom.setRandom();
    vcom.setRandom();
    acom.setRandom();
    ASSERT_TRUE(ci->setComPositionReference(pcom));
    ASSERT_TRUE(ci->setComVelocityReference(vcom));
    Eigen::Vector3d pcom1, vcom1, acom1;
    ASSERT_TRUE(ci->getComPositionReference(pcom1, &vcom1, &acom1));
    EXPECT_TRUE( pcom.isApprox(pcom1) );
    EXPECT_TRUE( vcom.isApprox(vcom1) );

    XBot::JointNameMap posture_ref;
    int i = 0;
    for(auto j : ci->getModel()->getEnabledJointNames())
    {
        if(i%3 == 0)
        {
            posture_ref[j] = i;
        }
    }
    ASSERT_TRUE(ci->setReferencePosture(posture_ref));
    XBot::JointNameMap posture_ref1;
    ASSERT_TRUE(ci->getReferencePosture(posture_ref1));
    ASSERT_TRUE(posture_ref == posture_ref1);


}


TEST_F(TestApi, checkTrajectories)
{
    ci->reset(0.0);
    std::string ee = "arm1_8";
    auto T = GetRandomFrame();
    ASSERT_TRUE(ci->setTargetPose(ee, T, 2.0));
    Eigen::Affine3d T2;
    ci->getPoseTarget(ee, T2);
    ASSERT_TRUE( T.isApprox(T2) );

    const double dt = 0.01;
    for(double t = 0; t < 2.5; t += dt)
    {
        ASSERT_TRUE(ci->update(t, dt));

        if(t < 2.0)
        {
            ASSERT_EQ(ci->getTaskState(ee), State::Reaching);
        }
        else if(t > 2.1)
        {
            ASSERT_EQ(ci->getTaskState(ee), State::Online);
            Eigen::Affine3d T1;
            ci->getPoseReference(ee, T1);
            EXPECT_TRUE( T.isApprox(T1) );
        }
    }


    T = GetRandomFrame();
    ASSERT_TRUE(ci->setTargetPose(ee, T, 3.0));
    ci->getPoseTarget(ee, T2);
    ASSERT_TRUE( T.isApprox(T2) );

    for(double t = 2.5; t < 12.0; t += dt)
    {
        ASSERT_TRUE(ci->update(t, dt));

        if(t < (2.5 + 3.0))
        {
            EXPECT_EQ(ci->getTaskState(ee), State::Reaching);
        }
        else if(t > (2.5 + 3.0))
        {
            EXPECT_EQ(ci->getTaskState(ee), State::Online);
            Eigen::Affine3d T1;
            ci->getPoseReference(ee, T1);
            EXPECT_TRUE( T.isApprox(T1) );
        }
    }

}


TEST_F(TestApi, checkWaypoints)
{
    ci->reset(0.0);
    std::string ee = "arm1_8";
    Trajectory::WayPointVector wpvec;
    wpvec.emplace_back(GetRandomFrame(), 1.0);
    wpvec.emplace_back(GetRandomFrame(), 1.5);
    wpvec.emplace_back(GetRandomFrame(), 3.0);
    wpvec.emplace_back(GetRandomFrame(), 4.0);

    ci->setWayPoints(ee, wpvec);

    Eigen::Affine3d T2;
    ci->getPoseTarget(ee, T2);
    ASSERT_TRUE( wpvec.back().frame.isApprox(T2) );

    const double dt = 0.01;
    for(double t = 0; t < 5.5; t += dt)
    {
        ASSERT_TRUE(ci->update(t, dt));

        if(t < 4.0)
        {
            ASSERT_EQ(ci->getTaskState(ee), State::Reaching);
        }
        else
        {
            ASSERT_EQ(ci->getTaskState(ee), State::Online);
            Eigen::Affine3d T1;
            ci->getPoseReference(ee, T1);
            EXPECT_TRUE( T1.isApprox(wpvec.back().frame) );
        }
    }



    Trajectory::WayPointVector wpvec1;
    wpvec1.emplace_back(GetRandomFrame(), 1.0);
    wpvec1.emplace_back(GetRandomFrame(), 1.5);
    wpvec1.emplace_back(GetRandomFrame(), 3.0);
    wpvec1.emplace_back(GetRandomFrame(), 4.0);

    ci->setWayPoints(ee, wpvec1);
    ci->getPoseTarget(ee, T2);
    ASSERT_TRUE( wpvec1.back().frame.isApprox(T2) );

    for(double t = 5.5; t < 12.0; t += dt)
    {
        ASSERT_TRUE(ci->update(t, dt));

        if(t < 9.5)
        {
            ASSERT_EQ(ci->getTaskState(ee), State::Reaching);
            Eigen::Affine3d T1;
            ci->getPoseReference(ee, T1);
            if(std::fabs(t-6.5) < 0.015)
            {
                EXPECT_TRUE( T1.isApprox(wpvec1[0].frame, 0.001) );
            }
            ASSERT_EQ(ci->getTaskState(ee), State::Reaching);
            if(std::fabs(t-7.0) < 0.015)
            {
                EXPECT_TRUE( T1.isApprox(wpvec1[1].frame, 0.001) );
            }
            ASSERT_EQ(ci->getTaskState(ee), State::Reaching);
            if(std::fabs(t-8.5) < 0.015)
            {
                EXPECT_TRUE( T1.isApprox(wpvec1[2].frame, 0.001) );
            }
            ASSERT_EQ(ci->getTaskState(ee), State::Reaching);
            if(std::fabs(t-9.5) < 0.015)
            {
                EXPECT_TRUE( T1.isApprox(wpvec1[3].frame, 0.001) );
            }
        }
        else
        {
            ASSERT_EQ(ci->getTaskState(ee), State::Online);
            Eigen::Affine3d T1;
            ci->getPoseReference(ee, T1);
            EXPECT_TRUE( T1.isApprox(wpvec1.back().frame) );
        }
    }
    
}


TEST_F(TestApi, checkLimits)
{
    const double dt = ci->getContext()->params()->getControlPeriod();
    
    
    auto logger = XBot::MatLogger::getLogger("/tmp/checkLimits_logger");
    std::string ee = "arm1_8";
    ci->enableOtg(dt);
    ci->reset(0.0);
   
    double vmax_lin, vmax_ang, amax_lin, amax_ang;
    ci->getVelocityLimits(ee, vmax_lin, vmax_ang);
    ci->getAccelerationLimits(ee, amax_lin, amax_ang);
    std::cout << ee << " " << vmax_lin << "  " << vmax_ang << std::endl;
    std::cout << ee << " " << amax_lin << "  " << amax_ang << std::endl;
    
    
    Eigen::Affine3d Told, Tref;
    ci->getPoseReference(ee, Told);
    Eigen::Vector3d v_lin_old(0,0,0), v_ang_old(0,0,0);
    for(double t = 0.0; t < 15.0; t += dt)
    {
        
        if(t < 5.0)
        {
            Tref = GetRandomFrame();
            Tref.translation() *= 10.0;
        }
        
        
        ci->setPoseReference(ee, Tref);
        ci->update(t, dt);
        Eigen::Affine3d Tnew;
        ci->getPoseReference(ee, Tnew);
        
        std::cout << "V_LIN ***********" << std::endl;
        
        Eigen::Vector3d v_lin = (Tnew.translation() - Told.translation())/dt;
        std::cout << v_lin.transpose() << std::endl;
        std::cout << v_lin.norm() << std::endl;
        const double EPS_V_LIN = 0.001;
        EXPECT_LE( std::fabs(v_lin.x()), vmax_lin + EPS_V_LIN );
        EXPECT_LE( std::fabs(v_lin.y()), vmax_lin + EPS_V_LIN );
        EXPECT_LE( std::fabs(v_lin.z()), vmax_lin + EPS_V_LIN );
        
        std::cout << "V_ANG ***********" << std::endl;
        
        Eigen::Matrix3d R12 = (Tnew.linear().transpose()*Told.linear()); 
        Eigen::Matrix3d S = (Tnew.linear() - Told.linear()) * Tnew.linear().transpose() / dt; 
        Eigen::Vector3d v_ang(S(2,1), S(0,2), S(1,0));
        double delta_theta = std::acos(0.5*(R12.trace() - 1));
        std::cout << "S\n " << S << std::endl;
        std::cout << "dtheta " << delta_theta / dt << std::endl;
        std::cout << "v_ang " << v_ang.transpose() << " --- " << v_ang.norm() <<  std::endl;
        const double EPS_V_ANG = 0.001;
//         EXPECT_TRUE( std::fabs(delta_theta) <= vmax_ang + EPS_V_ANG );
        
        std::cout << "A_LIN ***********" << std::endl;
        
        Eigen::Vector3d acc_lin = (v_lin - v_lin_old)/dt;
        std::cout << acc_lin.transpose() << std::endl;
        std::cout << acc_lin.norm() << std::endl;
        const double EPS_A_LIN = 0.001;
        EXPECT_LE( std::fabs(acc_lin.x()), amax_lin + EPS_A_LIN );
        EXPECT_LE( std::fabs(acc_lin.y()), amax_lin + EPS_A_LIN );
        EXPECT_LE( std::fabs(acc_lin.z()), amax_lin + EPS_A_LIN );
        
        
        std::cout << "A_ANG ***********" << std::endl;
        Eigen::Vector3d acc_ang = (v_ang - v_ang_old)/dt;
        std::cout << acc_ang.transpose() << std::endl;
        std::cout << acc_ang.norm() << std::endl;
        const double EPS_A_ANG = 0.001;
//         EXPECT_TRUE( std::fabs(acc_ang.x()) <= amax_ang + EPS_A_ANG );
//         EXPECT_TRUE( std::fabs(acc_ang.y()) <= amax_ang + EPS_A_ANG );
//         EXPECT_TRUE( std::fabs(acc_ang.z()) <= amax_ang + EPS_A_ANG );
        
        Told = Tnew;
        v_lin_old = v_lin;
        v_ang_old = v_ang;
        
        logger->add("v_lin", v_lin  );
        logger->add("a_lin", acc_lin);
        logger->add("v_ang", v_ang  );
        logger->add("a_ang", acc_ang);
        logger->add("v_lin_norm", v_lin  .norm());
        logger->add("a_lin_norm", acc_lin.norm());
        logger->add("v_ang_norm", v_ang  .norm());
        logger->add("a_ang_norm", acc_ang.norm());
        
    }
    
    logger->flush();
}

   

}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
