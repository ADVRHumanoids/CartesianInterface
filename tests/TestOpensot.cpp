#include <gtest/gtest.h>
#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <algorithm>

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
         
         YAML::Node ik_yaml = YAML::LoadFile(path_to_cfg + "centauro_test_stack.yaml");
         
         XBot::Cartesian::ProblemDescription ik_problem(ik_yaml, model);
         
         std::string impl_name = "OpenSot";
         std::string path_to_shared_lib = XBot::Utils::FindLib("libCartesian" + impl_name + ".so", "LD_LIBRARY_PATH");
         if (path_to_shared_lib == "")
         {
             throw std::runtime_error("libCartesian" + impl_name + ".so must be listed inside LD_LIBRARY_PATH");
         } 
        
         ci = SoLib::getFactoryWithArgs<XBot::Cartesian::CartesianInterfaceImpl>(path_to_shared_lib, 
                                                                                impl_name + "Impl", 
                                                                                model, ik_problem);
         
     }

     virtual ~TestOpensot() {
     }

     virtual void SetUp() {
         
     }

     virtual void TearDown() {
     }
     
     XBot::Cartesian::CartesianInterfaceImpl::Ptr ci;
     
};

TEST_F(TestOpensot, checkParsing)
{
    
};


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}