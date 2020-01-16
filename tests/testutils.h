#ifndef TESTUTILS_H
#define TESTUTILS_H

#include <XBotInterface/ModelInterface.h>

inline XBot::ModelInterface::Ptr GetTestModel()
{

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

    return model;

}

#endif // TESTUTILS_H
