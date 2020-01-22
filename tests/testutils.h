#ifndef TESTUTILS_H
#define TESTUTILS_H

#include <XBotInterface/ModelInterface.h>

#include <unistd.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/wait.h>


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

class Process
{

public:

    Process(std::vector<std::string> args);

    int wait();

    void kill(int signal = SIGTERM);

    ~Process();

private:

    std::string _name;
    pid_t _pid;

};

Process::Process(std::vector<std::string>  args):
    _name(args[0])
{
    std::vector<const char *> args_cstr;
    for(auto& a : args) args_cstr.push_back(a.c_str());
    args_cstr.push_back(nullptr);

    char ** argv = (char**)args_cstr.data();

    _pid = ::fork();

    if(_pid == -1)
    {
        perror("fork");
        throw std::runtime_error("Unable to fork()");
    }

    if(_pid == 0)
    {
        ::execvp(argv[0], argv);
        perror("execvp");
        throw std::runtime_error("Unknown command");
    }

}

int Process::wait()
{
    int status;
    while(::waitpid(_pid, &status, 0) != _pid);
    printf("Child process '%s' exited with status %d\n", _name.c_str(), status);
    return status;

}

void Process::kill(int signal)
{
    ::kill(_pid, signal);
    printf("Killed process '%s' with signal %d\n", _name.c_str(), signal);
}

Process::~Process()
{
    kill(SIGINT);
    wait();
}

#endif // TESTUTILS_H
