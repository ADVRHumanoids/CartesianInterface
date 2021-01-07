#ifndef CONTEXT_H
#define CONTEXT_H

#include <memory>
#include <stdexcept>
#include <cartesian_interface/Macro.h>
#include <XBotInterface/ModelInterface.h>

namespace XBot { namespace Cartesian {

class ProblemDescription;

class Parameters
{

public:

    CARTESIO_DECLARE_SMART_PTR(Parameters)

    Parameters(double dt,
               std::string log_path = "/tmp");

    double getControlPeriod() const;
    std::string getLogPath() const;

    bool isLogEnabled() const;
    void setLogEnabled(bool flag);

private:

    double _dt;
    std::string _log_path;
    bool _enable_log;

};

class Context
{

public:

    CARTESIO_DECLARE_SMART_PTR(Context)

    Context(Parameters::Ptr params,
            ModelInterface::Ptr model);

    Context(Parameters::Ptr params);

    Parameters::Ptr params();
    Parameters::ConstPtr params() const;

    ModelInterface::Ptr model();
    ModelInterface::ConstPtr model() const;


private:

    Parameters::Ptr _params;
    ModelInterface::Ptr _model;

};


} }


#endif // CONTEXT_H
