#include "OpenSotImpl.h"

#include <cartesian_interface/problem/ProblemDescription.h>
#include <cartesian_interface/problem/Task.h>
#include <boost/make_shared.hpp>
#include <OpenSoT/constraints/TaskToConstraint.h>
#include <OpenSoT/solvers/eHQP.h>
#include <OpenSoT/solvers/iHQP.h>
#include <OpenSoT/solvers/nHQP.h>

#include "utils/DynamicLoading.h"

#include "opensot/OpenSotTask.h"

using namespace XBot::Cartesian;


extern "C" CartesianInterface* create_instance(XBot::ModelInterface::Ptr model,
                                               ProblemDescription pb)
{
    return new OpenSotImpl(model, pb);
}

namespace
{
    OpenSoT::solvers::solver_back_ends backend_from_string(std::string back_end_string)
    {
        if(back_end_string == "qpoases")
        {
            return OpenSoT::solvers::solver_back_ends::qpOASES;
        }
        else if(back_end_string == "osqp")
        {
            return OpenSoT::solvers::solver_back_ends::OSQP;
        }
        else if(back_end_string == "eiquadprog")
        {
            return OpenSoT::solvers::solver_back_ends::eiQuadProg;
        }
        else if(back_end_string == "odys")
        {
            return OpenSoT::solvers::solver_back_ends::ODYS;
        }
        else
        {
            throw std::runtime_error("Invalid back end '" + back_end_string + "'");
        }
    }

    OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>::SolverPtr frontend_from_string(std::string front_end_string,
                                                                                      OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>::Stack& stack_of_tasks,
                                                                                      OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>::ConstraintPtr bounds,
                                                                                      const double eps_regularisation,
                                                                                      const OpenSoT::solvers::solver_back_ends be_solver,
                                                                                      YAML::Node options)
    {
        if(front_end_string == "ihqp")
        {
            return boost::make_shared<OpenSoT::solvers::iHQP>(stack_of_tasks,
                                                              bounds,
                                                              eps_regularisation,
                                                              be_solver);
        }
        else if(front_end_string == "ehqp")
        {
            return boost::make_shared<OpenSoT::solvers::eHQP>(stack_of_tasks);
        }
        else if(front_end_string == "nhqp")
        {
            auto frontend = boost::make_shared<OpenSoT::solvers::nHQP>(stack_of_tasks,
                                                                       bounds,
                                                                       eps_regularisation,
                                                                       be_solver);
            if(options && options["nhqp_min_sv_ratio"])
            {
                if(options["nhqp_min_sv_ratio"].IsScalar())
                {
                    frontend->setMinSingularValueRatio(options["nhqp_min_sv_ratio"].as<double>());
                }

                if(options["nhqp_min_sv_ratio"].IsSequence())
                {
                    frontend->setMinSingularValueRatio(options["nhqp_min_sv_ratio"].as<std::vector<double>>());
                }
            }

            return frontend;
        }
        else
        {
            throw std::runtime_error("Invalid front end '" + front_end_string + "'");
        }
    }
}


OpenSoT::tasks::Aggregated::TaskPtr OpenSotImpl::aggregated_from_stack(AggregatedTask stack)
{
    std::list<OpenSoT::tasks::Aggregated::TaskPtr> tasks_list;

    for(TaskDescription::Ptr task_desc : stack)
    {
        auto task = construct_task(task_desc);
        
        if(task)
        {
            tasks_list.push_back(task);
        }
        
    }

    /* Return Aggregated */
    if(tasks_list.size() > 1)
    {
        return boost::make_shared<OpenSoT::tasks::Aggregated>(tasks_list, _q.size());
    }
    else
    {
        return tasks_list.front();
    }
}

OpenSotImpl::TaskPtr OpenSotImpl::construct_task(TaskDescription::Ptr task_desc)
{
    OpenSotImpl::TaskPtr opensot_task;

    try
    {
        auto adapter = OpenSotTaskAdapter::MakeInstance(task_desc, _model);
        _task_adapters.push_back(adapter);

        opensot_task = adapter->getOpenSotTask();
    }
    catch(std::runtime_error& e)
    {
        Logger::error("[open_sot] construct_task failed with error '%s' \n", e.what());
    }

    return opensot_task;

}



OpenSoT::Constraint< Eigen::MatrixXd, Eigen::VectorXd >::ConstraintPtr OpenSotImpl::constraint_from_description(ConstraintDescription::Ptr constr_desc)
{
    OpenSotImpl::ConstraintPtr opensot_constraint;

    try
    {
        auto adapter = OpenSotConstraintAdapter::MakeInstance(constr_desc, _model);
        _constr_adapters.push_back(adapter);

        opensot_constraint = adapter->getOpenSotConstraint();
    }
    catch(std::runtime_error& e)
    {
        Logger::error("[open_sot] constraint_from_description failed with error '%s' \n", e.what());
    }

    return opensot_constraint;

}


OpenSotImpl::OpenSotImpl(XBot::ModelInterface::Ptr model,
                         ProblemDescription ik_problem):
    CartesianInterfaceImpl(model, ik_problem),
    _logger(XBot::MatLogger::getLogger("/tmp/xbot_cartesian_opensot_log"))
{
    _model->getJointPosition(_q);
    _dq.setZero(_q.size());
    _ddq = _dq;

    /* Parse stack #0 and create autostack */
    auto stack_0 = aggregated_from_stack(ik_problem.getTask(0));
    _autostack = boost::make_shared<OpenSoT::AutoStack>(stack_0);

    /* Parse remaining stacks  */
    for(int i = 1; i < ik_problem.getNumTasks(); i++)
    {
        _autostack = _autostack / aggregated_from_stack(ik_problem.getTask(i));
    }

    /* Parse constraints */
    for(auto constr : ik_problem.getBounds())
    {
        auto constr_ptr = constraint_from_description(constr);

        if(constr_ptr)
        {
            _autostack << constr_ptr;
        }
    }
    
    /* Parse solver configs (if any) */
    using BackEnd = OpenSoT::solvers::solver_back_ends;
    double eps_regularization = 1e6;
    BackEnd solver_backend = BackEnd::qpOASES;
    std::string front_end_string = "ihqp";

    if(has_config() && get_config()["front_end"])
    {
        front_end_string = get_config()["front_end"].as<std::string>();
        Logger::info(Logger::Severity::HIGH, "OpenSot: using front-end '%s'\n", front_end_string.c_str());
    }
    
    if(has_config() && get_config()["back_end"])
    {
        std::string back_end_string = get_config()["back_end"].as<std::string>();
        solver_backend = ::backend_from_string(back_end_string);
        Logger::info(Logger::Severity::HIGH, "OpenSot: using back-end '%s'\n", back_end_string.c_str());
    }
    
    if(has_config() && get_config()["regularization"])
    {
        eps_regularization = get_config()["regularization"].as<double>();
        eps_regularization *= 1e12;
    }
    
    Logger::info(Logger::Severity::HIGH, "OpenSot: regularization value is %.1e\n", eps_regularization);

    /* Create solver */
    _solver = ::frontend_from_string(front_end_string,
                                   _autostack->getStack(),
                                   _autostack->getBounds(),
                                   eps_regularization,
                                   solver_backend,
                                   get_config()
                                   );
    
}

bool OpenSotImpl::update(double time, double period)
{
    bool success = true;

    CartesianInterfaceImpl::update(time, period);

    _model->getJointPosition(_q);
    
    /* Update all plugin-based tasks and constraints */
    for(auto t : _task_adapters)
    {
        t->update(time, period);
    }

    for(auto c : _constr_adapters)
    {
        c->update(time, period);
    }

    /* Force estimation */
    if(_force_estimation)
    {
        _force_estimation->update();
        _force_estimation->log(_logger);
    }
    
    /* Update tasks and solve */
    _autostack->update(_q);
    _autostack->log(_logger);

    if(!_solver->solve(_dq))
    {
        _dq.setZero(_dq.size());
        XBot::Logger::error("OpenSot: unable to solve\n");
        success = false;
    }

    /* Set solution to model */
    _dq /= period;
    _model->setJointVelocity(_dq);
    _model->setJointAcceleration(_ddq);

    _logger->add("q", _q);

    return success;

}


OpenSotImpl::~OpenSotImpl()
{
    _logger->flush();
}


