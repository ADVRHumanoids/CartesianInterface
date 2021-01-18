#include "OpenSotImpl.h"
#include <cartesian_interface/sdk/SolverPlugin.h>

#include <cartesian_interface/problem/ProblemDescription.h>
#include <cartesian_interface/problem/Task.h>
#include <boost/make_shared.hpp>
#include <OpenSoT/constraints/TaskToConstraint.h>
#include <OpenSoT/solvers/eHQP.h>
#include <OpenSoT/solvers/iHQP.h>
#include <OpenSoT/solvers/nHQP.h>
#include <OpenSoT/solvers/l1HQP.h>

#ifdef _GLPK_FOUND
    #define GLPK_FOUND true
    #include <OpenSoT/solvers/GLPKBackEnd.h>
#else
    #define GLPK_FOUND false
#endif

#include "utils/DynamicLoading.h"

#include "opensot/OpenSotTask.h"

#include "fmt/format.h"

using namespace XBot::Cartesian;

CARTESIO_REGISTER_SOLVER_PLUGIN(OpenSotImpl, OpenSot)

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
    else if(back_end_string == "glpk")
    {
        return OpenSoT::solvers::solver_back_ends::GLPK;
    }
    else
    {
        throw std::runtime_error("Invalid back end '" + back_end_string + "'");
    }
}

OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>::SolverPtr frontend_from_string(std::string front_end_string,
                                                                                  OpenSoT::AutoStack& as,
                                                                                  const double eps_regularisation,
                                                                                  const OpenSoT::solvers::solver_back_ends be_solver,
                                                                                  YAML::Node options)
{
    if(front_end_string == "ihqp")
    {
        return boost::make_shared<OpenSoT::solvers::iHQP>(as,
                                                          eps_regularisation,
                                                          be_solver);
    }
    else if(front_end_string == "ehqp")
    {
        return boost::make_shared<OpenSoT::solvers::eHQP>(as.getStack());
    }
    else if(front_end_string == "l1hqp")
    {
        OpenSoT::solvers::l1HQP::Ptr l1hqp_solver =  boost::make_shared<OpenSoT::solvers::l1HQP>(as,
                                                           eps_regularisation,
                                                           be_solver);


        if(be_solver == OpenSoT::solvers::solver_back_ends::GLPK)
        {
#if GLPK_FOUND
            OpenSoT::solvers::BackEnd::Ptr GLPK;
            l1hqp_solver->getBackEnd(GLPK);

            if(options && options["MILP"])
            {
                OpenSoT::solvers::GLPKBackEnd::GLPKBackEndOptions opt;
                for(unsigned int i = l1hqp_solver->getFirstSlackIndex(); i < GLPK->getNumVariables(); ++i)
                    opt.var_id_kind_.push_back(std::pair<int, int>(i, GLP_IV));

                GLPK->setOptions(opt);
            }

        }

        return std::move(l1hqp_solver);
#else
            throw std::runtime_error("Solver Back-End GLPK can not be requested because GLPK is not found!");
#endif

    }
    else if(front_end_string == "nhqp")
    {
        auto frontend = boost::make_shared<OpenSoT::solvers::nHQP>(as.getStack(),
                                                                   as.getBounds(),
                                                                   //as.getRegularisationTask(),
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

        return std::move(frontend);
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
        auto it = *std::find_if(_task_adapters.begin(), _task_adapters.end(),
                                [task_desc](auto t){ return t->getTaskDescription() == task_desc; });
        
        tasks_list.push_back(it->getOpenSotTask());
    }

    /* Return Aggregated */
    if(tasks_list.size() > 1)
    {
        return boost::make_shared<OpenSoT::tasks::Aggregated>(tasks_list, _x.size());
    }
    else if(tasks_list.empty())
    {
        throw std::runtime_error("Empty stack found");
    }
    else
    {
        return tasks_list.front();
    }
}

void OpenSotImpl::make_task_adapter(TaskDescription::Ptr task_desc)
{
    try
    {
        auto adapter = OpenSotTaskAdapter::MakeInstance(task_desc, getContext());
        _task_adapters.push_back(adapter);
    }
    catch(std::runtime_error& e)
    {
        Logger::error("[open_sot] construct_task failed with error '%s' \n", e.what());
    }
}



void OpenSotImpl::make_constraint_adapter(ConstraintDescription::Ptr constr_desc)
{
    try
    {
        auto adapter = OpenSotConstraintAdapter::MakeInstance(constr_desc, getContext());
        _constr_adapters.push_back(adapter);
    }
    catch(std::runtime_error& e)
    {
        Logger::error("[open_sot] constraint_from_description failed with error '%s' \n", e.what());
    }
}


OpenSotImpl::OpenSotImpl(ProblemDescription ik_problem,
                         Context::Ptr context):
    CartesianInterfaceImpl(ik_problem, context),
    _vars({}),
    _force_space_references(false)
{
    if(this->getContext()->params()->isLogEnabled())
    {
        /* Create logger */
        MatLogger2::Options logger_opt;
        logger_opt.default_buffer_size = 1e5;
        _logger = MatLogger2::MakeLogger(context->params()->getLogPath() + "/cartesio_opensot_log_" + std::to_string(rand()),
                                         logger_opt);
        _logger->set_buffer_mode(VariableBuffer::Mode::circular_buffer);
    }

    _model->getJointPosition(_q);
    _dq.setZero(_q.size());
    _ddq = _dq;
    _tau.setZero(_dq.size());
    _J.setZero(6, _dq.size());

    /* Make adapters for all tasks and constraints */
    for(int i = 0; i < ik_problem.getNumTasks(); i++)
    {
        for(auto task : ik_problem.getTask(i))
        {
            make_task_adapter(task);
        }
    }

    for(auto task : ik_problem.getRegularizationTask())
    {
        make_task_adapter(task);
    }

    for(auto constr : ik_problem.getBounds())
    {
        make_constraint_adapter(constr);
    }

    /* Get required variable list */
    std::map<std::string, int> vars_map;

    // lambda to process one task or constr
    auto parse_task = [&vars_map](auto t)
    {
        for(auto v : t->getRequiredVariables())
        {
            if(vars_map.count(v.first) > 0 &&
                    vars_map.at(v.first) != v.second)
            {
                throw BadVariables(fmt::format("conflicting size for var '{}': {} vs {}",
                                               v.first, v.second, vars_map.at(v.first)));
            }

            fmt::print("[OpenSot] found variable '{}', size = {} \n",
                       v.first, v.second);

            vars_map[v.first] = v.second;

        }
    };

    // apply lambda to tasks and constraints
    for(auto t : _task_adapters)
    {
        parse_task(t);
    }
    for(auto c : _constr_adapters)
    {
        parse_task(c);
    }

    // initialize solution map
    for(auto v : vars_map)
    {
        _solution[v.first].setZero(v.second);
    }

    // copy to vector and create optvar helper
    OpenSoT::OptvarHelper::VariableVector vars;
    std::copy(vars_map.begin(), vars_map.end(), std::back_inserter(vars));
    _vars = OpenSoT::OptvarHelper(vars);
    _x.setZero(_vars.getSize());
    if(_x.size() == 0)
    {
        _x.setZero(_q.size());
        _solution["qdot"].setZero(_x.size());
    }

    // add full solution
    _solution["full_solution"].setZero(_x.size());

    // fill variable map
    for(auto p : vars_map)
    {
        _vars_map[p.first] = _vars.getVariable(p.first);
    }

    /* Initialize all tasks and constraints (TBD error checking) */
    for(auto t : _task_adapters)
    {
        t->initialize(_vars);
    }
    for(auto c : _constr_adapters)
    {
        c->initialize(_vars);
    }


    /* Parse stack #0 and create autostack */
    auto stack_0 = aggregated_from_stack(ik_problem.getTask(static_cast<int>(0)));
    _autostack = boost::make_shared<OpenSoT::AutoStack>(stack_0);

    /* Parse remaining stacks  */
    for(int i = 1; i < ik_problem.getNumTasks(); i++)
    {
        _autostack = _autostack / aggregated_from_stack(ik_problem.getTask(i));
    }

    /* Parse constraints */
    for(auto constr : ik_problem.getBounds())
    {
        auto constr_adapter = *std::find_if(_constr_adapters.begin(), _constr_adapters.end(),
                                         [constr](auto c){ return c->getConstraintDescription() == constr; });

        auto constr_ptr = constr_adapter->getOpenSotConstraint();

        if(constr_ptr)
        {
            _autostack << constr_ptr;
        }
    }

    /* Set regularization task */
    if(ik_problem.getRegularizationTask().size() > 0)
    {
        Logger::info(Logger::Severity::HIGH, "OpenSot: regularization task found \n");
        auto reg_task = aggregated_from_stack(ik_problem.getRegularizationTask());
        _autostack->setRegularisationTask(reg_task);
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

    if(has_config() && get_config()["force_space_references"])
    {
        _force_space_references = get_config()["force_space_references"].as<bool>();
    }
    
    Logger::info(Logger::Severity::HIGH, "OpenSot: regularization value is %.1e\n", eps_regularization);

    /* Create solver */
    _solver = ::frontend_from_string(front_end_string,
                                     *_autostack,
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
    _model->getJointVelocity(_dq);
    if(_vars.getSize() == 0)
    {
        _x = _q;
    }
    
    /* Update all plugin-based tasks and constraints */
    for(auto t : _task_adapters)
    {
        t->update(time, period);
    }

    for(auto c : _constr_adapters)
    {
        c->update(time, period);
    }

    /* Update tasks and solve */
    _autostack->update(_x);

    if(_logger)
    {
        _autostack->log(_logger);
    }

    if(!_solver->solve(_x))
    {
        _x.setZero(_x.size());
        XBot::Logger::error("OpenSot: unable to solve\n");
        success = false;
    }

    if(_logger)
    {
        _solver->log(_logger);
    }

    _solution.at("full_solution") = _x;

    /* Allow tasks and constraints to read solution */
    for(auto t : _task_adapters)
    {
        t->processSolution(_x);
    }

    for(auto c : _constr_adapters)
    {
        c->processSolution(_x);
    }

    /* Set solution to model */
    _tau.setZero(_dq.size());

    if(_vars.getSize() == 0)
    {
        _dq = _x;
        _dq /= period;
        _model->setJointVelocity(_dq);
        _solution.at("qdot") = _dq;
    }

    if(_vars_map.count("qddot"))
    {
        _vars_map.at("qddot").getValue(_x, _ddq);
        _model->setJointAcceleration(_ddq);
        _solution.at("qddot") = _ddq;
    }

    _model->update();
    _model->computeInverseDynamics(_tau);

    for(auto& p : _vars_map)
    {
        if(p.first.find("force_") != 0)
        {
            continue;
        }

        auto link_name = p.first.substr(6); // removes "force_"

        _model->getJacobian(link_name, _J);

        Eigen::Vector6d f_value;
        p.second.getValue(_x, f_value);

        _tau.noalias() -= _J.transpose() * f_value;

        if(_logger)
        {
            _logger->add(p.first, f_value);
        }

        _solution.at(p.first) = f_value;

    }

    _model->setJointEffort(_tau);

    if(_logger)
    {
        _logger->add("q", _q);
        _logger->add("dq", _dq);
        _logger->add("ddq", _ddq);
        _logger->add("tau", _tau);
    }

    return success;

}


OpenSotImpl::~OpenSotImpl()
{

}


