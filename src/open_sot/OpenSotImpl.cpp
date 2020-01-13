#include <cartesian_interface/open_sot/OpenSotImpl.h>
#include <cartesian_interface/problem/ProblemDescription.h>
#include <cartesian_interface/problem/Task.h>
#include <cartesian_interface/problem/Interaction.h>
#include <cartesian_interface/problem/Com.h>
#include <cartesian_interface/problem/Postural.h>
#include <cartesian_interface/problem/Gaze.h>
#include <cartesian_interface/problem/Limits.h>
#include <cartesian_interface/utils/LoadObject.hpp>
#include <boost/make_shared.hpp>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <OpenSoT/constraints/velocity/VelocityLimits.h>
#include <OpenSoT/constraints/TaskToConstraint.h>
#include <OpenSoT/solvers/eHQP.h>
#include <OpenSoT/solvers/iHQP.h>
#include <OpenSoT/solvers/nHQP.h>

#include "task_adapters/OpenSotTask.h"

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


bool OpenSotImpl::setBaseLink(const std::string& ee_name, const std::string& new_base_link)
{
    if(!CartesianInterfaceImpl::setBaseLink(ee_name, new_base_link))
    {
        return false;
    }
    
    bool success = false;
    
    for(auto task : _cartesian_tasks)
    {
        if(task->getDistalLink() == ee_name)
        {
            success = task->setBaseLink(new_base_link);
        }
    }
    
    return success;
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
    catch(std::exception& e)
    {
        Logger::error("[open_sot] construct_task failed with error '%s' \n", e.what());
    }

    return opensot_task;

}



OpenSoT::Constraint< Eigen::MatrixXd, Eigen::VectorXd >::ConstraintPtr OpenSotImpl::constraint_from_description(ConstraintDescription::Ptr constr_desc)
{
    OpenSotImpl::ConstraintPtr opensot_constraint;

    if(constr_desc->type == "JointLimits")
    {
        Eigen::VectorXd qmin, qmax;
        _model->getJointLimits(qmin, qmax);

        auto joint_lims = boost::make_shared<OpenSoT::constraints::velocity::JointLimits>
                                            (_q,
                                                qmax,
                                                qmin
                                                );

        return joint_lims;


    }
    else if(constr_desc->type == "VelocityLimits")
    {
        Eigen::VectorXd qdotmax;
        _model->getVelocityLimits(qdotmax);
        
        double control_dt = 0.01;
        if(!get_control_dt(control_dt))
        {
            Logger::warning("Unable to find control period \
in configuration file (add problem_description/solver_options/control_dt field)\n");
        }
        
        auto vel_lims = boost::make_shared<OpenSoT::constraints::velocity::VelocityLimits>
                                            (qdotmax, control_dt);

        return vel_lims;

    }
    else if(constr_desc->type == "ConstraintFromTask")
    {
        auto task = GetTaskFromConstraint(constr_desc);
        if(task)
        {
            auto opensot_task = construct_task(task);
            
            if(opensot_task)
            {
                return boost::make_shared<OpenSoT::constraints::TaskToConstraint>(opensot_task);
            }
            else
            {
                XBot::Logger::warning("Unable to construct OpenSoT task (%s)\n", constr_desc->type.c_str());
                return nullptr;
            }
        }
        else
        {
            XBot::Logger::warning("Unable to construct task description (%s)\n", constr_desc->type.c_str());
            return nullptr;
        }
    }
    else if(!constr_desc->lib_name.empty())
    {
        auto constr_ifc = Utils::LoadObject<SoT::ConstraintInterface>(constr_desc->lib_name, 
                                          constr_desc->type + "OpenSotConstraintFactory",
                                          constr_desc, _model
                                         );
        
        if(constr_ifc)
        {
            opensot_constraint = constr_ifc->getConstraintPtr();
            _constr_ifc.emplace_back(std::move(constr_ifc));
            return opensot_constraint;
        }
        else
        {
            Logger::warning("OpenSot: unable to construct task type '%s'd\n", 
                constr_desc->type.c_str());
            
            return nullptr;
        }
    }
    else
    {
        XBot::Logger::warning("Unsupported constraint type (%s)\n", constr_desc->type.c_str());
        return nullptr;
    }
}


OpenSotImpl::OpenSotImpl(XBot::ModelInterface::Ptr model,
                         ProblemDescription ik_problem):
    CartesianInterfaceImpl(model, ik_problem),
    _logger(XBot::MatLogger::getLogger("/tmp/xbot_cartesian_opensot_log")),
    _update_lambda(false)
{
    _model->getJointPosition(_q);
    _dq.setZero(_q.size());
    _ddq = _dq;

    _B.setIdentity(_q.size(), _q.size());
    
    

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
    
    /* Update all plugin-based tasks */
    for(auto t : _task_ifc)
    {
        t->update(this, time, period);
    }

    /* Update all plugin-based constraints */
    for(auto c : _constr_ifc)
    {
        c->update(this, time, period);
    }

    /* Update reference for all cartesian tasks */
    for(auto cart_task : _cartesian_tasks)
    {
        Eigen::Affine3d T_ref;
        Eigen::Vector6d v_ref, a_ref;

        if(!getPoseReference(cart_task->getDistalLink(), T_ref, &v_ref, &a_ref))
        {
            continue;
        }
        
        v_ref *= period;
        cart_task->setReference(T_ref, v_ref);

    }
    
    /* Process admittance tasks */
    for(auto i_task : _admittance_tasks)
    {
        Eigen::Vector6d f;
        Eigen::Matrix6d k, d;

        if(!getDesiredInteraction(i_task->getDistalLink(), f, k, d))
        {
            continue;
        }
        
        if(i_task->getLambda() > 0)
        {
        
            i_task->setImpedanceParams(k.diagonal(),
                                       d.diagonal(), 
                                       i_task->getLambda(), 
                                       i_task->getFilterTimeStep()
                                      );
            
            i_task->setWrenchReference(f);
            
        }
        else
        {
            double dt = i_task->getFilterTimeStep();
            Eigen::Vector6d inertia = i_task->getInertia().diagonal();
            i_task->setRawParams(d.diagonal().cwiseInverse() * dt, 
                d.diagonal().cwiseProduct(inertia.cwiseInverse()),
                0.0, 
                dt
            );
            
            i_task->setWrenchReference(f);
        }

    }

    /* Handle GAZE reference */
    if(_gaze_task)
    {
        Eigen::Affine3d T_ref;
        if(getPoseReference("gaze", T_ref))
        {
            _gaze_task->setGaze(T_ref);
        }
    }
    
    /* Handle COM reference */
    if(_com_task)
    {
        Eigen::Vector3d x, v, a;
        
        if(getComPositionReference(x, &v, &a))
        {
            _com_task->setReference(x, v*period);
            
        }
    }


    /* Postural task */
    if(_postural_tasks.size() > 0 && getReferencePosture(_qref))
    {
        bool compute_inertia = true;
        int idx = 0;
        for(auto postural : _postural_tasks)
        {
            postural->setReference(_qref);
            if(_use_inertia_matrix[idx])
            {
                if(compute_inertia)
                {
                    _model->getInertiaMatrix(_B);
                    compute_inertia = false;
                }
                postural->setWeight(_B);
            }
            idx++;
        }
    }

    /* Lambda */
    if(_update_lambda)
    {
        for(auto t : _cartesian_tasks)
        {
            if( getControlMode(t->getDistalLink()) == ControlType::Position )
            {
                t->setLambda(_lambda_map.at(t->getDistalLink()));
            }
        }
        
        _update_lambda = false;
    }
    
    /* Force estimation */
    if(_force_estimation)
    {
        _force_estimation->update();
        _force_estimation->log(_logger);
    }
    
    _autostack->update(_q);
    _autostack->log(_logger);

    if(!_solver->solve(_dq))
    {
        _dq.setZero(_dq.size());
        XBot::Logger::error("OpenSot: unable to solve\n");
        success = false;
    }


    _dq /= period;
    _model->setJointVelocity(_dq);
    _model->setJointAcceleration(_ddq);

    _logger->add("q", _q);


    return success;


}

bool OpenSotImpl::setControlMode(const std::string& ee_name, 
                                 ControlType ctrl_type)
{
    if(!CartesianInterfaceImpl::setControlMode(ee_name, ctrl_type))
    {
        return false;
    }
    
    OpenSoT::tasks::Aggregated::TaskPtr task_ptr;
    
    if(ee_name == "com" && _com_task)
    {
        task_ptr = _com_task;
    }
    else if(ee_name == "gaze" && _gaze_task)
    {
        if(ctrl_type == ControlType::Velocity)
        {
            XBot::Logger::error("Gaze task does not allow velocity ctrl type! \n");
            return false;
        }
    }
    
    for(const auto t : _cartesian_tasks)
    {
        if(t->getDistalLink() == ee_name)
        {
            task_ptr = t;
        }
    }
    
    if(task_ptr)
    {
        
        switch(ctrl_type)
        {
            case ControlType::Disabled:
                task_ptr->setActive(false);
                break;
                
            case ControlType::Velocity:
                task_ptr->setActive(true);
                _lambda_map[ee_name] = task_ptr->getLambda();
                task_ptr->setLambda(0.0);
                break;
                
            case ControlType::Position:
                if( _lambda_map.find(ee_name) == _lambda_map.end() )
                {
                    XBot::Logger::error("No lambda value for task %s defined, contact the developers\n", ee_name.c_str());
                    return false;
                }
                task_ptr->setActive(true);
                task_ptr->setLambda(_lambda_map.at(ee_name));
                break;
                
            default:
                break;
            
        }
        
        return true;
    }
    
    return false;
}

bool OpenSotImpl::get_control_dt(double& dt)
{
    if(has_config() && get_config()["control_dt"])
    {
        dt = get_config()["control_dt"].as<double>();
        return true;
    }
    
    return false;
}


OpenSotImpl::~OpenSotImpl()
{
    _logger->flush();
}


