#include <cartesian_interface/open_sot/OpenSotImpl.h>
#include <cartesian_interface/problem/ProblemDescription.h>
#include <cartesian_interface/problem/Task.h>
#include <cartesian_interface/problem/Cartesian.h>
#include <cartesian_interface/problem/Com.h>
#include <cartesian_interface/problem/Postural.h>
#include <cartesian_interface/problem/Gaze.h>
#include <cartesian_interface/problem/Limits.h>
#include <cartesian_interface/problem/AngularMomentum.h>
#include <boost/make_shared.hpp>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <OpenSoT/constraints/velocity/VelocityLimits.h>
#include <OpenSoT/constraints/TaskToConstraint.h>



extern "C" XBot::Cartesian::CartesianInterface* create_instance(XBot::ModelInterface::Ptr model,
                                                                XBot::Cartesian::ProblemDescription pb)
{
    return new XBot::Cartesian::OpenSotImpl(model, pb);
}

extern "C" void destroy_instance( XBot::Cartesian::CartesianInterface* instance )
{
    delete instance;
}

bool XBot::Cartesian::OpenSotImpl::setBaseLink(const std::string& ee_name, const std::string& new_base_link)
{
    if(!XBot::Cartesian::CartesianInterfaceImpl::setBaseLink(ee_name, new_base_link))
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


OpenSoT::tasks::Aggregated::TaskPtr XBot::Cartesian::OpenSotImpl::aggregated_from_stack(XBot::Cartesian::AggregatedTask stack)
{
    std::list<OpenSoT::tasks::Aggregated::TaskPtr> tasks_list;

    for(XBot::Cartesian::TaskDescription::Ptr task_desc : stack)
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
        return *tasks_list.begin();
    }
}

XBot::Cartesian::OpenSotImpl::TaskPtr XBot::Cartesian::OpenSotImpl::construct_task(XBot::Cartesian::TaskDescription::Ptr task_desc)
{
    
    XBot::Cartesian::OpenSotImpl::TaskPtr opensot_task;
    
    if(task_desc->type == "Cartesian")
    {
        auto cartesian_desc = XBot::Cartesian::GetAsCartesian(task_desc);
        std::string distal_link = cartesian_desc->distal_link;
        std::string base_link = cartesian_desc->base_link;
        
        auto cartesian_task = boost::make_shared<OpenSoT::tasks::velocity::Cartesian>
                                            (base_link + "_TO_" + distal_link,
                                                _q,
                                                *_model,
                                                distal_link,
                                                base_link
                                                );

        cartesian_task->setLambda(cartesian_desc->lambda);
        cartesian_task->setOrientationErrorGain(cartesian_desc->orientation_gain);

        std::list<uint> indices(cartesian_desc->indices.begin(), cartesian_desc->indices.end());

        _cartesian_tasks.push_back(cartesian_task);

        if(indices.size() == 6)
        {
            opensot_task = cartesian_desc->weight*(cartesian_task);
        }
        else
        {
            opensot_task = cartesian_desc->weight*(cartesian_task%indices);
        }
        
        XBot::Logger::info("OpenSot: Cartesian found (%s -> %s), lambda = %f, dofs = %d\n", 
                            base_link.c_str(), 
                            distal_link.c_str(), 
                            cartesian_desc->lambda,
                            indices.size()
                            );

    }
    else if(task_desc->type == "Gaze")
    {
        
        auto gaze_desc = XBot::Cartesian::GetAsGaze(task_desc);
        std::string base_link = gaze_desc->base_link;

        _gaze_task = boost::make_shared<GazeTask>(base_link + "_TO_" + "gaze",_q, *_model, base_link);
        _gaze_task->setLambda(gaze_desc->lambda);
        
        
        std::list<uint> indices(gaze_desc->indices.begin(), gaze_desc->indices.end());

        if(indices.size() == 2)
        {
            opensot_task = gaze_desc->weight*(_gaze_task);
        }
        else
        {
            opensot_task = gaze_desc->weight*(_gaze_task%indices);
        }
        
        XBot::Logger::info("OpenSot: Gaze found, base_link is %s, lambda is %f\n", 
            gaze_desc->base_link.c_str(),
            gaze_desc->lambda
        );

    }
    else if(task_desc->type == "Com")
    {
        auto com_desc = XBot::Cartesian::GetAsCom(task_desc);
        _com_task = boost::make_shared<CoMTask>(_q, *_model);
        
        _com_task->setLambda(com_desc->lambda);

        std::list<uint> indices(com_desc->indices.begin(), com_desc->indices.end());

        if(indices.size() == 3)
        {
            opensot_task = com_desc->weight*(_com_task);
        }
        else
        {
            opensot_task = com_desc->weight*(_com_task%indices);
        }
        
        XBot::Logger::info("OpenSot: Com found, lambda is %f\n", com_desc->lambda);
    }
    else if(task_desc->type == "AngularMomentum")
    {
        auto angular_mom_desc = XBot::Cartesian::GetAsAngularMomentum(task_desc);
        _minimize_rate_of_change = angular_mom_desc->min_rate;

        _angular_momentum_task = boost::make_shared<AngularMomentumTask>(_q, *_model);

        std::list<uint> indices(angular_mom_desc->indices.begin(), angular_mom_desc->indices.end());

        if(indices.size() == 3)
        {
            opensot_task = angular_mom_desc->weight*(_angular_momentum_task);
        }
        else
        {
            opensot_task = angular_mom_desc->weight*(_angular_momentum_task%indices);
        }

        XBot::Logger::info("OpenSot: Angular Momentum found, rate of change minimization set to: %d\n", _minimize_rate_of_change);
    }
    else if(task_desc->type == "Postural")
    {
        auto postural_desc = XBot::Cartesian::GetAsPostural(task_desc);
        _use_inertia_matrix.push_back(postural_desc->use_inertia_matrix);

        auto postural_task = boost::make_shared<OpenSoT::tasks::velocity::Postural>(_q);
        
        _postural_tasks.push_back(postural_task);
        
        postural_task->setLambda(postural_desc->lambda);

        std::list<uint> indices(postural_desc->indices.begin(), postural_desc->indices.end());

        if(indices.size() == _model->getJointNum())
        {
            opensot_task = postural_desc->weight*(postural_task);
        }
        else
        {
            opensot_task = postural_desc->weight*(postural_task%indices);
        }
        
        XBot::Logger::info("OpenSot: Postural found, lambda is %f, %d dofs, use inertia matrix: %d\n",
            postural_desc->lambda,
            postural_desc->indices.size(),
            postural_desc->use_inertia_matrix
        );
    }
    else
    {
            Logger::warning("OpenSot: task type not supported\n");
    }
    
    return opensot_task;
}



OpenSoT::Constraint< Eigen::MatrixXd, Eigen::VectorXd >::ConstraintPtr XBot::Cartesian::OpenSotImpl::constraint_from_description(XBot::Cartesian::ConstraintDescription::Ptr constr_desc)
{
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

        auto vel_lims = boost::make_shared<OpenSoT::constraints::velocity::VelocityLimits>
                                            (qdotmax, 0.01);

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
    else
    {
        XBot::Logger::warning("Unsupported constraint type (%s)\n", constr_desc->type.c_str());
        return nullptr;
    }
}


XBot::Cartesian::OpenSotImpl::OpenSotImpl(XBot::ModelInterface::Ptr model,
                                          XBot::Cartesian::ProblemDescription ik_problem):
    CartesianInterfaceImpl(model, ik_problem),
    _logger(XBot::MatLogger::getLogger("/tmp/xbot_cartesian_opensot_log"))
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
    
    if(has_config() && get_config()["back_end"])
    {
        std::string back_end_string = get_config()["back_end"].as<std::string>();
        
        if(back_end_string != "osqp" && back_end_string != "qpoases")
        {
            Logger::warning("OpenSot: unrecognised solver %s, falling back to qpOASES (available options: qpoases, osqp)\n", back_end_string.c_str());
        }
        
        solver_backend = back_end_string == "qpoases" ? BackEnd::qpOASES : BackEnd::OSQP;
    }
    
    if(has_config() && get_config()["regularization"])
    {
        eps_regularization = get_config()["regularization"].as<double>();
        if(solver_backend == BackEnd::qpOASES)
        {
            eps_regularization *= 1e12;
        }
    }
    
    std::string back_end = solver_backend == BackEnd::qpOASES ? "qpOASES" : "OSQP";
    
    Logger::info(Logger::Severity::HIGH, "OpenSot: using back-end %s\n", back_end.c_str());
    Logger::info(Logger::Severity::HIGH, "OpenSot: regularization value is %.1e\n", eps_regularization);

    /* Create solver */
    _solver = boost::make_shared<OpenSoT::solvers::iHQP>(_autostack->getStack(),
                                                         _autostack->getBounds(),
                                                         eps_regularization,
                                                         solver_backend
                                                        );
    
    /* Fill lambda map */
    if(_com_task)
    {
        _lambda_map["com"] = _com_task->getLambda();
    }

    if(_gaze_task)
    {
        _lambda_map["gaze"] = _gaze_task->getLambda();
    }
    
    for(auto t : _cartesian_tasks)
    {
        _lambda_map[t->getDistalLink()] = t->getLambda();
    }
    
    
    

}

bool XBot::Cartesian::OpenSotImpl::update(double time, double period)
{
    bool success = true;

    XBot::Cartesian::CartesianInterfaceImpl::update(time, period);

    _model->getJointPosition(_q);

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

    /* Handle AngularMomentum */
    if(_angular_momentum_task)
    {
        if(_minimize_rate_of_change)
        {
            Eigen::Vector6d MoM;
            _model->getCentroidalMomentum(MoM);
            _angular_momentum_task->setReference(MoM.tail(3)*period);
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

bool XBot::Cartesian::OpenSotImpl::setControlMode(const std::string& ee_name, 
                                                  XBot::Cartesian::CartesianInterface::ControlType ctrl_type)
{
    if(!XBot::Cartesian::CartesianInterfaceImpl::setControlMode(ee_name, ctrl_type))
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


XBot::Cartesian::OpenSotImpl::~OpenSotImpl()
{
    _logger->flush();
}

void XBot::Cartesian::OpenSotImpl::set_adaptive_lambda(OpenSoT::tasks::velocity::Cartesian::Ptr cartesian_task)
{
    Eigen::Vector6d error = cartesian_task->getError();
    const double e_0 = 0.02;
    const double lambda =  1.0/(1 + std::pow(error.norm() / e_0, 2.0));

    std::cout << cartesian_task->getDistalLink() << ": " << lambda << std::endl;

    cartesian_task->setLambda(lambda);
}

