#include <cartesian_interface/open_sot/OpenSotImpl.h>
#include <boost/make_shared.hpp>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <OpenSoT/constraints/velocity/VelocityLimits.h>



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
        switch(task_desc->type)
        {
            case TaskType::Cartesian:
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
                    tasks_list.push_back(cartesian_desc->weight*(cartesian_task));
                }
                else
                {
                    tasks_list.push_back(cartesian_desc->weight*(cartesian_task%indices));
                }

                break;
            }
            case TaskType::Gaze:
            {
                auto gaze_desc = XBot::Cartesian::GetAsGaze(task_desc);
                std::string base_link = gaze_desc->base_link;

                _gaze_task = boost::make_shared<GazeTask>(base_link + "_TO_" + "gaze",_q, *_model, base_link);
                _gaze_task->setLambda(gaze_desc->lambda);

                std::list<uint> indices(gaze_desc->indices.begin(), gaze_desc->indices.end());

                if(indices.size() == 2)
                {
                    tasks_list.push_back(gaze_desc->weight*(_gaze_task));
                }
                else
                {
                    tasks_list.push_back(gaze_desc->weight*(_gaze_task%indices));
                }

                break;
            }
            case TaskType::Com:
            {
                auto com_desc = XBot::Cartesian::GetAsCom(task_desc);
                _com_task = boost::make_shared<CoMTask>(_q, *_model);
                
                _com_task->setLambda(com_desc->lambda);

                std::list<uint> indices(com_desc->indices.begin(), com_desc->indices.end());

                if(indices.size() == 3)
                {
                    tasks_list.push_back(com_desc->weight*(_com_task));
                }
                else
                {
                    tasks_list.push_back(com_desc->weight*(_com_task%indices));
                }
                break;
            }
            case TaskType::Postural:
            {
                auto postural_desc = XBot::Cartesian::GetAsPostural(task_desc);

                auto postural_task = boost::make_shared<OpenSoT::tasks::velocity::Postural>(_q);
                
                _postural_tasks.push_back(postural_task);
                
                postural_task->setLambda(postural_desc->lambda);

                std::list<uint> indices(postural_desc->indices.begin(), postural_desc->indices.end());

                if(indices.size() == _model->getJointNum())
                {
                    tasks_list.push_back(postural_desc->weight*(postural_task));
                }
                else
                {
                    tasks_list.push_back(postural_desc->weight*(postural_task%indices));
                }
                break;
            }
            default:
                Logger::warning("OpenSot: task type not supported\n");
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

OpenSoT::Constraint< Eigen::MatrixXd, Eigen::VectorXd >::ConstraintPtr XBot::Cartesian::OpenSotImpl::constraint_from_description(XBot::Cartesian::ConstraintDescription::Ptr constr_desc)
{
    switch(constr_desc->type)
    {
        case ConstraintType::JointLimits:
        {
            Eigen::VectorXd qmin, qmax;
            _model->getJointLimits(qmin, qmax);

            auto joint_lims = boost::make_shared<OpenSoT::constraints::velocity::JointLimits>
                                                (_q,
                                                    qmax,
                                                    qmin
                                                    );

            return joint_lims;

            break;

        }

        case ConstraintType::VelocityLimits:
        {
            Eigen::VectorXd qdotmax;
            _model->getVelocityLimits(qdotmax);

            auto vel_lims = boost::make_shared<OpenSoT::constraints::velocity::VelocityLimits>
                                                (qdotmax, 0.01);

            return vel_lims;

            break;
        }

        default:
            XBot::Logger::warning("Unsupported constraint type\n");
            return nullptr;
            break;
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
        cart_task->setReference(T_ref.matrix(), v_ref);

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
        for(auto postural : _postural_tasks)
        {
            postural->setReference(_qref);
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

