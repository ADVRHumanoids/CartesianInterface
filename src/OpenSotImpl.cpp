#include <CartesianInterface/open_sot/OpenSotImpl.h>
#include <boost/make_shared.hpp>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <OpenSoT/constraints/velocity/VelocityLimits.h>

OpenSoT::tasks::Aggregated::Ptr XBot::Cartesian::OpenSotImpl::aggregated_from_stack(XBot::Cartesian::AggregatedTask stack)
{
    std::list<OpenSoT::tasks::Aggregated::TaskPtr> tasks_list;
    
    for(XBot::Cartesian::TaskDescription::Ptr task_desc : stack)
    {
        switch(task_desc->type)
        {
            case TaskType::Cartesian:
                
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
    }
    
    /* Return Aggregated */
    return boost::make_shared<OpenSoT::tasks::Aggregated>(tasks_list, _q.size());
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
    
    /* Add postural task by default */
    auto postural_task = boost::make_shared<OpenSoT::tasks::velocity::Postural>(_q);
    postural_task->setLambda(0.01);
    
    _autostack = _autostack / postural_task;
    
    /* Parse constraints */
    for(auto constr : ik_problem.getBounds())
    {
        auto constr_ptr = constraint_from_description(constr);
        
        if(constr_ptr)
        {
            _autostack << constr_ptr;
        }
    }
    
    /* Create solver */
    _solver = boost::make_shared<OpenSoT::solvers::QPOases_sot>(_autostack->getStack(), 
                                                                _autostack->getBounds(), 1e8);
    
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
        
        cart_task->setReference(T_ref.matrix(), v_ref);
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

XBot::Cartesian::OpenSotImpl::~OpenSotImpl()
{
    _logger->flush();
}


