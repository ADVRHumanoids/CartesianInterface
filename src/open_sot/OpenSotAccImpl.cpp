#include <cartesian_interface/open_sot/OpenSotAccImpl.h>
#include <cartesian_interface/problem/ProblemDescription.h>
#include <cartesian_interface/problem/Task.h>
#include <cartesian_interface/problem/Interaction.h>
#include <cartesian_interface/problem/Com.h>
#include <cartesian_interface/problem/Postural.h>
#include <cartesian_interface/problem/Gaze.h>
#include <cartesian_interface/problem/Limits.h>
#include <cartesian_interface/problem/AngularMomentum.h>
#include <cartesian_interface/utils/LoadObject.hpp>
#include <boost/make_shared.hpp>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <OpenSoT/constraints/velocity/VelocityLimits.h>
#include <OpenSoT/constraints/TaskToConstraint.h>

using namespace XBot::Cartesian;


extern "C" CartesianInterface* create_instance(XBot::ModelInterface::Ptr model,
                                               ProblemDescription pb)
{
    return new OpenSotAccImpl(model, pb);
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
        else
        {
            throw std::runtime_error("Invalid back end '" + back_end_string + "'");
        }
    }
}


bool OpenSotAccImpl::setBaseLink(const std::string& ee_name, const std::string& new_base_link)
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


OpenSoT::tasks::Aggregated::TaskPtr OpenSotAccImpl::aggregated_from_stack(AggregatedTask stack)
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

OpenSotAccImpl::TaskPtr OpenSotAccImpl::construct_task(TaskDescription::Ptr task_desc)
{
    
    OpenSotAccImpl::TaskPtr opensotacc_task;
    
    
    
    if(task_desc->type == "Cartesian")
    {
        auto cartesian_desc = GetAsCartesian(task_desc);
        std::string distal_link = cartesian_desc->distal_link;
        std::string base_link = cartesian_desc->base_link;
        
//         auto cartesian_task = boost::make_shared<OpenSoT::tasks::velocity::Cartesian>
//                                             (base_link + "_TO_" + distal_link,
//                                                 _q,
//                                                 *_model,
//                                                 distal_link,
//                                                 base_link
//                                                 );
        //ADDED
        auto cartesian_task = boost::make_shared<OpenSoT::tasks::acceleration::Cartesian>
                                            (base_link + "_TO_" + distal_link,
                                                *_model,
                                                distal_link,
                                                base_link,
                                                _qddot
                                            );

        opensotacc_task = cartesian_task;
        
        cartesian_task->setLambda(cartesian_desc->lambda);
//         cartesian_task->setOrientationErrorGain(cartesian_desc->orientation_gain);

//         cartesian_task->setIsBodyJacobian(cartesian_desc->is_body_jacobian);

        _cartesian_tasks.push_back(cartesian_task);
        
        XBot::Logger::info("OpenSot: Cartesian found (%s -> %s), lambda = %f, dofs = %d\n", 
                            base_link.c_str(), 
                            distal_link.c_str(), 
                            cartesian_desc->lambda,
                            cartesian_desc->indices.size()
                            );

    }
//     else if(task_desc->type == "Interaction")
//     {
//         auto i_desc = GetAsInteraction(task_desc);
//         std::string distal_link = i_desc->distal_link;
//         std::string base_link = i_desc->base_link;
//         
//         
//         XBot::ForceTorqueSensor::ConstPtr ft;
//         
//         
//         try
//         {
//             ft = _model->getForceTorque().at(distal_link);
//         }
//         catch(...)
//         {
//             if(!_force_estimation)
//             {
//                 _force_estimation = std::make_shared<Utils::ForceEstimation>(_model);
//             }
//             
//             ft = _force_estimation->add_link(distal_link, 
//                                              {}, 
//                                              i_desc->force_estimation_chains);
//         }
//         
//         auto admittance_task = boost::make_shared<OpenSoT::tasks::velocity::CartesianAdmittance>
//                                                ("adm_" + base_link + "_TO_" + distal_link,
//                                                 _q,
//                                                 *_model,
//                                                 base_link,
//                                                 ft
//                                                 );
//                                                
//         opensotacc_task = admittance_task;
// 
//         admittance_task->setOrientationErrorGain(i_desc->orientation_gain);
//         
//         double control_dt = 0.01;
//         if(!get_control_dt(control_dt))
//         {
//             throw std::runtime_error("Unable to find control period in configuration file " 
//                                      "required by admittance task "  
//                                      "(add problem_description/solver_options/control_dt field)");
//         }
//         
//         if(i_desc->lambda > 0)
//         {
//         
//             admittance_task->setImpedanceParams(i_desc->stiffness,
//                                                 i_desc->damping, 
//                                                 i_desc->lambda, 
//                                                 control_dt
//                                             );
//             
//         }
//         else
//         {
//             if((i_desc->inertia.array() <= 0).any())
//             {
//                 throw std::invalid_argument("all inertias must be > 0 when lambda = 0");
//             }
//             
//             admittance_task->setRawParams(i_desc->damping.cwiseInverse() * control_dt, 
//                 i_desc->damping.cwiseProduct(i_desc->inertia.cwiseInverse()),
//                 0.0, 
//                 control_dt
//             );
//         }
//         
//         admittance_task->setDeadZone(i_desc->force_dead_zone);
// 
//         _cartesian_tasks.push_back(admittance_task);
//         _admittance_tasks.push_back(admittance_task);
// 
//         XBot::Logger::info("OpenSot: Admittance found (%s -> %s), lambda = %f, dofs = %d\n", 
//                             base_link.c_str(), 
//                             distal_link.c_str(), 
//                             i_desc->lambda,
//                             i_desc->indices.size()
//                             );
// 
//     }
//     else if(task_desc->type == "Gaze")
//     {
//         
//         auto gaze_desc = GetAsGaze(task_desc);
//         std::string base_link = gaze_desc->base_link;
// 
//         opensotacc_task = _gaze_task = boost::make_shared<GazeTask>(base_link + "_TO_" + "gaze",_q, *_model, base_link);
//         _gaze_task->setLambda(gaze_desc->lambda);
// 
//         
//         XBot::Logger::info("OpenSot: Gaze found, base_link is %s, lambda is %f\n", 
//             gaze_desc->base_link.c_str(),
//             gaze_desc->lambda
//         );
// 
//     }
//     else if(task_desc->type == "Com")
//     {
//         auto com_desc = GetAsCom(task_desc);
//         opensotacc_task = _com_task = boost::make_shared<CoMTask>(_q, *_model);
//         
//         _com_task->setLambda(com_desc->lambda);
//         
//         XBot::Logger::info("OpenSot: Com found, lambda is %f\n", com_desc->lambda);
//     }
//     else if(task_desc->type == "AngularMomentum")
//     {
//         auto angular_mom_desc = GetAsAngularMomentum(task_desc);
//         _minimize_rate_of_change = angular_mom_desc->min_rate;
// 
//         opensotacc_task = _angular_momentum_task = boost::make_shared<AngularMomentumTask>(_q, *_model);
// 
// 
//         XBot::Logger::info("OpenSot: Angular Momentum found, rate of change minimization set to: %d\n", _minimize_rate_of_change);
//     }
    else if(task_desc->type == "Postural")
    {
        auto postural_desc = GetAsPostural(task_desc);
        _use_inertia_matrix.push_back(postural_desc->use_inertia_matrix);

//         auto postural_task = boost::make_shared<OpenSoT::tasks::velocity::Postural>(_q);
         auto postural_task = boost::make_shared<OpenSoT::tasks::acceleration::Postural>(*_model, _qddot);
        
        _postural_tasks.push_back(postural_task);
        
        postural_task->setLambda(postural_desc->lambda);
        
        opensotacc_task = postural_task;
        
        XBot::Logger::info("OpenSot: Postural found, lambda is %f, %d dofs, use inertia matrix: %d\n",
            postural_desc->lambda,
            postural_desc->indices.size(),
            postural_desc->use_inertia_matrix
        );
    }
    else if(!task_desc->lib_name.empty())
    {
        auto task_ifc = Utils::LoadObject<SoT::TaskInterface>(task_desc->lib_name, 
                                          task_desc->type + "OpenSotTaskFactory",
                                          task_desc, _model
                                         );
        
        if(task_ifc)
        {
            opensotacc_task = task_ifc->getTaskPtr();
            _task_ifc.emplace_back(std::move(task_ifc));
        }
        else
        {
            Logger::warning("OpenSot: unable to construct task type '%s'd\n", 
                task_desc->type.c_str());
        }
    }
    else
    {
        Logger::warning("OpenSot: task type '%s' not supported\n",
            task_desc->type.c_str()
        );
    }
    
    /* Apply active joint mask */
    if(task_desc->disabled_joints.size() > 0)
    {
        std::vector<bool> active_joints_mask(_model->getJointNum(), true);
        
        for(auto jstr : task_desc->disabled_joints)
        {
            active_joints_mask.at(_model->getDofIndex(jstr)) = false;
        }
        
        opensotacc_task->setActiveJointsMask(active_joints_mask);
    }
    
    /* Apply weight and extract subtask */
    std::list<uint> indices(task_desc->indices.begin(), task_desc->indices.end());

    // for postural tasks, active joint masks acts on indices as well    
    if(task_desc->type == "Postural")
    {

        auto is_disabled = [opensotacc_task](uint i)
        {
            return !opensotacc_task->getActiveJointsMask().at(i);
        };
        
        indices.remove_if(is_disabled);
        
        std::vector<int> indices_vec(indices.begin(), indices.end());
        task_desc = indices_vec % task_desc;
        
    }
    
    if(indices.size() == opensotacc_task->getTaskSize())
    {
        opensotacc_task = task_desc->weight*(opensotacc_task);
    }
    else
    {
        opensotacc_task = task_desc->weight*(opensotacc_task%indices);
    }
    
    /* Return task */
    return opensotacc_task;
}



OpenSoT::Constraint< Eigen::MatrixXd, Eigen::VectorXd >::ConstraintPtr OpenSotAccImpl::constraint_from_description(ConstraintDescription::Ptr constr_desc)
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
            auto opensotacc_task = construct_task(task);
            
            if(opensotacc_task)
            {
                return boost::make_shared<OpenSoT::constraints::TaskToConstraint>(opensotacc_task);
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
            return constr_ifc->getConstraintPtr();
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


OpenSotAccImpl::OpenSotAccImpl(XBot::ModelInterface::Ptr model,
                               ProblemDescription ik_problem):
    CartesianInterfaceImpl(model, ik_problem),
    _logger(XBot::MatLogger::getLogger("/tmp/xbot_cartesian_opensot_log")),
    _update_lambda(false)
{
    _model->getJointPosition(_q);
    _dq.setZero(_q.size());
    _ddq = _dq;
    _tau = _dq;

    _B.setIdentity(_q.size(), _q.size());
    
    // ADDED
    // for now it _links_in_contact is empty since there are no links in contact
    _id = boost::make_shared<OpenSoT::utils::InverseDynamics>(_links_in_contact, *_model);
    _qddot = _id->getJointsAccelerationAffine();
    _x.setZero(_qddot.getInputSize());
    _tau.setZero(_q.size());
    // END ADDED

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
        
        solver_backend = ::backend_from_string(back_end_string);
    }
    
    if(has_config() && get_config()["regularization"])
    {
        eps_regularization = get_config()["regularization"].as<double>();
        eps_regularization *= 1e12;
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

void OpenSotAccImpl::lambda_callback(const std_msgs::Float32ConstPtr& msg, const string& ee_name)
{
    _lambda_map.at(ee_name) = msg->data;
    _update_lambda = true;
}


bool OpenSotAccImpl::initRos(ros::NodeHandle nh)
{
    
    
    
//     for(auto t: _cartesian_tasks)
//     {
//         std::string ee_name = t->getDistalLink();
//         auto sub = nh.subscribe<std_msgs::Float32>(ee_name + "/lambda", 
//                                                     1, 
//                                                     boost::bind(&OpenSotAccImpl::lambda_callback, 
//                                                                 this,
//                                                                 _1,
//                                                                 ee_name
//                                                                 )
//                                                    );
//         _lambda_sub_map[ee_name] = sub;
//         
//     }
    
    return true;
}

void OpenSotAccImpl::updateRos()
{

}



bool OpenSotAccImpl::update(double time, double period)
{
    bool success = true;

    CartesianInterfaceImpl::update(time, period);

    _model->getJointPosition(_q);
    
    /* Update all plugin-based tasks */
    for(auto t : _task_ifc)
    {
        t->update(this, time, period);
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

//     if(!_solver->solve(_dq))
//     {
//         _dq.setZero(_dq.size());
//         XBot::Logger::error("OpenSot: unable to solve\n");
//         success = false;
//     }
    
    if(!_solver->solve(_x))
    {
        _x.setZero(_x.size());
        XBot::Logger::error("OpenSot: unable to solve\n");
        success = false;
    }

//     _dq /= period;
//     _model->setJointVelocity(_dq);
    _id->computedTorque(_x, _tau, _ddq);
    
    _model->setJointAcceleration(_ddq);
    _model->setJointEffort(_tau);

    _logger->add("q", _q);


    return success;


}

bool OpenSotAccImpl::setControlMode(const std::string& ee_name, 
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

bool OpenSotAccImpl::get_control_dt(double& dt)
{
    if(has_config() && get_config()["control_dt"])
    {
        dt = get_config()["control_dt"].as<double>();
        return true;
    }
    
    return false;
}


OpenSotAccImpl::~OpenSotAccImpl()
{
    _logger->flush();
}


