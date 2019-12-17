#include <cartesian_interface/open_sot/OpenSotAccImpl.h>
#include <cartesian_interface/problem/ProblemDescription.h>
#include <cartesian_interface/problem/Task.h>
#include <cartesian_interface/problem/Interaction.h>
#include <cartesian_interface/problem/Com.h>
#include <cartesian_interface/problem/Postural.h>
#include <cartesian_interface/problem/Gaze.h>
#include <cartesian_interface/problem/Limits.h>
#include <cartesian_interface/utils/LoadObject.hpp>
#include <boost/make_shared.hpp>
#include <OpenSoT/constraints/acceleration/JointLimits.h>
#include <OpenSoT/constraints/acceleration/VelocityLimits.h>
#include <OpenSoT/constraints/TaskToConstraint.h>
#include <cartesian_interface/problem/MinJointVel.h>


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
        else if(back_end_string == "odys")
        {
            return OpenSoT::solvers::solver_back_ends::ODYS;
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

void OpenSotAccImpl::links_in_contact_from_description(AggregatedTask stack,
                                                       std::vector<std::string>& distal_links,
                                                       std::vector<std::string>& base_links)
{
  for(TaskDescription::Ptr task_desc : stack)
  {
    if(task_desc->type == "Interaction")
    {
      auto i_desc = GetAsInteraction(task_desc);
      std::string distal_link = i_desc->distal_link;
      std::string base_link = i_desc->base_link;
       
      if (distal_links.empty() || std::find(distal_links.begin(), distal_links.end(), distal_link) == distal_links.end())
      {
        distal_links.push_back(distal_link);
        base_links.push_back(base_link);
        XBot::Logger::info("Link in contact parsed: %s\n", distal_link.c_str());
      }
    }
  }  
}

void XBot::Cartesian::OpenSotAccImpl::aggregate_force_tasks(AggregatedTask stack, OpenSoT::tasks::force::Wrenches::Ptr aggregated_force_tasks)
{
  for(TaskDescription::Ptr task_desc : stack)
  {
    if(task_desc->type == "Interaction")
    {
      auto i_desc = GetAsInteraction(task_desc);
      std::string distal_link = i_desc->distal_link;
      std::string base_link = i_desc->base_link;
       
//       if (links.empty() || std::find(links.begin(), links.end(), distal_link) == links.end())
//       {
//         links.push_back(distal_link);
//         XBot::Logger::info("Link in contact parsed: %s\n", distal_link.c_str());
//       }
      
//       aggregated_force_tasks
    }
  }
}

OpenSoT::tasks::Aggregated::TaskPtr OpenSotAccImpl::aggregated_from_regularization(AggregatedTask stack)
{
    return aggregated_from_stack_level(stack);
}


OpenSoT::tasks::Aggregated::TaskPtr OpenSotAccImpl::aggregated_from_stack_level(AggregatedTask stack)
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
        return boost::make_shared<OpenSoT::tasks::Aggregated>(tasks_list, _qddot.getInputSize());
    }
    else
    {
        return tasks_list.front();
    }
}

OpenSotAccImpl::TaskPtr OpenSotAccImpl::construct_task(TaskDescription::Ptr task_desc)
{
  
    bool skip_indices = false;
    
    OpenSotAccImpl::TaskPtr opensotacc_task;
    double control_dt = 0.01;
    if(!get_control_dt(control_dt))
    {
        throw std::runtime_error("Unable to find control period in configuration file " 
                                 "(add problem_description/solver_options/control_dt field)");
    }
    
    if(task_desc->type == "Cartesian")
    {
        auto cartesian_desc = GetAsCartesian(task_desc);
        std::string distal_link = cartesian_desc->distal_link;
        std::string base_link = cartesian_desc->base_link;
        
        //ADDED
        auto cartesian_task = boost::make_shared<OpenSoT::tasks::acceleration::Cartesian>
                                            (base_link + "_TO_" + distal_link,
                                                *_model,
                                                distal_link,
                                                base_link,
                                                _qddot
                                            );

        opensotacc_task = cartesian_task;
        
        
//         cartesian_task->setLambda(cartesian_desc->lambda);  // velocity tasks
        double lambda = (cartesian_desc->lambda * cartesian_desc->lambda)/(control_dt*control_dt);
        double lambda2 = cartesian_desc->lambda2;
//         if (lambda2 >= 0)
//         {
//             cartesian_task->setLambda(lambda,lambda2);
//         }
//         else
//         {
//             lambda2 = 2*std::sqrt(lambda);
//             cartesian_desc->lambda2 = lambda2;
//             cartesian_task->setLambda(lambda, lambda2);
//         }
        
        if (lambda2 < 0)
        {
            lambda2 = 2*std::sqrt(lambda);
            cartesian_desc->lambda2 = lambda2;
        }
        cartesian_task->setLambda(lambda, lambda2);
        
//         cartesian_task->setOrientationErrorGain(cartesian_desc->orientation_gain);

//         cartesian_task->setIsBodyJacobian(cartesian_desc->is_body_jacobian);

        _cartesian_tasks.push_back(cartesian_task);
        
        XBot::Logger::info("OpenSot: Cartesian found (%s -> %s), lambda = %f, lambda2 = %f, dofs = %d\n",
                            base_link.c_str(), 
                            distal_link.c_str(), 
                            cartesian_desc->lambda,
                            cartesian_desc->lambda2,
                            cartesian_desc->indices.size()
                            );

    }
    else if(task_desc->type == "MinJointVel")
    {
        auto mjv_desc = GetAsMinJointVel(task_desc);

        auto tmp_minjointvel_task = boost::make_shared<MinJointVelTask>
                (*_model, control_dt, _qddot);

        tmp_minjointvel_task->setEps(mjv_desc->eps);
        _min_joint_vel_tasks.push_back(tmp_minjointvel_task);

        opensotacc_task = tmp_minjointvel_task;
        XBot::Logger::info("OpenSot: MinJointVel found, eps = %f\n", mjv_desc->eps);

    }
    else if(task_desc->type == "Interaction")
    {
        auto i_desc = GetAsInteraction(task_desc);
        std::string distal_link = i_desc->distal_link;
        std::string base_link = i_desc->base_link;
        
//         if (_links_in_contact.empty() || std::find(_links_in_contact.begin(), _links_in_contact.end(), distal_link) == _links_in_contact.end())
//         {
//           _links_in_contact.push_back(distal_link);
//           XBot::Logger::info("Link in contact found: %s\n", distal_link.c_str());
//         }
        
        auto tmp_cartesian_task = boost::make_shared<CartesianAccTask>
                                            (base_link + "_TO_" + distal_link,
                                                *_model,
                                                distal_link,
                                                base_link,
                                                _qddot
                                            );
                                            
        for (int i = 0; i < i_desc->stiffness.size(); i++)
        {
          if(i_desc->stiffness(i) < 0.)
            throw std::runtime_error("Negative stiffness value found. Stiffness values must be positive");  
        }
        
        for (int i = 0; i < i_desc->damping.size(); i++)
        {
          if(i_desc->damping(i) < 0.)
            throw std::runtime_error("Negative damping value found. Damping values must be positive");  
        }
        
        
        double lambda, lambda2;
        
        // at the moment we set both lambdas equal to 1
        lambda = 1.;
        lambda2 = 1.;
        
        tmp_cartesian_task->setLambda(lambda, lambda2);
        tmp_cartesian_task->setKp(i_desc->stiffness.asDiagonal());
        tmp_cartesian_task->setKd(i_desc->damping.asDiagonal()); 
                                            
        auto tmp_force_task = _wrenches_->getWrenchTask(distal_link);
        
        OpenSoT::tasks::Aggregated::TaskPtr ic_task;
        
        std::list<uint> indices(task_desc->indices.begin(), task_desc->indices.end());
        if(tmp_force_task) // handle manually the indices now
        {
            if(indices.size() == 6)
            {
                ic_task = task_desc->weight*tmp_cartesian_task + task_desc->weight*tmp_force_task;
            }
            else
            {
                ic_task = task_desc->weight*(tmp_cartesian_task%indices) + task_desc->weight*(tmp_force_task%indices);
            }
            
            skip_indices = true;
            
        }
        else  // handle the indices later
        {
            ic_task = tmp_cartesian_task;
        }

//         // set lambda using tmp std vector
//         std::vector<double> stiffness, damping;
//         Eigen::Map<Eigen::Vector6d> stiffness_map(stiffness.data(),6);
//         Eigen::Map<Eigen::Vector6d> damping_map(damping.data(),6);
//         stiffness_map = i_desc->stiffness;
//         damping_map = i_desc->damping;
//         
//         // check if all the elements of i_desc->stiffness are equal
//         if ( std::adjacent_find( stiffness.data.begin(), stiffness.data.end(), std::not_equal_to<>() ) == stiffness.data.end() )
//         {
//           lambda = i_desc->stiffness(0); // set lambda equal to the stiffness
//         }
//         else 
//           throw std::runtime_error("Unsupported stiffness vector: elements should be all equal");
//         
//         // check if all the elements of i_desc->damping are equal
//         if ( std::adjacent_find( damping.data.begin(), damping.data.end(), std::not_equal_to<>() ) == damping.data.end() )
//         {
//           lambda2 = i_desc->damping(0); // set lambda2 equal to the damping
//         }
//         else 
//           throw std::runtime_error("Unsupported damping vector: elements should be all equal");
        
        // check if all the elements of i_desc->stiffness are equal
//         for (int i=1; i<i_desc->stiffness.size(); i++)
//         {
//           if (!(i_desc->stiffness(i-1) == i_desc->stiffness(i)))
//           {
//             XBot::Logger::warning("Unsupported stiffness vector: elements should be all equal\n");
//           }
//         }
        
        // check if all the elements of i_desc->damping are equal
//         for (int i=1; i<i_desc->damping.size(); i++)
//         {
//           if (!(i_desc->damping(i-1) == i_desc->damping(i)))
//           {
//             XBot::Logger::warning("Unsupported damping vector: elements should be all equal\n");
//           }
//         }
        
        opensotacc_task = ic_task;
        
        _cartesian_tasks.push_back(tmp_cartesian_task);
        _interaction_tasks.push_back(tmp_cartesian_task);
        _force_tasks.push_back(tmp_force_task);
        _aggregated_tasks.push_back(ic_task);
         
        XBot::Logger::info("OpenSot: Interaction found (%s -> %s), lambda = %f, lambda2 = %f, dofs = %d\n",
                            base_link.c_str(), 
                            distal_link.c_str(), 
                            i_desc->lambda,
                            i_desc->lambda2,
                            i_desc->indices.size()
                            );
        
        
    }
//     else if(task_desc->type == "Com")
//     {
//         auto com_desc = GetAsCom(task_desc);
//         opensotacc_task = _com_task = boost::make_shared<CoMTask>(_q, *_model);
//         
//         _com_task->setLambda(com_desc->lambda);
//         
//         XBot::Logger::info("OpenSot: Com found, lambda is %f\n", com_desc->lambda);
//     }
    else if(task_desc->type == "Postural")
    {
        auto postural_desc = GetAsPostural(task_desc);
        _use_inertia_matrix.push_back(postural_desc->use_inertia_matrix);

         auto postural_task = boost::make_shared<OpenSoT::tasks::acceleration::Postural>(*_model, _qddot);
        
        _postural_tasks.push_back(postural_task);

        double lambda = (postural_desc->lambda * postural_desc->lambda)/(control_dt*control_dt);
        double lambda2 = postural_desc->lambda2;

        if (lambda2 < 0)
        {
            lambda2 = 2*std::sqrt(lambda);
            postural_desc->lambda2 = lambda2;
        }
        postural_task->setLambda(lambda, lambda2);
        
        opensotacc_task = postural_task;
        
        XBot::Logger::info("OpenSot: Postural found, lambda = %f, lambda2 = %f, dofs = %d, use inertia matrix: %d\n",
            postural_desc->lambda,
            postural_desc->lambda2,
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
    
    if(!skip_indices)
    {
      
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
        
        double control_dt = 0.01;
        if(!get_control_dt(control_dt))
        {
            Logger::warning("Unable to find control period \
in configuration file (add problem_description/solver_options/control_dt field)\n");
        }

        auto joint_lims = boost::make_shared<OpenSoT::constraints::acceleration::JointLimits>
                                            (*_model, 
                                             _qddot,
                                             qmax,
                                             qmin,
                                             Eigen::VectorXd::Ones(qmax.size()) * 40.0,
                                             control_dt
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
        
        auto vel_lims = boost::make_shared<OpenSoT::constraints::acceleration::VelocityLimits>
                                            (*_model, _qddot, qdotmax, control_dt);

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
                               ProblemDescription id_problem):
    CartesianInterfaceImpl(model, id_problem),
    _logger(XBot::MatLogger::getLogger("/tmp/xbot_cartesian_opensot_log")),
    _update_lambda(false)
{
    _model->getJointPosition(_q);
    _dq.setZero(_q.size());
    _ddq = _dq;
    _tau = _dq;
    
//     _l1_old = 0.;
//     _l2_old = 0.;

    _B.setIdentity(_q.size(), _q.size());
    
    // ADDED
    /* Parse stack levels  */
    for(int i = 0; i < id_problem.getNumTasks(); i++)
    {
        links_in_contact_from_description(id_problem.getTask(i), _links_in_contact, _base_links);
    }
//     XBot::Logger::info("ID - Links in contact : %i\n", _links_in_contact.size());
    _id = boost::make_shared<OpenSoT::utils::InverseDynamics>(_links_in_contact, *_model);
    _qddot = _id->getJointsAccelerationAffine();
    _contact_wrenches = _id->getContactsWrenchAffine();
    for(unsigned int i = 0; i < _contact_wrenches.size(); ++i)
    {
        _contact_wrncs.push_back(Eigen::Vector6d::Zero());
    }

//     XBot::Logger::info("ID - Contact wrench size : %i\n", _contact_wrenches[0].getOutputSize());
    _x.setZero(_qddot.getInputSize());
    _tau.setZero(_q.size());
    
    /* Parse force tasks */
    _wrenches_ = boost::make_shared<OpenSoT::tasks::force::Wrenches>("wrenches", _links_in_contact, _base_links, _contact_wrenches);
       
    // END ADDED

    /* Parse stack #0 (first level of the stack) and create autostack */
    auto stack_0 = aggregated_from_stack_level(id_problem.getTask(0));
    _autostack = boost::make_shared<OpenSoT::AutoStack>(stack_0);

    /* Parse remaining levels  */
    for(int i = 1; i < id_problem.getNumTasks(); i++)
    {
        _autostack = _autostack / aggregated_from_stack_level(id_problem.getTask(i));
    }

    /* Parse constraints */
    for(auto constr : id_problem.getBounds())
    {
        auto constr_ptr = constraint_from_description(constr);

        if(constr_ptr)
        {
            _autostack << constr_ptr;
        }
    }

    if(id_problem.getRegularizationTask().size() > 0)
        _autostack->setRegularisationTask(
                    aggregated_from_regularization(id_problem.getRegularizationTask()));
    
    /* Parse solver configs (if any) */
    using BackEnd = OpenSoT::solvers::solver_back_ends;
    double eps_regularization = 1e6;
    BackEnd solver_backend = BackEnd::qpOASES;
    
    if(has_config() && get_config()["back_end"])
    {
        std::string back_end_string = get_config()["back_end"].as<std::string>();
        solver_backend = ::backend_from_string(back_end_string);
    }
    
    if(has_config() && get_config()["regularization"])
    {
        eps_regularization = get_config()["regularization"].as<double>();
        eps_regularization *= 1e12;
    }
    

    std::string back_end = OpenSoT::solvers::whichBackEnd(solver_backend);

    Logger::info(Logger::Severity::HIGH, "OpenSot: using back-end %s\n", back_end.c_str());
    Logger::info(Logger::Severity::HIGH, "OpenSot: regularization value is %.1e\n", eps_regularization);

    /* Create solver */
    _solver = boost::make_shared<OpenSoT::solvers::iHQP>(*_autostack,
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


        cart_task->setReference(T_ref, v_ref);

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
    
    /* Set desired interaction */
    for(auto t : _interaction_tasks)
    {
        Eigen::Vector6d f;
        Eigen::Matrix6d k, d;    
        if(getDesiredInteraction(t->getDistalLink(), f, k, d))
        {         
          t->setKp(k);
          t->setKd(d);
          t->setVirtualForce(f);
      } 
    }
    
//    for(auto t: _force_tasks)
//    {
//      Eigen::Vector6d f;
//      Eigen::Matrix6d k, d;
//      if(getDesiredInteraction(t->getDistalLink(), f, k, d))
//      {
//        // the solver expects forces from the environment to the robot, so to set the forces from the robot to the environment we change sign
//        t->setReference(-f);
////         Logger::info() << "Set new force: " << f.transpose() << Logger::endl();
//      }
//    }
    
    _autostack->update(_q);
    _autostack->log(_logger);

    
    if(!_solver->solve(_x))
    {
        _x.setZero(_x.size());
        XBot::Logger::error("OpenSot: unable to solve\n");
        success = false;
    }
    
//     Logger::info() << "Solution: " << _x.transpose() << Logger::endl();

    _id->computedTorque(_x, _tau, _ddq, _contact_wrncs);
    
    _model->setJointAcceleration(_ddq);
    _model->setJointEffort(_tau);

    _logger->add("ddq", _ddq);
    _logger->add("tau", _tau);
    _logger->add("solution", _x);
    for(unsigned int i = 0; i < _contact_wrncs.size(); ++i)
        _logger->add("contact_wrench_"+i, _contact_wrncs[i]);


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


