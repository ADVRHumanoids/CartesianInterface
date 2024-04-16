#include "cartesioplugin.h"

using namespace XBot;
using namespace XBot::Cartesian;

bool CartesioRt::on_initialize()
{
    setJournalLevel(Journal::Level::Low);

    _nh = std::make_unique<ros::NodeHandle>();

    /* Get ik problem from ros param */
    auto problem_param = getParamOr<std::string>("~problem_param",
                                                 "cartesian/problem_description");
    std::string ik_str;
    if(getParam("~problem_description/content", ik_str))
    {

    }
    else if(!_nh->getParam(problem_param, ik_str))
    {
        jerror("ros param '{}' not found \n", problem_param);
        return false;
    }

    auto ik_yaml = YAML::Load(ik_str);

    /* Create model and ci for rt loop */
    _rt_model = _robot->model().clone();

    auto rt_ctx = std::make_shared<Cartesian::Context>(
        std::make_shared<Parameters>(getPeriodSec()),
        _rt_model);

    ProblemDescription ik_problem(ik_yaml, rt_ctx);

    auto impl_name = getParamOr<std::string>("~solver", "OpenSot");

    _rt_ci = CartesianInterfaceImpl::MakeInstance(impl_name, ik_problem, rt_ctx);
    _rt_ci->enableOtg(rt_ctx->params()->getControlPeriod());
    _rt_ci->update(0, 0);

    /* Create model and ci for nrt loop */
    ModelInterface::Ptr nrt_model = ModelInterface::getModel(_robot->getConfigOptions());

    auto nrt_ctx = std::make_shared<Cartesian::Context>(
        std::make_shared<Parameters>(*_rt_ci->getContext()->params()),
        nrt_model);

    _nrt_ci = std::make_shared<LockfreeBufferImpl>(_rt_ci.get(), nrt_ctx);
    _nrt_ci->pushState(_rt_ci.get(), _rt_model.get());
    _nrt_ci->updateState();
    auto nrt_ci = _nrt_ci;

    /*  Create ros api server */
    RosServerClass::Options opt;
    opt.tf_prefix = getParamOr<std::string>("~tf_prefix", "ci");
    opt.ros_namespace = getParamOr<std::string>("~ros_ns", "cartesian");
    auto ros_srv = std::make_shared<RosServerClass>(_nrt_ci, opt);

    /* Initialization */
    _rt_active = false;
    auto rt_active_ptr = &_rt_active;

    _nrt_exit = false;
    auto nrt_exit_ptr = &_nrt_exit;

    _qdot = _q.setZero(_rt_model->getNv());

    /* Spawn thread */
    _nrt_th = std::make_unique<thread>(
        [rt_active_ptr, nrt_exit_ptr, nrt_ci, ros_srv]()
        {
            this_thread::set_name("cartesio_nrt");

            while(!*nrt_exit_ptr)
            {
                this_thread::sleep_for(10ms);

                if(!*rt_active_ptr) continue;

                nrt_ci->updateState();
                ros_srv->run();

            }

        });

    /* Set robot control mode */
    setDefaultControlMode(ControlMode::Effort() + ControlMode::Impedance());
    //setDefaultControlMode(ControlMode::Position() + ControlMode::Effort());
    //setDefaultControlMode(ControlMode::Position());

    /* Feedback */
    _enable_feedback = getParamOr("~enable_feedback", false);

    if(_enable_feedback)
    {
        jinfo("running with feedback enabled \n");
    }

    /* Link state (ground truth) */
    auto lss_vec = _robot->getDevices<Hal::LinkStateSensor>().get_device_vector();
    for(auto d : lss_vec)
    {
        if(d->getLinkName() == "base_link")
        {
            _lss = d;
            jinfo("found ground truth pose and twist for base_link");
        }
    }

    return true;
}

bool CartesioRt::init_load_world_frame()
{
    if(!_rt_model->isFloatingBase())
    {
        return false;
    }

    /* If set_world_from_param is false, and a world_frame_link is defined... */
    std::string world_frame_link = getParamOr<std::string>("~world_frame_link", "");
    if(world_frame_link != "")
    {
        std::string floating_base_link;
        _rt_model->getFloatingBaseLink(floating_base_link);
        if(!_rt_model->getPose(world_frame_link, floating_base_link, _fb_T_l))
        {
            throw std::runtime_error("World frame link '" + world_frame_link + "' is undefined");
        }
        jinfo("Setting world frame in: " + world_frame_link);

        return true;
    }
    return false;
}

void CartesioRt::starting()
{
    // we use a fake time, and integrate it by the expected dt
    _fake_time = 0;

    // align model to current position reference
    _robot->sense(false);
    _rt_model->setJointPosition(_robot->getMotorPosition());
    _rt_model->update();


    if(_lss)
    {
        _lss->sense();

        /* world frame */
        if(init_load_world_frame())
        {
            _l_T_m = _fb_T_l.inverse() * _lss->getPose().inverse();
        }
        else
        {
            _l_T_m.setIdentity();
        }

        _rt_model->setFloatingBasePose(_l_T_m * _lss->getPose());
        _rt_model->setFloatingBaseTwist(_lss->getTwist());
        _rt_model->update();
    }

    // reset ci
    _rt_ci->reset(_fake_time);

    // signal nrt thread that rt is active
    _rt_active = true;

    // setting zero stiffness and damping
    _robot->getStiffness(_initial_stiffness);
    _robot->getDamping(_initial_damping);
    _zeros.setZero(_initial_stiffness.size());
    _robot->setStiffness(_zeros);
    _robot->setDamping(_zeros);
    _robot->move();

    // transit to run
    start_completed();

}

void CartesioRt::run()
{
    /* Receive commands from nrt */
    _nrt_ci->callAvailable(_rt_ci.get());

    /* Update robot */
    if(_enable_feedback)
    {
        _robot->sense(false);
        _rt_model->syncFrom(*_robot);

        if(_lss)
        {
            _lss->sense();

            _rt_model->setFloatingBasePose(_l_T_m * _lss->getPose());
            _rt_model->setFloatingBaseTwist(_lss->getTwist());
            _rt_model->update();
        }
    }

    /* Solve IK */
    if(!_rt_ci->update(_fake_time, getPeriodSec()))
    {
        jerror("unable to solve \n");
        return;
    }

    /* Integrate solution */
    if(!_enable_feedback)
    {
        _qdot = _rt_model->getJointVelocity() * getPeriodSec() + _rt_model->getJointAcceleration()*0.5*getPeriodSec()*getPeriodSec();
        _rt_model->integrateJointPosition(_qdot);
        _rt_model->update();
    }

    _fake_time += getPeriodSec();

    /* Send state to nrt */
    _nrt_ci->pushState(_rt_ci.get(), _rt_model.get());

    /* Move robot */
    if(_enable_feedback)
    {
        _robot->setReferenceFrom(*_rt_model, XBot::ControlMode::EFFORT);
    }
    else
    {
        _robot->setReferenceFrom(*_rt_model);
    }

    _robot->move();
}

void CartesioRt::stopping()
{
    _rt_active = false;

    _robot->setStiffness(_initial_stiffness);
    _robot->setDamping(_initial_damping);
    _robot->setEffortReference(_zeros);
    _robot->move();

    stop_completed();
}

void CartesioRt::on_abort()
{
    _rt_active = false;
    _nrt_exit = true;
}

void CartesioRt::on_close()
{
    _nrt_exit = true;
    jinfo("joining with nrt thread.. \n");
    if(_nrt_th) _nrt_th->join();
}

XBOT2_REGISTER_PLUGIN(CartesioRt,
                      cartesio_plugin);
