#ifndef CARTESIO_RT_H
#define CARTESIO_RT_H

#include <xbot2/xbot2.h>
#include <xbot2/gazebo/dev_link_state_sensor.h>

#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <cartesian_interface/sdk/rt/LockfreeBufferImpl.h>
#include <cartesian_interface/ros/RosServerClass.h>

namespace XBot {

class CartesioRt : public ControlPlugin
{

public:

    using ControlPlugin::ControlPlugin;

    bool on_initialize() override;
    void starting() override;
    void run() override;
    void stopping() override;
    void on_abort() override;
    void on_close() override;

private:

    std::unique_ptr<ros::NodeHandle> _nh;

    bool _enable_feedback;

    Eigen::VectorXd _q, _qdot;
    ModelInterface::Ptr _rt_model;
    Cartesian::CartesianInterfaceImpl::Ptr _rt_ci;
    Cartesian::LockfreeBufferImpl::Ptr _nrt_ci;
    std::atomic_bool _rt_active;
    std::atomic_bool _nrt_exit;
    double _fake_time;

    std::unique_ptr<thread> _nrt_th;

    Hal::LinkStateSensor::Ptr _lss;
};


}

#endif // CARTESIO_RT_H
