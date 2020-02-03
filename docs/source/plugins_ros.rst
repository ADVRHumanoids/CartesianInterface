Implementing the ROS API for a custom task
==========================================

Easy!

.. code-block:: c++


    #ifndef ANGULARMOMENTUMROS_H
    #define ANGULARMOMENTUMROS_H

    #include "AngularMomentum.h"
    #include <cartesian_interface/sdk/ros/server_api/TaskRos.h>
    #include <geometry_msgs/Vector3Stamped.h>

    namespace XBot { namespace Cartesian {

    namespace ServerApi
    {
    class AngularMomentumRos;
    }

    /**
     * @brief The ServerApi::AngularMomentumRos class implements a ROS
     * interface for the task.
     */
    class ServerApi::AngularMomentumRos : public ServerApi::TaskRos
    {

    public:

        CARTESIO_DECLARE_SMART_PTR(AngularMomentumRos)

        AngularMomentumRos(TaskDescription::Ptr task,
                           ModelInterface::ConstPtr model);

        void run(ros::Time time) override;


    private:

        void on_ref_recv(geometry_msgs::Vector3StampedConstPtr msg);

        ros::Subscriber _ref_sub;
        ros::Publisher _cur_ref_pub;

        AngularMomentum::Ptr _ci_angmom;


    };

    } }

    #endif // ANGULARMOMENTUMROS_H



.. code-block:: c++

    #include "AngularMomentumRos.h"
    #include <eigen_conversions/eigen_msg.h>
    #include "fmt/format.h"

    using namespace XBot::Cartesian;
    using namespace XBot::Cartesian::ServerApi;

    AngularMomentumRos::AngularMomentumRos(TaskDescription::Ptr task,
                                           XBot::ModelInterface::ConstPtr model):
        TaskRos(task, model)
    {
        /* Type cast to the required type, and throw on failure */
        _ci_angmom = std::dynamic_pointer_cast<AngularMomentum>(task);
        if(!_ci_angmom) throw std::runtime_error("Provided task description "
                                                 "does not have expected type 'AngularMomentum'");

        /* Open topics */
        _ref_sub = _ctx.nh().subscribe(task->getName() + "/reference", 1,
                                       &AngularMomentumRos::on_ref_recv, this);

        _cur_ref_pub = _ctx.nh().advertise<geometry_msgs::Vector3Stamped>(task->getName() + "/current_reference",
                                                                      1);
        /* Register type name */
        registerType("AngularMomentum");
    }

    void AngularMomentumRos::run(ros::Time time)
    {
        geometry_msgs::Vector3Stamped msg;
        msg.header.stamp = time;

        tf::vectorEigenToMsg(_ci_angmom->getReference(),
                             msg.vector);

        _cur_ref_pub.publish(msg);
    }

    void AngularMomentumRos::on_ref_recv(geometry_msgs::Vector3StampedConstPtr msg)
    {
        Eigen::Vector3d ref;
        tf::vectorMsgToEigen(msg->vector, ref);

        _ci_angmom->setReference(ref);
    }

    CARTESIO_REGISTER_ROS_API_PLUGIN(AngularMomentumRos)

