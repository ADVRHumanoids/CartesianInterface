#include <cartesian_interface/problem/Task.h>
#include <cartesian_interface/open_sot/TaskInterface.h>
#include <OpenSoT/tasks/velocity/AngularMomentum.h>
#include <boost/make_shared.hpp>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <geometry_msgs/Vector3Stamped.h>

using namespace XBot::Cartesian;

/* (1) Define struct that contains the description of the task/constraint,
 * to be parsed from the problem description YAML file
 */
struct AngularMomentum : public TaskDescription // inherit from [Constraint/Task]Description!
{
    AngularMomentum();
};

AngularMomentum::AngularMomentum():
    TaskDescription(TaskInterface::None, "AngularMomentum", 3)  // task type (specified by user in YAML file)
{
}

/* (2) Define a factory function that dynamically allocates the struct defined in (1)
 * given a YAML::Node and an XBot::ModelInterface::ConstPtr.
 * The function must be declared as 'extern "C"' in order to disable mangling.
 * The function must be named #TASKTYPE#[Task/Constraint]DescriptionFactory.
 * This is the place where you parse the yaml node.
 */
extern "C" TaskDescription * AngularMomentumTaskDescriptionFactory(YAML::Node task_node,
                                                                   XBot::ModelInterface::ConstPtr model)
{
    AngularMomentum * task_desc = new AngularMomentum();

    return task_desc;
}

/* (3) Define the corresponding SoT::TaskInterface class. Its purpose is to:
 *  - construct the correct OpenSoT task/constr from the [Task/Constraint]Description defined in (1)
 *  - provide setBaseLink, setControlMode, and update functionalities (if any)
 * The constructor must take two arguments as in this example.
 */
class AngularMomentumOpenSot : public SoT::TaskInterface
{

public:

    AngularMomentumOpenSot(TaskDescription::Ptr task_desc, XBot::ModelInterface::ConstPtr model):
        SoT::TaskInterface(task_desc, model),
        _model(model)
    {
        auto angularmom_desc = std::dynamic_pointer_cast<AngularMomentum>(task_desc); //not used in this case

        Eigen::VectorXd q;
        model->getJointPosition(q);

        _task = boost::make_shared<OpenSoT::tasks::velocity::AngularMomentum>
                                (q, const_cast<XBot::ModelInterface&>(*model)); // HACK non-const model required

        _angular_momentum_reference_sub = _nh.subscribe("cartesian/angular_momentum/reference",
                                                        100, &AngularMomentumOpenSot::setReference, this);

        _angular_momentum_pub = _nh.advertise<geometry_msgs::Vector3Stamped>("cartesian/angular_momentum", 100);

        _period = 0.0;

        ROS_INFO("AngularMomentum Task found");
    }

    SoT::TaskPtr getTaskPtr() const override
    {
        return _task;
    }

    bool setBaseLink(const std::string & ee_name, const std::string & base_link) override
    {
        return false;
    }

    bool setControlMode(const std::string & ee_name, ControlType ctrl_mode) override
    {
        return false;
    }

    bool update(const CartesianInterface * ci, double time, double period) override
    {
        _period = period;

        Eigen::Vector6d centroidal_mom;
        _model->getCentroidalMomentum(centroidal_mom);


        geometry_msgs::Vector3Stamped angular_mom;
        angular_mom.vector.x = centroidal_mom[3];
        angular_mom.vector.y = centroidal_mom[4];
        angular_mom.vector.z = centroidal_mom[5];
        angular_mom.header.stamp = ros::Time::now();
        angular_mom.header.frame_id = "ci/world";

        _angular_momentum_pub.publish(angular_mom);

        ros::spinOnce();
        return true;
    }

private:
    /**
     * @brief setReference is used to get references for the angular momentum task from outside
     * @param msg desider angular momentum [kg*m*m/s]
     */
    void setReference(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
    {
        Eigen::Vector3d angular_mom_ref;
        angular_mom_ref[0] = msg->vector.x;
        angular_mom_ref[1] = msg->vector.y;
        angular_mom_ref[2] = msg->vector.z;

        boost::dynamic_pointer_cast<OpenSoT::tasks::velocity::AngularMomentum>(_task)->setReference(angular_mom_ref*_period);
    }

    SoT::TaskPtr _task;
    XBot::ModelInterface::ConstPtr _model;
    ros::NodeHandle _nh;
    ros::Subscriber _angular_momentum_reference_sub;
    double _period; //period is required since the

    ros::Publisher _angular_momentum_pub;

};

/* (4) Define the factory function for the SoT::[Task/Constraint]Interface as well. */
extern "C" SoT::TaskInterface * AngularMomentumOpenSotTaskFactory(TaskDescription::Ptr task_desc,
                                                                  XBot::ModelInterface::ConstPtr model)
{
    return new AngularMomentumOpenSot(task_desc, model);
}

