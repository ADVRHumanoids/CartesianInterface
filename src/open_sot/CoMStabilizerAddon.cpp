#include <cartesian_interface/problem/Task.h>
#include <cartesian_interface/problem/Cartesian.h>
#include <cartesian_interface/open_sot/TaskInterface.h>
#include <OpenSoT/tasks/velocity/CoMStabilizer.h>
#include <boost/make_shared.hpp>
#include <ros/subscriber.h>
#include <ros/node_handle.h>

#include <geometry_msgs/PoseStamped.h>

using namespace XBot::Cartesian;

/* (1) Define struct that contains the description of the task/constraint,
 * to be parsed from the problem description YAML file
 */
struct CoMStabilizer : public CartesianTask // inherit from [Constraint/Task]Description!
{
    // Stabilizer gains
    Eigen::Vector3d K;
    Eigen::Vector3d D;

    Eigen::Vector2d foot_size;

    // F/T sensors frames names
    std::string ft_sensor_l_sole;
    std::string ft_sensor_r_sole;

    // Controlled feet frames names
    std::string l_sole;
    std::string r_sole;

    // left or right ankle frame name
    std::string ankle;

    // Minimium contact force on Z
    double Fzmin;

    Eigen::Vector3d MaxLims;
    Eigen::Vector3d MinLims;

    double samples2ODE;

    double freq;

    double dT;

    CoMStabilizer(const std::string& ft_sensor_l_sole_, const std::string& ft_sensor_r_sole_,
                  const std::string& l_sole_, const std::string& r_sole_,
                  const std::string& ankle_,
                  const Eigen::Vector2d& foot_size_,
                  const Eigen::Vector3d& K_,
                  const Eigen::Vector3d& D_,
                  const double Fzmin_,
                  const Eigen::Vector3d& MaxLims_,
                  const Eigen::Vector3d& MinLims_,
                  const double samples2ODE_, const double freq_, const double dT_);
};

CoMStabilizer::CoMStabilizer(const std::string& ft_sensor_l_sole_, const std::string& ft_sensor_r_sole_,
                             const std::string& l_sole_, const std::string& r_sole_,
                             const std::string& ankle_,
                             const Eigen::Vector2d& foot_size_,
                             const Eigen::Vector3d& K_,
                             const Eigen::Vector3d& D_,
                             const double Fzmin_,
                             const Eigen::Vector3d& MaxLims_,
                             const Eigen::Vector3d& MinLims_,
                             const double samples2ODE_, const double freq_, const double dT_):
    CartesianTask("com", "world", 3, "CoMStabilizer"),
    ft_sensor_l_sole(ft_sensor_l_sole_),
    ft_sensor_r_sole(ft_sensor_r_sole_),
    l_sole(l_sole_), r_sole(r_sole_),
    ankle(ankle_),
    foot_size(foot_size_),
    K(K_), D(D_), Fzmin(Fzmin_),
    MaxLims(MaxLims_),MinLims(MinLims_),
    samples2ODE(samples2ODE_), freq(freq_), dT(dT_)
   
     
{
}

/* (2) Define a factory function that dynamically allocates the struct defined in (1)
 * given a YAML::Node and an XBot::ModelInterface::ConstPtr.
 * The function must be declared as 'extern "C"' in order to disable mangling.
 * The function must be named #TASKTYPE#[Task/Constraint]DescriptionFactory.
 * This is the place where you parse the yaml node.
 */
extern "C" TaskDescription * CoMStabilizerTaskDescriptionFactory(YAML::Node task_node,
                                                                 XBot::ModelInterface::ConstPtr model)
{
    XBot::Logger::info("CoMStabilizer Initialization\n");

    double lambda = 1.;
    if(task_node["lambda"])
    {
        lambda = task_node["lambda"].as<double>();
    }
    
    Eigen::Vector3d K(0.01, 0.01, 0.0);
    if(task_node["K"])
    {
        int i = 0;
        for(auto cl : task_node["K"])
        {
            K[i] = cl.as<double>();
            i += 1;
            if (i == 2)
                break;
        }
    }
    else
        XBot::Logger::warning("Compliance not set, default values will be used.\n");
    XBot::Logger::info("K: [ %f, %f, %f ]\n", K[0], K[1], K[2]);

    Eigen::Vector3d D(0.0001, 0.0001, 0.0);
    if(task_node["D"])
    {
        int i = 0;
        for(auto cl : task_node["D"])
        {
            D[i] = cl.as<double>();
            i += 1;
            if (i == 2)
                break;
        }
    }
    else
        XBot::Logger::warning("Damping not set, default values will be used.\n");
    XBot::Logger::info("D: [ %f, %f, %f ]\n", D[0], D[1], D[2]);

    Eigen::Vector2d foot_size;
    if(task_node["foot_size"])
    {
        int i = 0;
        for(auto cl : task_node["foot_size"])
        {
            foot_size[i] = cl.as<double>();
            i += 1;
            if (i == 2)
                break;
        }
    }
    else
        throw std::runtime_error("'foot_size' (X and Y) mandatory information is missing");

    std::string l_sole, r_sole;
    if(task_node["l_sole"])
        l_sole = task_node["l_sole"].as<std::string>();
    else
        throw std::runtime_error("'l_sole' string mandatory information is missing");
    if(task_node["r_sole"])
        r_sole = task_node["r_sole"].as<std::string>();
    else
        throw std::runtime_error("'r_sole' string mandatory information is missing");


    std::string ft_sensor_l_sole, ft_sensor_r_sole;
    if(task_node["ft_sensor_l_sole"])
        ft_sensor_l_sole = task_node["ft_sensor_l_sole"].as<std::string>();
    else
        throw std::runtime_error("'ft_sensor_l_sole' string mandatory information is missing");
    if(task_node["ft_sensor_r_sole"])
        ft_sensor_r_sole = task_node["ft_sensor_r_sole"].as<std::string>();
    else
        throw std::runtime_error("'ft_sensor_r_sole' string mandatory information is missing");


    std::string ankle;
    if(task_node["ankle"])
        ankle = task_node["ankle"].as<std::string>();
    else
        throw std::runtime_error("'ankle' string mandatory information is missing");


    double Fzmin = 10.;
    if(task_node["Fzmin"])
        Fzmin = task_node["Fzmin"].as<double>();
    else
        XBot::Logger::warning("Fzmin not set, default values will be used.\n");
    XBot::Logger::info("Fzmin: %f\n", Fzmin);

    double samples2ODE = 50.;
    if(task_node["samples2ODE"])
        samples2ODE = task_node["samples2ODE"].as<double>();
    else
        XBot::Logger::warning("samples2ODE not set, default values will be used.\n");
    XBot::Logger::info("samples2ODE: %f \n", samples2ODE);

    double freq = 10.;
    if(task_node["freq"])
        freq = task_node["freq"].as<double>();
    else
        XBot::Logger::warning("freq not set, default values will be used.\n");
    XBot::Logger::info("freq: %f \n", freq);

    double dT;
    if(task_node["dT"])
        dT = task_node["dT"].as<double>();
    else
        throw std::runtime_error("dT mandatory information is missing\n");


    Eigen::Vector3d MaxLims(0.3, 0.15, 0.001);
    if(task_node["MaxLims"])
    {
        int i = 0;
        for(auto cl : task_node["MaxLims"])
        {
            MaxLims[i] = cl.as<double>();
            i += 1;
            if (i == 3)
                break;
        }
    }
    else
        XBot::Logger::warning("MaxLims not set, default values will be used.\n");
    XBot::Logger::info("MaxLims: [ %f, %f, %f ] \n", MaxLims[0], MaxLims[1], MaxLims[2]);

    Eigen::Vector3d MinLims(-0.2, -0.15, -0.1);
    if(task_node["MinLims"])
    {
        int i = 0;
        for(auto cl : task_node["MinLims"])
        {
            MinLims[i] = cl.as<double>();
            i += 1;
            if (i == 3)
                break;
        }
    }
    else
        XBot::Logger::warning("MinLims not set, default values will be used.\n");
    XBot::Logger::info("MinLims: [ %f, %f, %f ] \n", MinLims[0], MinLims[1], MinLims[2]);


    CoMStabilizer * task_desc = new CoMStabilizer(ft_sensor_l_sole, ft_sensor_r_sole,
                                                  l_sole,  r_sole,
                                                  ankle,
                                                  foot_size,
                                                  K, D,
                                                  Fzmin,
                                                  MaxLims, MinLims,
                                                  samples2ODE,freq, dT);
    task_desc->lambda = lambda;



    return task_desc;

}

/* (3) Define the corresponding SoT::TaskInterface class. Its purpose is to:
 *  - construct the correct OpenSoT task/constr from the [Task/Constraint]Description defined in (1)
 *  - provide setBaseLink, setControlMode, and update functionalities (if any)
 * The constructor must take two arguments as in this example.
 */
class CoMStabilizerOpenSot : public SoT::TaskInterface
{

public:

    CoMStabilizerOpenSot(TaskDescription::Ptr task_desc,
                         XBot::ModelInterface::ConstPtr model):
        SoT::TaskInterface(task_desc, model)
    {

        _comstabilizer_desc = std::dynamic_pointer_cast<CoMStabilizer>(task_desc);

        Eigen::VectorXd q;
        model->getJointPosition(q);

        Affine3d Tlsole, Trsole;
        model->getPose(_comstabilizer_desc->l_sole, Tlsole);
        model->getPose(_comstabilizer_desc->r_sole, Trsole);
       

        XBot::ForceTorqueSensor::ConstPtr ft_sensor_l_sole, ft_sensor_r_sole;
        try 
        {
            ft_sensor_l_sole = model->getForceTorque().at(_comstabilizer_desc->ft_sensor_l_sole);
            ft_sensor_r_sole = model->getForceTorque().at(_comstabilizer_desc->ft_sensor_r_sole);
        }
        catch (std::out_of_range e)
        {
                throw std::runtime_error("Force/Torque sensor does not exist, specify a valid one.");
        }
        
            
        Affine3d Tankles;
        model->getPose(_comstabilizer_desc->l_sole, _comstabilizer_desc->ankle , Tankles);


        _task = boost::make_shared<OpenSoT::tasks::velocity::CoMStabilizer>
                                (q,
                                 const_cast<XBot::ModelInterface&>(*model), // HACK non-const model required
                                 Tlsole, Trsole,
                                 ft_sensor_l_sole,
                                 ft_sensor_r_sole,
                                 _comstabilizer_desc->dT,
                                 model->getMass(),
                                 fabs(Tankles.translation()[2]),
                                 _comstabilizer_desc->foot_size,
                                 _comstabilizer_desc->Fzmin,
                                 _comstabilizer_desc->K, _comstabilizer_desc->D,
                                 _comstabilizer_desc->MaxLims,
                                 _comstabilizer_desc->MinLims,
                                 _comstabilizer_desc->samples2ODE,
                                 _comstabilizer_desc->freq);
                                
        _task->setLambda(_comstabilizer_desc->lambda);
        _zmp_sub = n_CoMStab.subscribe("cartesian/com_stabilizer/zmp/reference", 1, &CoMStabilizerOpenSot::zmp_callback, this); 
        _zmp_ref.setZero();
    }

    SoT::TaskPtr getTaskPtr() const override
    {
        return _task;
    }

    bool update(const CartesianInterface * ci,
                double time,
                double period)
    {
        Eigen::Vector3d com_ref;
        ci->getComPositionReference(com_ref);
        
        Eigen::Affine3d l_sole_ref;
        Eigen::Affine3d r_sole_ref;
        ci->getPoseReference(_comstabilizer_desc->l_sole, l_sole_ref);
        ci->getPoseReference(_comstabilizer_desc->r_sole, r_sole_ref);
        
        _task->setSoleRef(l_sole_ref, r_sole_ref);
        _task->setReference(com_ref);
        _task->setZMP(_zmp_ref);
        
        return true; /*??*/
    }

    bool setControlMode(const std::string& ee_name,
                        ControlType ctrl_mode)
    {
        return false;
    }

    bool setBaseLink(const std::string& ee_name,
                             const std::string& base_link)
    {
        return true;
    }

private:

    OpenSoT::tasks::velocity::CoMStabilizer::Ptr _task;
    
        
    ros::NodeHandle n_CoMStab;
    ros::Subscriber _zmp_sub;
    
    Eigen::Vector2d _zmp_ref;

    std::shared_ptr<CoMStabilizer> _comstabilizer_desc;
    
    void zmp_callback(const geometry_msgs::PoseStamped msg_rcv)
    {
        _zmp_ref << msg_rcv.pose.position.x, msg_rcv.pose.position.y;
    }
    
};

/* (4) Define the factory function for the SoT::[Task/Constraint]Interface as well. */
extern "C" SoT::TaskInterface * CoMStabilizerOpenSotTaskFactory(TaskDescription::Ptr task_desc,
                                                         XBot::ModelInterface::ConstPtr model)
{
    return new CoMStabilizerOpenSot(task_desc, model);
}
