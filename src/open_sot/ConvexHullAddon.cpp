#include <cartesian_interface/problem/Task.h>
#include <cartesian_interface/open_sot/TaskInterface.h>
#include <OpenSoT/constraints/velocity/ConvexHull.h>
#include <boost/make_shared.hpp>
#include <visualization_msgs/Marker.h>
#include <ros/node_handle.h>


using namespace XBot::Cartesian;

/* (1) Define struct that contains the description of the task/constraint, 
 * to be parsed from the problem description YAML file 
 */
struct ConvexHull : public ConstraintDescription // inherit from [Constraint/Task]Description!
{
    std::list<std::string> contact_links;
    double safety_margin;
    
    ConvexHull(std::list<std::string> contact_links);
};

ConvexHull::ConvexHull(std::list<std::string> arg_contact_links):
    ConstraintDescription("ConvexHull"),  // task type (specified by user in YAML file)
    contact_links(arg_contact_links),
    safety_margin(.01)
{
}

/* (2) Define a factory function that dynamically allocates the struct defined in (1)
 * given a YAML::Node and an XBot::ModelInterface::ConstPtr.
 * The function must be declared as 'extern "C"' in order to disable mangling.
 * The function must be named #TASKTYPE#[Task/Constraint]DescriptionFactory.
 * This is the place where you parse the yaml node.
 */
extern "C" ConstraintDescription * ConvexHullConstraintDescriptionFactory(YAML::Node task_node, 
                                                                          XBot::ModelInterface::ConstPtr model)
{
    std::list<std::string> contact_links;
    
    if(task_node["contact_links"])
    {
        for(auto cl : task_node["contact_links"])
        {
            contact_links.push_back(cl.as<std::string>());
        }
    }
    else
    {
        throw std::runtime_error("Missing mandatory node 'contact_links' in ConvexHull task");
    }
    
    ConvexHull * task_desc = new ConvexHull(contact_links);
    
    if(task_node["safety_margin"])
    {
        task_desc->safety_margin = task_node["safety_margin"].as<double>();
    }
    
    return task_desc;
    
}

/* (3) Define the corresponding SoT::TaskInterface class. Its purpose is to:
 *  - construct the correct OpenSoT task/constr from the [Task/Constraint]Description defined in (1)
 *  - provide setBaseLink, setControlMode, and update functionalities (if any)
 * The constructor must take two arguments as in this example.
 */
class ConvexHullOpenSot : public SoT::ConstraintInterface
{
    
public:
    
    ConvexHullOpenSot(ConstraintDescription::Ptr constr_desc, 
                      XBot::ModelInterface::ConstPtr model):
        SoT::ConstraintInterface(constr_desc, model),
        _model(model)
    {
        
        auto convexhull_desc = std::dynamic_pointer_cast<ConvexHull>(constr_desc);
        
        Eigen::VectorXd q;
        model->getJointPosition(q);
        
        _constr = boost::make_shared<OpenSoT::constraints::velocity::ConvexHull>
                                (q,
                                 const_cast<XBot::ModelInterface&>(*model), // HACK non-const model required
                                 convexhull_desc->contact_links,
                                 convexhull_desc->safety_margin
                                );
        _vis_pub = _n.advertise<visualization_msgs::Marker>( "convex_hull", 0 );
    }
    
    SoT::ConstraintPtr getConstraintPtr() const override
    {
        return _constr;
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
        /**
          * This part should be done in a dedicated update_ros() in the future
          */
        OpenSoT::constraints::velocity::ConvexHull::Ptr constr = boost::static_pointer_cast<
                OpenSoT::constraints::velocity::ConvexHull>(_constr);
        ch.clear();
        if(constr->getConvexHull(ch))
        {
            ch_marker.header.frame_id = "ci/world_odom";
            ch_marker.header.stamp = ros::Time::now();
            ch_marker.ns = "convex_hull";
            ch_marker.id = 0;
            ch_marker.type = visualization_msgs::Marker::LINE_STRIP;
            ch_marker.action = visualization_msgs::Marker::ADD;

            KDL::Vector com;
            _model->getCOM(com);

            geometry_msgs::Point p;
            ch_marker.points.clear();
            for(std::vector<KDL::Vector>::iterator i =ch.begin(); i != ch.end(); ++i){
                p.x = i->x() + com[0];
                p.y = i->y() + com[1];
                p.z = i->z();// + com[2];

                ch_marker.points.push_back(p);
            }

            p.x = ch.begin()->x() + com[0];
            p.y = ch.begin()->y() + com[1];
            p.z = ch.begin()->z();// + com[2];
            ch_marker.points.push_back(p);

            ch_marker.color.a = 1.0;
            ch_marker.color.r = 0.0;
            ch_marker.color.g = 1.0;
            ch_marker.color.b = 0.0;

            ch_marker.scale.x = 0.01;

            _vis_pub.publish(ch_marker);
        }

        /** **/

        return true;
    }
    
private:
    
    SoT::ConstraintPtr _constr;
    ros::NodeHandle _n;
    ros::Publisher _vis_pub;
    XBot::ModelInterface::ConstPtr _model;
    std::vector<KDL::Vector> ch;
    visualization_msgs::Marker ch_marker;


    
};

/* (4) Define the factory function for the SoT::[Task/Constraint]Interface as well. */
extern "C" SoT::ConstraintInterface * ConvexHullOpenSotConstraintFactory(ConstraintDescription::Ptr task_desc, 
                                                         XBot::ModelInterface::ConstPtr model)
{
    return new ConvexHullOpenSot(task_desc, model);
}
