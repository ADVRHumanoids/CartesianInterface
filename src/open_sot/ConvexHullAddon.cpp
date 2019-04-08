#include <cartesian_interface/problem/Task.h>
#include <cartesian_interface/open_sot/TaskInterface.h>
#include <OpenSoT/constraints/velocity/ConvexHull.h>
#include <boost/make_shared.hpp>

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
        SoT::ConstraintInterface(constr_desc, model)
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
        return true;
    }
    
private:
    
    SoT::ConstraintPtr _constr;
    
    
};

/* (4) Define the factory function for the SoT::[Task/Constraint]Interface as well. */
extern "C" SoT::ConstraintInterface * ConvexHullOpenSotConstraintFactory(ConstraintDescription::Ptr task_desc, 
                                                         XBot::ModelInterface::ConstPtr model)
{
    return new ConvexHullOpenSot(task_desc, model);
}
