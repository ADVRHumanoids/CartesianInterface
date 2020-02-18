#include <cartesian_interface/problem/Task.h>
#include <cartesian_interface/open_sot/TaskInterface.h>
#include <OpenSoT/constraints/velocity/CartesianPositionConstraint.h>
#include <boost/make_shared.hpp>
#include <visualization_msgs/Marker.h>
#include <ros/node_handle.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <OpenSoT/constraints/GenericConstraint.h>

using namespace XBot::Cartesian;

/**
 * @brief The CartesianPosition Addon
 *
 * Implements a Cartesian position constraint in the form:
 *
 *      Ax <= b     (1)
 *
 *  where A = [a d c] and d defines a plane:
 *
 *      ax + dy + cz -b = 0
 *
 * The definition in the stack.yaml has the form:
 *
 *  CartesianPosition:
        lib_name: "libCartesianPositionConstraintAddon.so"
        type: "CartesianPosition"
        distal_link: "LSoftHand"
        base_link: "Waist"
        A:
        - [1, 0, 0]
        - [-1, 0, 0]
        - [0, 1, 0]
        - [0, -1, 0]
        b: [0.2, 0.1, 0.1, 0.2]
        bound_scaling: 0.09 # early activation
 *
 * Notice that the A matrix and d has to choosen considering that the constraint is in the form (1)
 */

/* (1) Define struct that contains the description of the task/constraint,
 * to be parsed from the problem description YAML file
 */
struct CartesianPosition : public ConstraintDescription // inherit from [Constraint/Task]Description!
{
    Eigen::MatrixXd A_Cartesian;
    Eigen::MatrixXd b_Cartesian;
    std::string base_link_, distal_link_;
    double bound_scaling_;

    /**
     * @brief CartesianPosition constraints the position of a distal link wrt a certain base link
     * @param A [m x 3] matrix ,each row corresponds to a plane
     * @param b [m] vector, each elemnt correspond to a plane
     * @param base_link wrt the plane is specified
     * @param distal_link constrained frame
     * @param bound_scaling early activation of the task
     */
    CartesianPosition(const Eigen::MatrixXd& A, const Eigen::VectorXd& b,
                      const std::string& base_link, const std::string& distal_link,
                      const double bound_scaling);
};

CartesianPosition::CartesianPosition(const Eigen::MatrixXd& A, const Eigen::VectorXd& b,
                                     const std::string& base_link, const std::string& distal_link,
                                     const double bound_scaling):
    ConstraintDescription("CartesianPosition"),  // task type (specified by user in YAML file)
    A_Cartesian(A), b_Cartesian(b), base_link_(base_link), distal_link_(distal_link), bound_scaling_(bound_scaling)
{
}

/* (2) Define a factory function that dynamically allocates the struct defined in (1)
 * given a YAML::Node and an XBot::ModelInterface::ConstPtr.
 * The function must be declared as 'extern "C"' in order to disable mangling.
 * The function must be named #TASKTYPE#[Task/Constraint]DescriptionFactory.
 * This is the place where you parse the yaml node.
 */
extern "C" ConstraintDescription * CartesianPositionConstraintDescriptionFactory(YAML::Node task_node,
                                                               XBot::ModelInterface::ConstPtr model)
{
    std::string distal_link, base_link;
    if(task_node["distal_link"])
    {
        distal_link = task_node["distal_link"].as<std::string>();

        if(task_node["base_link"])
            base_link = task_node["base_link"].as<std::string>();
        else
            base_link = "world";
    }
    else
    {
        throw std::runtime_error("Missing mandatory node 'distal_link' in CartesianPositionConstraint");
    }


    std::cout<<"distal_link: "<<distal_link<<std::endl;
    std::cout<<"base_link: "<<base_link<<std::endl;


    Eigen::MatrixXd AC(0,3);
    if(task_node["A"])
    {
        for(auto plane : task_node["A"])
        {
            std::vector<double> plane_params = plane.as<std::vector<double>>();
            if(plane_params.size() != 3)
                throw std::runtime_error("Plane params size should be 3!");

            AC.conservativeResize(AC.rows() + 1, Eigen::NoChange);
            AC.row(AC.rows()-1) = Eigen::Vector3d(plane_params[0], plane_params[1], plane_params[2]);
        }
    }
    else
    {
        throw std::runtime_error("Missing mandatory node 'A' in CartesianPositionConstraint");
    }
    std::cout<<"AC: \n"<<AC<<std::endl;

    Eigen::VectorXd bC(0);
    if(task_node["b"])
    {
        for(auto param : task_node["b"])
        {
            bC.conservativeResize(bC.size()+1);
            bC[bC.size()-1] = param.as<double>();
        }
    }
    else
    {
        throw std::runtime_error("Missing mandatory node 'b' in CartesianPositionConstraint");
    }
    std::cout<<"bC: \n"<<bC.transpose()<<std::endl;


    if(AC.rows() != bC.size())
        throw std::runtime_error("A.rows() should be equal to b.size() in CartesianPositionConstraint");

    double bound_scaling;
    if(task_node["bound_scaling"])
        bound_scaling = task_node["bound_scaling"].as<double>();
    else
        bound_scaling = 0.1;

    std::cout<<"bound_scaling: "<<bound_scaling<<std::endl;

    CartesianPosition * constr_desc = new CartesianPosition(AC, bC, base_link, distal_link, bound_scaling);

    return constr_desc;

}

/* (3) Define the corresponding SoT::TaskInterface class. Its purpose is to:
 *  - construct the correct OpenSoT task/constr from the [Task/Constraint]Description defined in (1)
 *  - provide setBaseLink, setControlMode, and update functionalities (if any)
 * The constructor must take two arguments as in this example.
 */
class CartesianPositionOpenSot : public SoT::ConstraintInterface
{

public:

    CartesianPositionOpenSot(ConstraintDescription::Ptr task_desc, XBot::ModelInterface::ConstPtr model):
        SoT::ConstraintInterface(task_desc, model),
        _model(model)
    {
        auto constr_desc = std::dynamic_pointer_cast<CartesianPosition>(task_desc);


        _AC = constr_desc->A_Cartesian;
        _bC = constr_desc->b_Cartesian;
        _base_link = constr_desc->base_link_;
        _distal_link = constr_desc->distal_link_;
        _bound_scaling = constr_desc->bound_scaling_;

        Eigen::VectorXd q;
        model->getJointPosition(q);


        //Dummy Cartesian Task
        _dummy_task = boost::make_shared<OpenSoT::tasks::velocity::Cartesian>(
                    "dummy_task", q, const_cast<XBot::ModelInterface&>(*model), _distal_link, _base_link);

        _constr = boost::make_shared<OpenSoT::constraints::velocity::CartesianPositionConstraint>
                                (q, _dummy_task, _AC, _bC, _bound_scaling);


        _base_link = "ci/" + _base_link;
        _distal_link = "ci/" + _distal_link;


        _rviz = std::make_shared<rviz_visual_tools::RvizVisualTools>(_base_link);


        _rviz->enableFrameLocking(true);
        _rviz->setAlpha(0.3);
        _rviz->enableBatchPublishing(true);
        _rviz->loadMarkerPub(true, true);

        for(unsigned int i = 0; i < _bC.size(); ++i)
            publishABCDPlane(_AC(i,0), _AC(i,1), _AC(i,2), -_bC[i], rviz_visual_tools::colors::RED);

        ROS_INFO("Cartesian Position Constraint found");
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
        _rviz->trigger();
        return true;
    }

private:
    /**
       * \brief Display a plane. Vector (A, B, C) gives the normal to the plane.
       *        |D|/|(A,B,C)| gives the distance to plane along that unit normal.
       *        The plane equation used is Ax+By+Cz+D=0.
       * \param A - coefficient from Ax+By+Cz+D=0
       * \param B - coefficient from Ax+By+Cz+D=0
       * \param C - coefficient from Ax+By+Cz+D=0
       * \param D - coefficient from Ax+By+Cz+D=0
       * \param color - the color of the plane
       * \param x_width - X-size of the vizualized plane [meters]
       * \param y_width - Y-size of the visualized plane [meters]
       * \return true on success
       *
       * NB: this has been copied from rviz_visual_tools master branch
       */
      bool publishABCDPlane(const double A, const double B, const double C, const double D,
                            rviz_visual_tools::colors color = rviz_visual_tools::TRANSLUCENT,
                            double x_width = 1.0, double y_width = 1.0)
      {
          // The coefficients A,B,C give the normal to the plane.
            Eigen::Vector3d n(A, B, C);

            // Graphic is centered at this point
            double distance = D / n.norm();
            Eigen::Vector3d center = -distance * n.normalized();

            Eigen::Isometry3d pose;
            pose.translation() = center;

            // Calculate the rotation matrix from the original normal z_0 = (0,0,1) to new normal n = (A,B,C)
            Eigen::Vector3d z_0 = Eigen::Vector3d::UnitZ();
            Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(z_0, n);
            pose.linear() = q.toRotationMatrix();

            double height = 0.001;  // very thin
            _rviz->publishCuboid(pose, x_width, y_width, height, color);

            return true;
      }


    Eigen::MatrixXd _AC;
    Eigen::VectorXd _bC;

    std::string _base_link, _distal_link;

    XBot::ModelInterface::ConstPtr _model;

    std::shared_ptr<rviz_visual_tools::RvizVisualTools> _rviz;

    SoT::ConstraintPtr _constr;

    OpenSoT::tasks::velocity::Cartesian::Ptr _dummy_task;

    double _bound_scaling;

};

/* (4) Define the factory function for the SoT::[Task/Constraint]Interface as well. */
extern "C" SoT::ConstraintInterface * CartesianPositionOpenSotConstraintFactory(ConstraintDescription::Ptr task_desc,
                                                         XBot::ModelInterface::ConstPtr model)
{
    return new CartesianPositionOpenSot(task_desc, model);
}

