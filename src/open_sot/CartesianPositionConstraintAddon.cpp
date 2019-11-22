#include <cartesian_interface/problem/Task.h>
#include <cartesian_interface/open_sot/TaskInterface.h>
#include <OpenSoT/constraints/velocity/CartesianPositionConstraint.h>
#include <boost/make_shared.hpp>
#include <visualization_msgs/Marker.h>
#include <ros/node_handle.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <OpenSoT/constraints/GenericConstraint.h>

using namespace XBot::Cartesian;

/* (1) Define struct that contains the description of the task/constraint,
 * to be parsed from the problem description YAML file
 */
struct CartesianPositionConstraint : public ConstraintDescription // inherit from [Constraint/Task]Description!
{
    Eigen::MatrixXd A_Cartesian;
    Eigen::MatrixXd b_Cartesian;
    std::string frame_;

    CartesianPositionConstraint(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, const std::string& frame);
};

CartesianPositionConstraint::CartesianPositionConstraint(const Eigen::MatrixXd& A, const Eigen::VectorXd& b,
                                                         const std::string& frame):
    ConstraintDescription("CartesianPosition"),  // task type (specified by user in YAML file)
    A_Cartesian(A), b_Cartesian(b), frame_(frame)
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
    std::string frame;
    if(task_node["frame"])
        frame = task_node["frame"].as<std::string>();
    else
    {
        throw std::runtime_error("Missing mandatory node 'frame' in CartesianPositionConstraint");
    }


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

    Eigen::VectorXd bC(0);
    if(task_node["b"])
    {
        for(auto plane : task_node["b"])
        {
            std::vector<double> plane_params = plane.as<std::vector<double>>();
            bC = Eigen::Map<Eigen::VectorXd>(plane_params.data(), plane_params.size());
        }
    }
    else
    {
        throw std::runtime_error("Missing mandatory node 'b' in CartesianPositionConstraint");
    }


    if(AC.rows() != bC.size())
        throw std::runtime_error("A.rows() should be equal to b.size() in CartesianPositionConstraint");


    CartesianPositionConstraint * constr_desc = new CartesianPositionConstraint(AC, bC, frame);

    return constr_desc;

}

/* (3) Define the corresponding SoT::TaskInterface class. Its purpose is to:
 *  - construct the correct OpenSoT task/constr from the [Task/Constraint]Description defined in (1)
 *  - provide setBaseLink, setControlMode, and update functionalities (if any)
 * The constructor must take two arguments as in this example.
 */
class CartesianPositionConstraintOpenSot : public SoT::ConstraintInterface
{

public:

    CartesianPositionConstraintOpenSot(ConstraintDescription::Ptr task_desc, XBot::ModelInterface::ConstPtr model):
        SoT::ConstraintInterface(task_desc, model),
        _model(model)
    {
        auto constr_desc = std::dynamic_pointer_cast<CartesianPositionConstraint>(task_desc); //not used in this case


        _AC = constr_desc->A_Cartesian;
        _bC = constr_desc->b_Cartesian;
        _frame = constr_desc->frame_;

        _rviz = std::make_shared<rviz_visual_tools::RvizVisualTools>(_frame);

        Eigen::VectorXd q;
        model->getJointPosition(q);

        Eigen::VectorXd zero = q; zero.setZero();
        boost::make_shared<OpenSoT::constraints::GenericConstraint>("dummy", zero, zero, q.size());


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
        for(unsigned int i = 0; i < _bC.size(); ++i)
            publishABCDPlane(_AC(i,0), _AC(i,1), _AC(i,2), _bC[i]);

        ros::spinOnce();
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

    std::string _frame;

    XBot::ModelInterface::ConstPtr _model;

    std::shared_ptr<rviz_visual_tools::RvizVisualTools> _rviz;

    SoT::ConstraintPtr _constr;

};

/* (4) Define the factory function for the SoT::[Task/Constraint]Interface as well. */
extern "C" SoT::ConstraintInterface * CartesianPositionConstraintOpenSotFactory(ConstraintDescription::Ptr task_desc,
                                                         XBot::ModelInterface::ConstPtr model)
{
    return new CartesianPositionConstraintOpenSot(task_desc, model);
}

