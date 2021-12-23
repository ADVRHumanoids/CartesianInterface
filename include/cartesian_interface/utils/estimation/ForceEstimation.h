#ifndef __CARTESIAN_INTERFACE_UTILS_FORCE_EST_H__
#define __CARTESIAN_INTERFACE_UTILS_FORCE_EST_H__

#include <XBotInterface/ModelInterface.h>
#include <algorithm>
#include <cartesian_interface/Macro.h>
#include <matlogger2/matlogger2.h>

namespace XBot { namespace Cartesian { namespace Utils {

class ForceEstimation
{

public:

    CARTESIO_DECLARE_SMART_PTR(ForceEstimation)

    static const double DEFAULT_SVD_THRESHOLD;

    /**
     * @brief ForceEstimation constructor.
     * @param model: shared pointer to ModelInterface; client code must keep this model up to date
     * with respect to the robot state
     * @param svd_threshold: threshold for solution regularization (close to singularities)
     */
    ForceEstimation(ModelInterface::ConstPtr model,
                    double svd_threshold = DEFAULT_SVD_THRESHOLD);

    /**
    * @brief The add_link method adds one link to the list of estimated forces.
    * @param name: the name of the link
    * @param dofs: force indices to be estimated (e.g. {0,1,2} for just linear force)
    * @param chains: chain names whose joints are used for the estimation (e.g. {"left_arm"})
    * @return a shared pointer to a virtual force-torque sensor, which is updated during the
    * call to update(); use getWrench() on it to retrieve the estimated wrench.
    */
    ForceTorqueSensor::ConstPtr add_link(std::string name,
                                         std::vector<int> dofs = {},
                                         std::vector<std::string> chains = {});

    /**
     * @brief ignore measuements from the provided joints when computing
     * the force estimate
     */
    void setIgnoredJoint(const std::string& jname);

    /**
    * @brief update computes the estimation and updates all registered virtual FT-sensors
    */
    void update();

    void log(MatLogger2::Ptr logger) const;

protected:

    ModelInterface::ConstPtr _model;

private:

    void compute_A_b();
    void solve();
    virtual void compute_residual(Eigen::VectorXd& res);

    struct TaskInfo
    {
        ForceTorqueSensor::Ptr sensor;
        std::vector<int> dofs;
        std::string link_name;

    };

    std::set<int> _ignore_idx;

    Eigen::MatrixXd _Jtot;
    Eigen::MatrixXd _A;
    Eigen::MatrixXd _Jtmp;

    Eigen::VectorXd _y, _tau, _g, _b, _sol;

    std::vector<TaskInfo> _tasks;
    std::set<int> _meas_idx;
    int _ndofs;

    Eigen::JacobiSVD<Eigen::MatrixXd> _svd;
    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> _qr;

};

class ForceEstimationMomentumBased : public ForceEstimation
{

public:

    CARTESIO_DECLARE_SMART_PTR(ForceEstimationMomentumBased);

    static constexpr double DEFAULT_OBS_BW = 4.0;

    ForceEstimationMomentumBased(ModelInterface::ConstPtr model,
                                 double rate,
                                 double svd_threshold = DEFAULT_SVD_THRESHOLD,
                                 double obs_bw = DEFAULT_OBS_BW);

    bool getResiduals(Eigen::VectorXd &res) const;

private:

    void compute_residual(Eigen::VectorXd& res) override;
    void init_momentum_obs();

    double _rate, _k_obs;

    Eigen::VectorXd _y, _tau, _g, _b, _sol;
    Eigen::VectorXd _p0, _p1, _p2, _q, _qdot, _q_old, _h, _coriolis, _y_static;
    Eigen::MatrixXd _M, _M_old, _Mdot;
};



} } }




#endif
