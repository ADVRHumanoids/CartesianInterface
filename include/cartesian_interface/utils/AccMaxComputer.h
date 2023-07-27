#ifndef __CI_ACCMAXCOMPUTER_H__
#define __CI_ACCMAXCOMPUTER_H__

#include <XBotInterface/ModelInterface.h>

namespace XBot { namespace Cartesian { namespace Utils{

typedef Eigen::VectorXd QDDotMin;
typedef Eigen::VectorXd QDDotMax;

class AccMaxComputer
{
public:
    /**
     * @brief computeAccMax computes an estimation of the maximum joint accelerations based on maximum joint velocities
     * and torques. It is based on the procedure described in "Joint Position and Velocity Bounds in Discrete-Time
     * Acceleration/Torque Control of Robot Manipulators" by Andrea del Prete
     * @param model of the robot
     * @param configs number of random configs used
     * @param initial_qddotmax initial value for maximum joint accelerations
     * @return pair of (qddotmin, qddotmax)
     */
    static std::pair<QDDotMin, QDDotMax> computeAccMax(XBot::ModelInterface::Ptr model,
                                                       int configs = 1e4,
                                                       double initial_qddotmax = 1e4);
};


}}}

#endif

