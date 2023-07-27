#include <cartesian_interface/utils/AccMaxComputer.h>
#include <random>

using namespace XBot::Cartesian::Utils;

std::pair<QDDotMin, QDDotMax> AccMaxComputer::computeAccMax(XBot::ModelInterface::Ptr model, int configs, double initial_qddotmax)
{
    Eigen::VectorXd qmin, qmax;
    model->getJointLimits(qmin, qmax);

    Eigen::VectorXd qdotmax;
    model->getVelocityLimits(qdotmax);

    Eigen::VectorXd taumax;
    model->getEffortLimits(taumax);

    std::vector<std::uniform_real_distribution<>> qrand, qdotrand;
    for(unsigned int j = 0; j < qmin.size(); ++j)
        qrand.push_back(std::uniform_real_distribution<>(qmin[j], qmax[j]));
    for(unsigned int j = 0; j < qdotmax.size(); ++j)
        qdotrand.push_back(std::uniform_real_distribution<>(-qdotmax[j], qdotmax[j]));

    Eigen::VectorXd q, qdot, h, qddotmax, qddotmin;
    q.setZero(qmin.size());
    qdot.setZero(qdotmax.size());
    h.setZero(qdotmax.size());
    qddotmax.setZero(qdot.size());
    qddotmin.setZero(qdot.size());
    Eigen::MatrixXd M;
    M.setZero(qdotmax.size(), qdotmax.size());

    Eigen::VectorXd min_qddot_max, max_qddot_min;
    min_qddot_max =  Eigen::VectorXd::Constant(qdot.size(), initial_qddotmax);
    max_qddot_min =  Eigen::VectorXd::Constant(qdot.size(), -initial_qddotmax);

    for(unsigned int i = 0; i < configs; ++i)
    {
        std::random_device rd;  // a seed source for the random number engine
        std::mt19937 gen(rd()); // mersenne_twister_engine seeded with rd()

        unsigned int s = 0;
        unsigned int n = q.size();
        if(model->isFloatingBase())
        {
            s = 6;
            n -= s;
        }

        for(unsigned int j = s; j < q.size(); ++j)
        {
            q[j] = qrand[j](gen);
            qdot[j] = qdotrand[j](gen);
        }

        model->setJointPosition(q);
        model->setJointVelocity(qdot);
        model->update();

        model->computeNonlinearTerm(h);
        model->getInertiaMatrix(M);

        for(unsigned int j = s; j < n; ++j)
        {
            qddotmax[j] = (taumax[j] - h[j])/M(j,j);
            qddotmin[j] = (-taumax[j] - h[j])/M(j,j);

            if(qddotmax[j] > 0. && qddotmax[j] < min_qddot_max[j])
                min_qddot_max[j] = qddotmax[j];
            if(qddotmin[j] < 0. && qddotmin[j] > max_qddot_min[j])
                max_qddot_min[j] = qddotmin[j];
        }
    }

    return std::pair<QDDotMin, QDDotMax>(max_qddot_min, min_qddot_max);
}
