#include <cartesian_interface/trajectory/Trajectory.h>
#include <XBotInterface/Utils.h>

using namespace XBot::Cartesian;

Trajectory::WayPoint::WayPoint()
{
    frame.setIdentity();
    vel.setZero();
    acc.setZero();
    
    time = 0.0;
}


Trajectory::Trajectory()
{
    _frames.reserve(10);
}

Trajectory::WayPoint::WayPoint(Eigen::Affine3d frame, double time):
    WayPoint()
{
    this->frame = frame;
    this->time = time;
}


void Trajectory::addWayPoint(const Trajectory::WayPoint& way_point, double time_offset)
{
    _frames.push_back(way_point);
    _frames.back().time += time_offset;
    
    sort_frames();
}

void Trajectory::addWayPoint(double time, const Eigen::Affine3d& frame)
{
    WayPoint wp;
    wp.frame = frame;
    wp.time = time;
    
    addWayPoint(wp);
}

void Trajectory::clear()
{
    _frames.clear();
}

const std::vector< Trajectory::WayPoint >& Trajectory::getWayPoints() const
{
    return _frames;
}

bool Trajectory::isTrajectoryEnded(double time)
{
    return time > _frames.back().time;
}

void Trajectory::sort_frames()
{
    std::sort(_frames.begin(), _frames.end(), [](const WayPoint& w1, const WayPoint& w2){ return w1.time < w2.time; });
}

Eigen::Affine3d Trajectory::evaluate(double time,
                                     Eigen::Vector6d * const vel,
                                     Eigen::Vector6d * const acc)
{
    /* Find relevant segment (first frame after time) */
    auto it = std::find_if(_frames.begin(),
                           _frames.end(),
                           [&time](const WayPoint& wp){ return wp.time > time; });
    
    if(it == _frames.begin()) // trajectory yet to start
    {
        if(vel) vel->setZero();
        if(acc) acc->setZero();
        return it->frame;
    }
    
    if(it == _frames.end()) // no frames after time (traj is finished)
    {
        if(vel) vel->setZero();
        if(acc) acc->setZero();
        return (--it)->frame;
    }
    
    Eigen::Affine3d end = it->frame;
    Eigen::Affine3d start = (it-1)->frame;
    double t_end = it->time;
    double t_start = (it-1)->time;

    Eigen::Quaterniond q_start(start.linear());
    Eigen::Quaterniond q_end(end.linear());
    
    double tau, dtau, ddtau;
    XBot::Utils::FifthOrderPlanning(0, 0, 0, 1, t_start, t_end, time, tau, dtau, ddtau);
    
    Eigen::Affine3d interpolated;
    interpolated.setIdentity();
    interpolated.linear() = q_start.slerp(tau, q_end).toRotationMatrix();
    interpolated.translation() = (1 - tau)*start.translation() + tau*end.translation();
    
    if(vel)
    {
        vel->setZero();
        vel->head<3>() = (- dtau)*start.translation() + dtau*end.translation();
    }

    if(acc)
    {
        acc->setZero();
        acc->head<3>() = (- ddtau)*start.translation() + ddtau*end.translation();
    }
    
    return interpolated;
    
    
}

int Trajectory::getCurrentSegmentId(double time) const
{
    if(_frames.empty())
    {
        return -1;
    }

    /* Find relevant segment (first frame after time) */
    auto it = std::find_if(_frames.begin(),
                           _frames.end(),
                           [&time](const WayPoint& wp){ return wp.time > time; });

    int id = std::distance(_frames.begin(), it) - 1;

    return id;

}


void XBot::Cartesian::Trajectory::compute()
{

}







