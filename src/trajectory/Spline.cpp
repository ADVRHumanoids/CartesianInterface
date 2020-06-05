#include <cartesian_interface/trajectory/Spline.h>
#include <alglib/interpolation.h>

using namespace XBot::Cartesian;


Spline::Spline():
    _splines_pos(3),
    _splines_rot(4),
    _knots(3),
    _values_pos(3),
    _values_rot(4)
{
    _knots.reserve(10);
    
    for(int i = 0; i < 3; i++)
    {
        _values_pos[i].reserve(10);
    }
    
    for(int i = 0; i < 4; i++)
    {
        _values_rot[i].reserve(10);
    }
}

void Spline::add_knot(double t, const Eigen::Affine3d& T)
{
    _knots.push_back(t);
    
    Eigen::Vector4d q_coeffs = Eigen::Quaterniond(T.linear()).coeffs();
    
    if(_knots.size() > 1)
    {

        Eigen::Vector4d q_last;
        
        for(int i : {0,1,2,3})
            q_last[i] = _values_rot[i][_knots.size()-2];
        
        if(q_last.dot(q_coeffs) < 0)
        {
            q_coeffs = -q_coeffs;
        }
    }
    
    
    for(int i = 0; i < 3; i++)
    {
        _values_pos[i].push_back(T.translation()[i]);
    }
    
    for(int i = 0; i < 4; i++)
    {
        _values_rot[i].push_back(q_coeffs[i]);
    }
    
}

void Spline::clear_all()
{
    
    _knots.clear();
    
    for(int i = 0; i < 3; i++)
    {
        _values_pos[i].clear();
    }
    
    for(int i = 0; i < 4; i++)
    {
        _values_rot[i].clear();
    }
}


void Spline::compute()
{
    clear_all();
    
    for(const WayPoint& wp : getWayPoints())
    {
        add_knot(wp.time, wp.frame);
    }

    alglib::real_1d_array tk_alg;
    tk_alg.attach_to_ptr(_knots.size(), _knots.data());
    
    for(int i = 0; i < 3; i++)
    {
        alglib::real_1d_array y_alg;
        
        y_alg.attach_to_ptr(_values_pos[i].size(), _values_pos[i].data());
        alglib::spline1dbuildcubic(tk_alg, y_alg, _knots.size(), 1, 0, 1, 0, _splines_pos[i]);
    }
    
    for(int i = 0; i < 4; i++)
    {
        alglib::real_1d_array y_alg;
        
        y_alg.attach_to_ptr(_values_rot[i].size(), _values_rot[i].data());
        alglib::spline1dbuildcubic(tk_alg, y_alg, _knots.size(), 1, 0, 1, 0, _splines_rot[i]);
    }
}


Eigen::Affine3d Spline::evaluate(double time, Eigen::Vector6d * const vel, Eigen::Vector6d * const acc)
{
    
    if(time <= getWayPoints()[0].time)
    {
        return getWayPoints()[0].frame;
    }
    
    if(time >= getWayPoints().back().time)
    {
        return getWayPoints().back().frame;
    }
    
    Eigen::Vector3d pos;
    Eigen::Vector4d q_coeffs;
    
    for(int i = 0; i < 3; i++)
    {
        pos[i] = alglib::spline1dcalc(_splines_pos[i], time);
    }
    
    for(int i = 0; i < 4; i++)
    {
        q_coeffs[i] = alglib::spline1dcalc(_splines_rot[i], time);
    }
    
    q_coeffs /= q_coeffs.norm();
    
    Eigen::Affine3d T;
    Eigen::Quaterniond q(q_coeffs);
    T.setIdentity();
    T.translation() = pos;
    T.linear() = q.toRotationMatrix();
    
    return T;
}

Spline::~Spline() = default;
