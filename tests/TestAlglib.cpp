#include <alglib/interpolation.h>
#include <Eigen/Dense>
#include <XBotInterface/MatLogger.hpp>
#include <cartesian_interface/trajectory/Spline.h>

int main()
{
    auto logger = XBot::MatLogger::getLogger("/tmp/alglib_test");
    
    Eigen::VectorXd x(5), y(5);
    x << 1, 2, 3, 4, 5;
    y.setRandom(5);
    
    logger->log("xk", x);
    logger->log("yk", y);
    
    alglib::real_1d_array x_alg, y_alg;
    x_alg.attach_to_ptr(x.size(), x.data());
    y_alg.attach_to_ptr(y.size(), y.data());
    
    alglib::spline1dinterpolant spline;
    alglib::spline1dbuildcubic(x_alg, y_alg, 5, 1, 0, 1, 0, spline);
    
    
    for(double x_val = 0; x_val < 6; x_val += 0.001)
    {
        double y_val = alglib::spline1dcalc(spline, x_val);
        logger->add("x", x_val);
        logger->add("y", y_val);
    }
    
    
    
    
    logger->flush();
    
    return 0;
    
}