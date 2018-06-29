/*
 * Copyright (C) 2017 IIT-ADVR
 * Author:
 * email:
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/

#include <CartesianPlugin/CartesianPlugin.h>
#include <cartesian_interface/open_sot/OpenSotImpl.h>

/* Specify that the class XBotPlugin::CartesianPlugin is a XBot RT plugin with name "CartesianPlugin" */
REGISTER_XBOT_PLUGIN_(XBot::Cartesian::CartesianPlugin)

namespace XBot { namespace Cartesian {
    
Eigen::Vector3d getGains(const double x, const double y, const double z)
{
    Eigen::Vector3d tmp;
    tmp<<x,y,z;
    return tmp;
}

bool CartesianPlugin::init_control_plugin(XBot::Handle::Ptr handle)
{
    /* This function is called outside the real time loop, so we can
     * allocate memory on the heap, print stuff, ...
     * The RT plugin will be executed only if this init function returns true. */


    /* Save robot to a private member. */
    _robot = handle->getRobotInterface();
    _model = ModelInterface::getModel(handle->getPathToConfigFile());
    
    Eigen::VectorXd qhome;
    _model->getRobotState("home", qhome);
    _model->setJointPosition(qhome);
    _model->update();
    
    _q.resize(_model->getJointNum());
    _qdot = _q;
    
    YAML::Node yaml_file = YAML::LoadFile(handle->getPathToConfigFile());
    ProblemDescription ik_problem(yaml_file["CartesianInterface"]["problem_description"], _model);
    _ci = std::make_shared<OpenSotImpl>(_model, ik_problem);

    /* Initialize a logger which saves to the specified file. Remember that
     * the current date/time is always appended to the provided filename,
     * so that logs do not overwrite each other. */

    _logger = XBot::MatLogger::getLogger("/tmp/CartesianPlugin_log");
    
    _sync_from_nrt = std::make_shared<Utils::SyncFromIO>("/xbotcore/cartesian_interface", handle->getSharedMemory());

    ///STABILIZER
    double dT = 0.001;
    Eigen::Affine3d ankle;
    _model->getPose("l_ankle", "l_sole", ankle);
    Eigen::Vector2d foot_size;
    foot_size<<0.2,0.1;
    double Fzmin = 10.;
    _stabilizer.reset(new CompliantStabilizer(dT, _model->getMass(), fabs(ankle(2,3)),
                                              foot_size, Fzmin,
                                              getGains(0.1,0.1,0.), getGains(-0.005,-0.005,0.),
                                              getGains(DEFAULT_MaxLimsx, DEFAULT_MaxLimsy, DEFAULT_MaxLimsz),
                                              getGains(DEFAULT_MinLimsx, DEFAULT_MinLimsy, DEFAULT_MinLimsz)));
    ///


    return true;


}

void CartesianPlugin::on_start(double time)
{
    /* This function is called on plugin start, i.e. when the start command
     * is sent over the plugin switch port (e.g. 'rosservice call /CartesianPlugin_switch true').
     * Since this function is called within the real-time loop, you should not perform
     * operations that are not rt-safe. */

    /* Save the robot starting config to a class member */
    _start_time = time;
    
    _first_sync_done = false;
    
    _model->syncFrom(*_robot, Sync::Position, Sync::MotorSide);

    ///STABILIZER
    _ci->getComPositionReference(_com_ref);
}

void CartesianPlugin::on_stop(double time)
{
    /* This function is called on plugin stop, i.e. when the stop command
     * is sent over the plugin switch port (e.g. 'rosservice call /CartesianPlugin_switch false').
     * Since this function is called within the real-time loop, you should not perform
     * operations that are not rt-safe. */
}


void CartesianPlugin::control_loop(double time, double period)
{
    /* This function is called on every control loop from when the plugin is start until
     * it is stopped.
     * Since this function is called within the real-time loop, you should not perform
     * operations that are not rt-safe. */
    
    /* Try to update references from NRT */
    if(!_first_sync_done)
    {
        if(_sync_from_nrt->try_reset(_model, time))
        {
            _first_sync_done = true;
            XBot::Logger::info(Logger::Severity::HIGH, "Resetting NRT CI \n");
        }
    }
    
    if(_first_sync_done)
    {
        if(_sync_from_nrt->try_sync(time, period, _ci, _model))
        {
            _logger->add("sync_done", 1);
        }
        else
        {
            _logger->add("sync_done", 0);
        }
    }

    ///STABILIZER
    Eigen::Vector3d zmp_ref = _com_ref; //HERE WE SUPPOSE THAT THE COM DOES NOT MOVE, THEREFORE COM = ZMP;
    Eigen::Affine3d lsole;
    _ci->getCurrentPose("l_sole", lsole);
    Eigen::Affine3d rsole;
    _ci->getCurrentPose("r_sole", rsole);

    Eigen::Vector2d CopPos_L, CopPos_R;
    CopPos_L(0) = zmp_ref[0] - lsole.matrix().col(3).head(3)[0];
    CopPos_L(1) = zmp_ref[1] - lsole.matrix().col(3).head(3)[1];

    CopPos_R(0) = zmp_ref[0] - rsole.matrix().col(3).head(3)[0];
    CopPos_R(1) = zmp_ref[1] - rsole.matrix().col(3).head(3)[1];

    Eigen::Vector6d left_wrench;
    _robot->getForceTorque().at("l_leg_ft")->getWrench(left_wrench);
    Eigen::Vector6d right_wrench;
    _robot->getForceTorque().at("r_leg_ft")->getWrench(right_wrench);

    Eigen::Vector3d delta_com = _stabilizer->update(left_wrench, right_wrench,
                                                   CopPos_L, CopPos_R,
                                                   lsole.matrix().col(3).head(3), rsole.matrix().col(3).head(3));

    _ci->setComPositionReference(_com_ref + delta_com);
    ///
    

    if(!_ci->update(time, period))
    {
        XBot::Logger::error("CartesianInterface: unable to solve \n");
        return;
    }
    
    /* Integrate solution */
    _model->getJointPosition(_q);
    _model->getJointVelocity(_qdot);
    
    _q += period * _qdot;
    
    _model->setJointPosition(_q);
    _model->update();
    
    _robot->setReferenceFrom(*_model, Sync::Position);
    _robot->move();

}

bool CartesianPlugin::close()
{
    /* This function is called exactly once, at the end of the experiment.
     * It can be used to do some clean-up, or to save logging data to disk. */

    /* Save logged data to disk */
    _logger->flush();

    return true;
}

CartesianPlugin::~CartesianPlugin()
{
  
}

} }
