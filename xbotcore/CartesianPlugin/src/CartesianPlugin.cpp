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

#include <time.h>

#include <CartesianPlugin/CartesianPlugin.h>
#include <cartesian_interface/open_sot/OpenSotImpl.h>

/* Specify that the class XBotPlugin::CartesianPlugin is a XBot RT plugin with name "CartesianPlugin" */
REGISTER_XBOT_PLUGIN_(XBot::Cartesian::CartesianPlugin)

namespace XBot { namespace Cartesian {
    


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
    
    /* Initialize a logger which saves to the specified file. Remember that
     * the current date/time is always appended to the provided filename,
     * so that logs do not overwrite each other. */

    _logger = XBot::MatLogger::getLogger("/tmp/CartesianPlugin_log");

    /* Construct CI objects */
    YAML::Node config = YAML::LoadFile(handle->getPathToConfigFile());
    ProblemDescription ik_problem(config["CartesianInterface"]["problem_description"], _model);
    
    std::string impl_name("OpenSot");
    std::string path_to_shared_lib = XBot::Utils::FindLib("libCartesian" + impl_name + ".so", "LD_LIBRARY_PATH");
    if (path_to_shared_lib == "") {
        throw std::runtime_error("libCartesian" + impl_name + ".so must be listed inside LD_LIBRARY_PATH");
    }
    
    _ci  = SoLib::getFactoryWithArgs<XBot::Cartesian::CartesianInterfaceImpl>(path_to_shared_lib, 
                                                                              impl_name + "Impl", 
                                                                              _model, ik_problem);
    _ci->enableOtg(0.001);
    _ci->update(0,0);
    
    /* Helper for RT <-> nRT communication */
    _sync = std::make_shared<Utils::SyncFromIO>(handle, _ci, _model);
    
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
    
    _sync->set_solver_active(true);
    
    _model->syncFrom(*_robot, Sync::Position, Sync::MotorSide);
    _ci->reset(time);
    
}

void CartesianPlugin::on_stop(double time)
{
    /* This function is called on plugin stop, i.e. when the stop command
     * is sent over the plugin switch port (e.g. 'rosservice call /CartesianPlugin_switch false').
     * Since this function is called within the real-time loop, you should not perform
     * operations that are not rt-safe. */
    
    _sync->set_solver_active(false);
}


void CartesianPlugin::control_loop(double time, double period)
{
    /* This function is called on every control loop from when the plugin is start until
     * it is stopped.
     * Since this function is called within the real-time loop, you should not perform
     * operations that are not rt-safe. */
    
    /* Callbacks from NRT */
    _sync->receive(_ci);
    
    /* Update model with sensor reading */
    _model->syncFrom(*_robot, Sync::Sensors, Sync::Effort);

    /* Solve IK */
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
    
    /* Send state to nrt */
    _sync->send(_ci, _model);
    
    /* Send command to robot */
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
