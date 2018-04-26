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
    


bool CartesianPlugin::init_control_plugin(XBot::Handle::Ptr handle)
{
    /* This function is called outside the real time loop, so we can
     * allocate memory on the heap, print stuff, ...
     * The RT plugin will be executed only if this init function returns true. */


    /* Save robot to a private member. */
    _robot = handle->getRobotInterface();

    /* Initialize a logger which saves to the specified file. Remember that
     * the current date/time is always appended to the provided filename,
     * so that logs do not overwrite each other. */

    _logger = XBot::MatLogger::getLogger("/tmp/CartesianPlugin_log");
    
    _ci_shobj = handle->getSharedMemory()->getSharedObject<CartesianInterfaceImpl::Ptr>("/xbotcore/cartesian_interface");
    
    

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
    
    while(!_ci)
    {
        XBot::Logger::info(Logger::Severity::HIGH, "Trying to get CI pointer.. \n");
        
        _ci = _ci_shobj.get();
        
        usleep(2e5);
    }
    
    _model = _ci->getModel();
    
    XBot::Logger::success(Logger::Severity::HIGH, "Got CI pointer with address %p \n", _ci.get());
    XBot::Logger::success(Logger::Severity::HIGH, "Got Model pointer with address %p \n", _model.get());
    
    
    /* MODIFY MODEL AND CI INSIDE CRITICAL SECTION */
    
    _ci_shobj.get_mutex()->lock();
    
    _model->syncFrom(*_robot);
    
    _ci->reset();
    
    _ci_shobj.get_mutex()->unlock();
    
    /* END CRITICAL SECTION */
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
    
    /* TRY TO ACQUIRE MUTEX, IF UNABLE SKIP THE LOOP */
    
    if(!_ci_shobj.get_mutex()->try_lock())
    {
        _logger->add("skip_loop", 1);
        return;
    }
    
    /* WE LOCKED THE MUTEX -> ENTER CRITICAL SECTION */

    _logger->add("skip_loop", 0);
    
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
    
    _robot->setReferenceFrom(*_model);
    _robot->move();
    
    _ci_shobj.get_mutex()->unlock();
    
    /* END CRITICAL SECTION */

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
