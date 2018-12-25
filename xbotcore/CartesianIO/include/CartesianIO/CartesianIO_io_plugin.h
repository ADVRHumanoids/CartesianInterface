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

#ifndef CartesianIO_IOPLUGIN_H_
#define CartesianIO_IOPLUGIN_H_

#include <atomic>

#include <XCM/IOPlugin.h>
#include <cartesian_interface/ros/RosServerClass.h>
#include <cartesian_interface/utils/LockfreeBufferImpl.h>


namespace XBot { namespace Cartesian {

/**
 * @brief CartesianIO XBot IO Plugin. This plugin extends the CommunicationHandler
 * loop with custom functionalities.
 *
 **/
class CartesianIO : public XBot::IOPlugin
{

public:

    virtual bool init(std::string path_to_config_file, 
                      SharedMemory::Ptr shmem
                      );

    virtual void run();

    virtual void close();

protected:


private:
    
    RosServerClass::Ptr _ros_server;
    LockfreeBufferImpl::Ptr _ci;
    SharedObject<RosServerClass::Ptr> _ros_shobj;
    SharedObject<LockfreeBufferImpl::Ptr> _ci_shobj;
    SharedObject<std::atomic<bool>> _solver_running_shobj;


};

} }

#endif // CartesianIO_PLUGIN_H_


bool XBot::Cartesian::CartesianIO::init(std::string path_to_config_file, XBot::SharedMemory::Ptr shmem)
{
    _ci_shobj = shmem->getSharedObject<LockfreeBufferImpl::Ptr>("/xbotcore/ci_buffer");
    
    _solver_running_shobj = shmem->getSharedObject<std::atomic<bool>>("/xbotcore/ci_running");
    _solver_running_shobj.get_object_ptr()->store(false);
    
    _ros_shobj = shmem->getSharedObject<RosServerClass::Ptr>("/xbotcore/ci_ros_server_class");
}

void XBot::Cartesian::CartesianIO::run()
{
    
    bool ros_server_acquired = _ros_server || (_ros_shobj.try_get(_ros_server) && _ros_server);
    bool ci_buffer_acquired = _ci || (_ci_shobj.try_get(_ci) && _ci);
    bool ci_solver_running = _solver_running_shobj.get_object_ptr()->load();
    
    if(ros_server_acquired && ci_buffer_acquired && ci_solver_running)
    {
        _ci->updateState();
        _ros_server->run();
    }
}

void XBot::Cartesian::CartesianIO::close()
{

}
