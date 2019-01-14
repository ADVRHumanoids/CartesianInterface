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

#include <XCM/IOPlugin.h>
#include <cartesian_interface/ros/RosServerClass.h>
#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <std_srvs/SetBool.h>


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

    bool switchCallback(std_srvs::SetBoolRequest& req, std_srvs::SetBoolResponse& res);
    
    bool _is_running = false;
    ros::ServiceServer _switch_srv;    
    ModelInterface::Ptr _model;
    RosServerClass::Ptr _ros_server;
    CartesianInterfaceImpl::Ptr _ci;
    SharedObject<CartesianInterfaceImpl::Ptr> _ci_shobj;


};

} }

#endif // CartesianIO_PLUGIN_H_

bool XBot::Cartesian::CartesianIO::switchCallback(std_srvs::SetBoolRequest& req, 
                                                  std_srvs::SetBoolResponse& res) 
{
    _is_running = req.data;
    res.success = true;
    res.message = "Setting switch to: ";
    res.message += _is_running ? "true" : "false";
    return true;
}


bool XBot::Cartesian::CartesianIO::init(std::string path_to_config_file, XBot::SharedMemory::Ptr shmem)
{
    _model = ModelInterface::getModel(path_to_config_file);
    
    /* Load IK problem and solver */
    auto yaml_file = YAML::LoadFile(path_to_config_file);
    ProblemDescription ik_problem(yaml_file["WalkingStackCI"]["problem_description"], _model);
    std::string impl_name = yaml_file["WalkingStackCI"]["solver"].as<std::string>();
    
    _ci = std::make_shared<CartesianInterfaceImpl>(_model, ik_problem);
    
    /* Obtain class to expose ROS API */
    RosServerClass::Options options;
    options.spawn_markers = false;
    options.tf_prefix = "mpl";
    _ros_server = std::make_shared<RosServerClass>(_ci, _model, options);
    
    _ci_shobj = shmem->getSharedObject<CartesianInterfaceImpl::Ptr>("/xbotcore/cartesian_interface");
    _ci_shobj.set(_ci);
    
    ros::NodeHandle nh;
    _switch_srv = nh.advertiseService("/xbotcore/cartesian/switch", 
                                      &CartesianIO::switchCallback, 
                                      this
                                      );
}

void XBot::Cartesian::CartesianIO::run()
{
    if(_is_running) {
        std::lock_guard<XBot::Mutex> lock_guard(*_ci_shobj.get_mutex());
        _ros_server->run();
    }
}

void XBot::Cartesian::CartesianIO::close()
{

}
