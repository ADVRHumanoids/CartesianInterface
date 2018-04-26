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

#ifndef CartesianPlugin_PLUGIN_H_
#define CartesianPlugin_PLUGIN_H_

#include <XCM/XBotControlPlugin.h>
#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <CartesianPlugin/Utils.h>


namespace XBot { namespace Cartesian {

/**
 * @brief CartesianPlugin XBot RT Plugin
 *
 **/
class CartesianPlugin : public XBot::XBotControlPlugin
{

public:
    
    virtual bool init_control_plugin(XBot::Handle::Ptr handle);

    virtual bool close();

    virtual void on_start(double time);

    virtual void on_stop(double time);
    
    virtual ~CartesianPlugin();

protected:

    virtual void control_loop(double time, double period);

private:

    XBot::RobotInterface::Ptr _robot;
    ModelInterface::Ptr _model;
    
    CartesianInterfaceImpl::Ptr _ci;
    SharedObject<CartesianInterfaceImpl::Ptr> _ci_shobj;
    bool _first_sync_done;

    double _start_time;

    Eigen::VectorXd _q, _qdot;

    XBot::MatLogger::Ptr _logger;

};

} }

#endif // CartesianPlugin_PLUGIN_H_
