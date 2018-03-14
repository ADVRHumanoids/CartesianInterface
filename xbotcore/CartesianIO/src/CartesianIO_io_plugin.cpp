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

#include <CartesianIO_io_plugin.h>

/* Specify that the class XBotPlugin::CartesianIO is a XBot RT plugin with name "CartesianIO" */
REGISTER_XBOT_IO_PLUGIN_(XBotPlugin::CartesianIO)

namespace XBotPlugin {

bool XBotPlugin::CartesianIO::init(std::string path_to_config_file)
{
    return true;
}

void XBotPlugin::CartesianIO::run()
{

}

void XBotPlugin::CartesianIO::close()
{

}



}