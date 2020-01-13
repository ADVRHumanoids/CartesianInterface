#ifndef __CARTESIAN_INTERFACE_ENUM_H__
#define __CARTESIAN_INTERFACE_ENUM_H__

namespace XBot { namespace Cartesian {

/**
* @brief Enum describing a state for each task. Available states are:
*  - State::Online: the task is following an online getPoseReference
*  - State::Reacing: the task is performing a point-to-point motion
*/
enum class State { Reaching, Online };

/**
* @brief Enum describing a control mode for each task. Available values are:
*  - ControlType::Position: the task is following position references
*  - ControlType::Velocity: the task is following velocity references
*  - ControlType::Disabled: the task is disabled
*/
enum class ControlType { Position, Velocity };

} }

#endif // __CARTESIAN_INTERFACE_ENUM_H__
