#ifndef __CARTESIAN_INTERFACE_ENUM_H__
#define __CARTESIAN_INTERFACE_ENUM_H__

#include <string>

namespace XBot { namespace Cartesian {

/**
 * @brief The ActivationState enum
 */
enum class ActivationState { Enabled, Disabled };

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

template <typename T>
std::string EnumToString(T value);

template <typename T>
T StringToEnum(const std::string& value);

template <>
std::string EnumToString<ActivationState>(ActivationState value);

template <>
std::string EnumToString<ControlType>(ControlType value);

template <>
std::string EnumToString<State>(State value);

template <>
ActivationState StringToEnum<ActivationState>(const std::string& value);

template <>
ControlType StringToEnum<ControlType>(const std::string& value);

template <>
State StringToEnum<State>(const std::string& value);

} }

#endif // __CARTESIAN_INTERFACE_ENUM_H__
