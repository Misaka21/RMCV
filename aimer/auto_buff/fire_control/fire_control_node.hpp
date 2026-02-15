// auto_buff fire control node (2026)

#ifndef AIMER_AUTOBUFF_FIRE_CONTROL_NODE_HPP
#define AIMER_AUTOBUFF_FIRE_CONTROL_NODE_HPP

#include <string>

namespace autobuff::fire_control {

void start_fire_control_node(const std::string& config_path = "aimer.toml");

}  // namespace autobuff::fire_control

#endif  // AIMER_AUTOBUFF_FIRE_CONTROL_NODE_HPP

