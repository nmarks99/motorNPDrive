// Library of helper functions for JSON-RPC 2.0 protocol
// for Renaissance Scientific NP-Drive nanopositioner controller

#include "nlohmann/json.hpp"

using json = nlohmann::json;

namespace NPDriveCmd {
const json rpc_template = {{"jsonrpc", "2.0"}, {"id", 0}};

// channel specific commands
std::string go_steps_forward(int channel, int steps, int amplitude, int frequency);
std::string go_steps_reverse(int channel, int steps, int amplitude, int frequency);
std::string go_continuous_forward(int channel, int amplitude, int frequency);
std::string go_continuous_reverse(int channel, int amplitude, int frequency);
std::string get_position(int channel);
std::string set_drive_channel(int channel);
std::string go_position(int channel, double target, int amplitude, int frequency);
std::string set_stop_limit(int channel, double threshold);
std::string hold_position(int channel, double target, int amplitude, int timeout);

// controller level commands
std::string stop_motion();
std::string get_drive_channel();
std::string get_status_drive_busy();
std::string get_status_drive_overload();
std::string get_status_positioning();
std::string stop_positioning();
std::string set_sensors_off();
std::string set_drive_channels_off();

} // namespace NPDriveCmd
