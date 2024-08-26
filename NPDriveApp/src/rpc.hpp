// Library of helper functions for JSON-RPC 2.0 protocol
// for Renaissance Scientific NP-Drive nanopositioner controller

#include "nlohmann/json.hpp"
using json = nlohmann::json;

namespace NPDriveCmd {
    const json rpc_template = {
        {"jsonrpc", "2.0"},
        {"id", 0}
    };

    std::string stop_motion();
    std::string get_drive_channel();
    std::string get_status_drive_busy();
    std::string get_status_drive_overload();

    std::string get_position(int axis);
    std::string go_position(int channel, double target, int amplitude, int frequency);
    std::string get_status_positioning();
    std::string stop_positioning();
    std::string set_sensors_off();
    std::string set_drive_channels_off();
    std::string set_stop_limit(int channel, double threshold);
    std::string hold_position(int channel, double target, int amplitude, int timeout);

}
