// Library of helper functions for JSON-RPC 2.0 protocol
// for Renaissance Scientific NP-Drive nanopositioner controller
#include "nlohmann/json.hpp"

using json = nlohmann::json;

namespace NPDriveCmd {
    const json rpc_template = {
        {"jsonrpc", "2.0"},
        {"id", 0}
    };

    // Open loop
    std::string stop_motion();
    std::string get_drive_channel();
    std::string get_status_drive_busy();
    std::string get_status_drive_overload();

    // Closed loop
    std::string get_position(int axis);
    std::string go_position(int channel, double target, int amplitude, int frequency);
    std::string get_status_positioning();
    std::string stop_positioning();

}
