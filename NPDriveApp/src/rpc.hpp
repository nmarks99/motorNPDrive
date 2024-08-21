// Library of helper functions for JSON-RPC 2.0 protocol
// for Renaissance Scientific NP-Drive nanopositioner controller
#include "nlohmann/json.hpp"

using json = nlohmann::json;

namespace NPDriveCmd {
    const json rpc_template = {
        {"jsonrpc", "2.0"},
        {"id", 0}
    };

    std::string get_position(int axis);

    std::string get_drive_channel();

    std::string get_status_drive_busy();

    std::string get_status_drive_overload();

    std::string get_status_positioning();

    std::string stop_positioning();

    template <typename T>
    T get_result(const std::string &in_string) {
        const json response_json = json::parse(std::string(in_string));
        return response_json["result"].template get<T>();
    }
}
