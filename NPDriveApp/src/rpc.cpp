#include "rpc.hpp"
namespace NPDriveCmd {

    std::string stop_motion() {
        json rpc = rpc_template;
        rpc["method"] = "stopMotion";
        return rpc.dump();
    }

    std::string get_drive_channel() {
        json rpc = rpc_template;
        rpc["method"] = "getDriveChannel";
        return rpc.dump();
    }

    std::string get_status_drive_busy() {
        json rpc = rpc_template;
        rpc["method"] = "getStatusDriveBusy";
        return rpc.dump();
    }

    std::string get_status_drive_overload() {
        json rpc = rpc_template;
        rpc["method"] = "getStatusDriveOverload";
        return rpc.dump();
    } 

    // ------------

    std::string get_position(int axis) {
        json rpc = rpc_template;
        rpc["method"] = "getPosition";
        rpc["params"] = {axis};
        return rpc.dump();
    }

    std::string go_position(int channel, double target, int amplitude, int frequency) {
        json rpc = rpc_template;
        rpc["method"] = "goPosition";
        rpc["params"] = {channel, target, amplitude, frequency};
        return rpc.dump();
    }

    std::string get_status_positioning() {
        json rpc = rpc_template;
        rpc["method"] = "getStatusPositioning";
        return rpc.dump();
    }

    std::string stop_positioning() {
        json rpc = rpc_template;
        rpc["method"] = "stopPositioning";
        return rpc.dump();
    }
}
