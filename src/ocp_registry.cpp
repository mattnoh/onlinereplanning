#include "ocp/ocp_hover.hpp"
#include "ocp/ocp_landing.hpp"
#include "ocp/ocp_stateswitch.hpp"
#include "ocp/ocp_stateswitch_stc.hpp"
#include "ocp/ocp_stateswitch_stc_hard.hpp"
#include "ocp/ocp_stc_landing.hpp"
#include "ocp/ocp_perception_track.hpp"
#include "ocp/ocp_stc_landing_noaug.hpp"
#include "ocp/ocp_stc_landing_mpc.hpp"
#include "ocp/ocp_tracking_circle_target.hpp"
#include "ocp/ocp_tracking_bodyrate_tf_noimu.hpp"
#include "ocp/ocp_tracking_bodyrate_tf_imu.hpp"
#include "ocp/ocp_tracking_bodyrate_bf_imu.hpp"
#include "ocp/ocp_tracking_bodyrate_bf_noimu.hpp"
#include "planner_core/types.hpp"
#include <map>
#include <stdexcept>
#include <string>
namespace OCPRegistry {

const std::map<std::string, OCPDescriptor>& getTable() {
    static const std::map<std::string, OCPDescriptor> table = {
        {"hover",                          HoverOCP::descriptor()},
        {"landing",                        LandingOCP::descriptor()},
        {"stateswitch_stc", StateswitchStcOCP::descriptor()},
        {"state_switch_stc", StateswitchStcOCP::descriptor()},
        {"stateswtich_stc", StateswitchStcOCP::descriptor()},
        {"stateswitch_stc_hard", StateswitchStcHardOCP::descriptor()},
        {"stateswitch",                    StateswitchOCP::descriptor()},
        {"state_switch",                   StateswitchOCP::descriptor()},
        {"stc_landing",                    StcLandingOCP::descriptor()},
        {"rh_stc",                         StcLandingOCP::descriptor()},
        {"perception_track",               PerceptionTrackOCP::descriptor()},
        {"stc_landing_noaug",              StcLandingNoAugOCP::descriptor()},
        {"stc_landing_mpc",                StcLandingMpcOCP::descriptor()},
        {"rh_stc_noaug",                   StcLandingNoAugOCP::descriptor()},
        {"tracking_circle_target",         TrackingCircleTargetOCP::descriptor()},
        {"circle_target",                  TrackingCircleTargetOCP::descriptor()},
        {"tracking_bodyrate_tf_noimu",     TrackingBodyrateTfNoImuOCP::descriptor()},
        {"tracking_bodyrate_tf_imu",       TrackingBodyrateTfImuOCP::descriptor()},
        {"tf_noimu",                       TrackingBodyrateTfNoImuOCP::descriptor()},
        {"tf_imu",                         TrackingBodyrateTfImuOCP::descriptor()},
        {"tracking_bodyrate_bf_imu",       TrackingBodyrateBfImuOCP::descriptor()},
        {"tracking_bodyrate_bf_noimu",     TrackingBodyrateBfNoImuOCP::descriptor()},
        {"bf_imu",                         TrackingBodyrateBfImuOCP::descriptor()},
        {"bf_noimu",                       TrackingBodyrateBfNoImuOCP::descriptor()},
    };
    return table;
}

const OCPDescriptor& getDescriptor(const std::string& ocp_type) {
    const auto& table = getTable();
    auto it = table.find(ocp_type);
    if (it == table.end()) {
        throw std::runtime_error("Unknown OCP type: " + ocp_type);
    }
    return it->second;
}

double getDT(const std::string& ocp_type)            { return getDescriptor(ocp_type).dt; }
int    getDefaultNReplay(const std::string& ocp_type) { return getDescriptor(ocp_type).default_n_replay; }
double getDefaultMassKg(const std::string& ocp_type)  { return getDescriptor(ocp_type).default_mass_kg; }
Param  getSolverParams(const std::string& ocp_type)   { return getDescriptor(ocp_type).getSolverParams(); }
std::shared_ptr<OptimalControlProblem<double>> create(
    const std::string& ocp_type, const OCPCreateArgs& args) {
    return getDescriptor(ocp_type).create(args);
}

} // namespace OCPRegistry
