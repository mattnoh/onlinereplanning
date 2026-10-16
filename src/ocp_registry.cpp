#include "ocp/ocp_landing_rh.hpp"
#include "ocp/ocp_landing_mpc.hpp"
#include "ocp/ocp_hover.hpp"
#include "ocp/ocp_stc_landing_noaug.hpp"
#include "ocp/ocp_stc_landing_mpc.hpp"
#include "planner_core/types.hpp"
#include <map>
#include <stdexcept>
#include <string>
namespace OCPRegistry {
const std::map<std::string, OCPDescriptor>& getTable() {
    static const std::map<std::string, OCPDescriptor> table = {
        {"hover", HoverOCP::descriptor()},
        {"landing_rh", LandingRhOCP::descriptor()},
        {"landing_mpc", LandingMpcOCP::descriptor()},
        {"landing_mpc_ctcs", LandingMpcOCP::ctcsDescriptor()},
        {"stc_landing_noaug", StcLandingNoAugOCP::descriptor()},
        {"stc_landing_mpc", StcLandingMpcOCP::descriptor()},
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
