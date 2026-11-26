#include "ocp/ocp_hover.hpp"
#include "ocp/ocp_stc_landing_noaug.hpp"
#include "planner_core/types.hpp"
#include <map>
#include <stdexcept>
#include <string>
namespace OCPRegistry {
const std::map<std::string, OCPDescriptor>& getTable() {
    using StcLandingNoAugOCP::Enforcement;
    static const std::map<std::string, OCPDescriptor> table = {
        {"hover", HoverOCP::descriptor()},
        {"stc_landing_noaug", StcLandingNoAugOCP::descriptor()},
        {"stc_landing_noaug_node", StcLandingNoAugOCP::descriptor(Enforcement::Node)},
    };
    return table;
}

const OCPDescriptor& getDescriptor(const std::string& ocp_type) {
    const auto& table = getTable();
    auto it = table.find(ocp_type);
    if (it == table.end()) throw std::runtime_error("Unknown OCP type: " + ocp_type);
    return it->second;
}

double getDT(const std::string& ocp_type)            { return getDescriptor(ocp_type).dt; }
int    getDefaultNReplay(const std::string& ocp_type) { return getDescriptor(ocp_type).default_n_replay; }
double getDefaultMassKg(const std::string& ocp_type)  { return getDescriptor(ocp_type).default_mass_kg; }
aligator_ocp::SolverSettings getSolverSettings(const std::string& ocp_type) {
    return getDescriptor(ocp_type).solver_settings();
}
std::shared_ptr<aligator_ocp::Ocp> create(const std::string& ocp_type, const OCPCreateArgs& args) {
    return getDescriptor(ocp_type).create(args);
}
}  // namespace OCPRegistry
