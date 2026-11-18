// Print the physical bounds of the stc_landing_noaug preset selected by
// STC_PRESET as JSON, keyed like bag_to_stc_csv.resolve_constants().
#include "ocp/ocp_stc_landing_noaug.hpp"
#include <cstdio>
int main() {
    const auto& p = StcLandingNoAugOCP::P;
    std::printf("{\"alt_trig\":%.17g,\"z_stage\":%.17g,\"stage_radius\":%.17g,\"stage_trigger_scale\":%.17g,"
                "\"stage_trigger_floor\":%.17g,\"los_alt_trig\":%.17g,\"los_trigger_scale\":%.17g,"
                "\"los_trigger_floor\":%.17g,\"los_cone_deg\":%.17g,\"v_phase0_max\":%.17g,\"v_land_stc_max\":%.17g,"
                "\"omega_phase0_max\":%.17g,\"omega_land_stc_max\":%.17g,\"tilt_phase0_deg\":%.17g,"
                "\"gs_cone_deg\":%.17g,\"gs_apex_offset\":%.17g,\"tilt_land_stc_deg\":%.17g}\n",
                p.alt_trig, p.z_stage, p.stage_radius, p.stage_trigger_scale, p.stage_trigger_floor,
                p.los_alt_trig, p.los_trigger_scale, p.los_trigger_floor, p.los_cone_deg, p.spd_phase0_max,
                p.spd_stc_max, p.omega_phase0_max, p.omega_stc_max, p.tilt_phase0_deg, p.gs_cone_deg,
                p.gs_apex_offset, p.alpha_theta_stc * p.tilt_stc_deg);
}
