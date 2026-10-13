#pragma once
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <string>
namespace comando_parameters::stc_landing {
struct StcLandingParameters {
// ── Env helpers ───────────────────────────────────────────────────────────────
static double envOrQ(const char* n, double d) {
    const char* v = std::getenv(n);
    return v ? std::atof(v) : d;
}
static int envOrInt(const char* n, int d) {
    const char* v = std::getenv(n);
    return v ? std::atoi(v) : d;
}

// ── Dimensions ────────────────────────────────────────────────────────────────
static constexpr int NX      = 13;
static constexpr int NU      = 4;
static constexpr int NX_SS   = 14;   // NX + IDX_DT (IDX_CTCS_Y intentionally REMOVED)
static constexpr int NU_SS   = 5;    // NU + IDX_THETA
static constexpr int IDX_DT     = 13;
static constexpr int IDX_THETA  = 4;

// ── Vehicle parameters ────────────────────────────────────────────────────────
static constexpr double MASS    = 0.027;
static constexpr double IXX     = 1.66e-5;
static constexpr double IYY     = 1.66e-5;
static constexpr double IZZ     = 2.92e-5;
static constexpr double J_SCALE = 1.0 / IXX;
// Benchmark split (quad_single_horizon_noaug_stc): the PLAN is bounded to
// FMIN/FMAX with margin inside the ACTUATOR_F* physical limits; TAU_MAX
// derives from the actuator limits, not the planning bounds.
static constexpr double ACTUATOR_FMIN = 0.08;
static constexpr double ACTUATOR_FMAX = 0.60;
static constexpr double FMIN    = 0.12;
static constexpr double FMAX    = 0.52;
static constexpr double L_ARM   = 0.046;
const double TAU_MAX =
    L_ARM * (ACTUATOR_FMAX/4.0 - ACTUATOR_FMIN/4.0) * J_SCALE;

const Eigen::Matrix3d J_B = [](){
    Eigen::Matrix3d J; J.setZero();
    J(0,0)=IXX*J_SCALE; J(1,1)=IYY*J_SCALE; J(2,2)=IZZ*J_SCALE;
    return J;
}();
const Eigen::Vector3d GRAVITY{0.0, 0.0, -9.81};

// ── Horizon / timing (env-resolved once at startup) ──────────────────────────
const int    HORIZON = std::max(1, std::getenv("SZMUK_RH_N")
                                  ? std::atoi(std::getenv("SZMUK_RH_N"))
                                  : envOrInt("SZMUK_N", 30));
const int    NEX     = std::max(1, envOrInt("SZMUK_RH_NEX", 7));
const double TH0     = envOrQ("SZMUK_TH0", 0.07);
const double THL     = envOrQ("SZMUK_THL", 0.05);
const double THH     = envOrQ("SZMUK_THH", 0.12);

// ── Glideslope / floor / descent cap ─────────────────────────────────────────
static constexpr double GS_APEX_OFFSET  = 0.1;
static constexpr double VZ_LAND_MAX     = 0.5;

// ── Terminal cost weights ─────────────────────────────────────────────────────
const double TERM_W_POS_XY  = envOrQ("SZMUK_TERM_W_POS_XY", 2500.0);
const double TERM_W_POS_Z   = envOrQ("SZMUK_TERM_W_POS_Z", 2500.0);
const double TERM_W_VEL_XY  = envOrQ("SZMUK_TERM_W_VEL_XY", 900.0);
const double TERM_W_VEL_Z   = envOrQ("SZMUK_TERM_W_VEL_Z", 900.0);
const double TERM_W_TILT    = envOrQ("SZMUK_TERM_W_TILT", 200.0);
const double TERM_W_OM      = envOrQ("SZMUK_TERM_W_OM", 50.0);
const double TERM_VZ_REF    = envOrQ("SZMUK_TERM_VZ_REF", 0.0);
const double STAGE_W_TIME   = envOrQ("SZMUK_W_TIME", 0.04);
const double R_THETA        = envOrQ("SZMUK_R_THETA", 10.0);
const double THETA_REF      = envOrQ("SZMUK_THETA_REF", 0.10);

// ── Smoothness/control weights ────────────────────────────────────────────────
static constexpr double HOVER_THRUST = MASS * 9.81;
const double R_CTRL_THR  = envOrQ("SZMUK_R_THR", 2.0);
const double R_CTRL_MOM  = envOrQ("SZMUK_R_MOM", 0.6);
const double W_TOUCH_VEL = envOrQ("SZMUK_W_TOUCH_VEL", 10.0);
const double TOUCH_ALT   = envOrQ("SZMUK_TOUCH_ALT", 0.25);
const double TOUCH_SCALE = envOrQ("SZMUK_TOUCH_SCALE", 0.25);
const double W_STAGE_TARGET_XY  = envOrQ("SZMUK_W_STAGE_TARGET_XY", 0.0);
const double W_STAGE_TARGET_Z   = envOrQ("SZMUK_W_STAGE_TARGET_Z", 0.0);
const double W_STAGE_TARGET_VEL = envOrQ("SZMUK_W_STAGE_TARGET_VEL", 0.0);

// ── CT-cSTC constraint bounds ─────────────────────────────────────────────────
const double THETA_PHASE0_DEG   = envOrQ("SZMUK_TILT_PHASE0_DEG", 11.0);
const double OMEGA_PHASE0_MAX   = envOrQ("SZMUK_OMEGA_PHASE0_MAX", 0.35);
const double SPD_PHASE0_MAX     = envOrQ("SZMUK_SPD_PHASE0_MAX", 3.0);
const double THETA_STC_DEG      = envOrQ("SZMUK_TILT_STC_DEG", 0.95);
const double ALPHA_THETA_STC    = envOrQ("SZMUK_ALPHA_THETA_STC", 1.0);
const double OMEGA_STC_MAX      = envOrQ("SZMUK_OMEGA_STC_MAX", 0.019);
const double GS_CONE_HALF_ANGLE_DEG = envOrQ("SZMUK_GS_CONE_DEG", 15.0);
const double GS_CONE_TAN            = std::tan(GS_CONE_HALF_ANGLE_DEG * M_PI / 180.0);
const double SPD_STC_MAX        = envOrQ("SZMUK_SPD_STC_MAX", 0.75);
const double W_LAND_SPEED       = envOrQ("SZMUK_W_LAND_SPEED", 1.0);
const double W_LAND_TILT        = envOrQ("SZMUK_W_LAND_TILT", 1.0e5);
const double W_LAND_OMEGA       = envOrQ("SZMUK_W_LAND_OMEGA", 100.0);
const double W_LAND_GS          = envOrQ("SZMUK_W_LAND_GS", 5000.0);
const double LOS_CONE_HALF_ANGLE_DEG = envOrQ("SZMUK_LOS_CONE_DEG", 30.0);
const double LOS_CONE_TAN            = std::tan(LOS_CONE_HALF_ANGLE_DEG * M_PI / 180.0);
const double W_STC_LOS               = envOrQ("SZMUK_W_STC_LOS", 50.0);
const double LOS_ALT_TRIG            = envOrQ("SZMUK_LOS_ALT_TRIG", 1.8);
const double LOS_TRIGGER_SCALE       = envOrQ("SZMUK_LOS_TRIGGER_SCALE", 0.25);
const double LOS_TRIGGER_FLOOR       = envOrQ("SZMUK_LOS_TRIGGER_FLOOR", 0.0);

// ── Altitude trigger / lateral capture cost gate ─────────────────────────────
const double ALT_TRIG        = envOrQ("SZMUK_ALT_TRIG", 0.8);
static constexpr double CAP_COST_RADIUS = 0.50;
const double CAP_LAND_RADIUS = envOrQ("SZMUK_RH_CAP_LAND_RADIUS", 0.45);
static constexpr double K_CAP          = 20.0;
const double Z_STAGE            = envOrQ("SZMUK_Z_STAGE", 1.8);
const double STAGE_RADIUS       = envOrQ("SZMUK_STAGE_RADIUS", 1.10);
const double STAGE_TRIGGER_SCALE = envOrQ("SZMUK_STAGE_TRIGGER_SCALE", 0.25);
const double STAGE_TRIGGER_FLOOR = envOrQ("SZMUK_STAGE_TRIGGER_FLOOR", 0.0);
const double STAGE_SETTLE_TILT_DEG  = envOrQ("SZMUK_STAGE_SETTLE_TILT_DEG", 12.0);
const double STAGE_SETTLE_OMEGA_MAX = envOrQ("SZMUK_STAGE_SETTLE_OMEGA_MAX", 0.50);
const double W_STC_STAGE        = envOrQ("SZMUK_W_STC_STAGE", 10.0);
static constexpr double CAP_DESC_RADIUS = 0.12;
static constexpr double K_DESC          = 40.0;

// ── Runtime toggles ───────────────────────────────────────────────────────────
static bool useTriggeredLandingStc() {
    const char* env = std::getenv("SZMUK_QUAD_TRIG_STC");
    return !(env && std::string(env) == "0");
}

static std::string stageTriggerMode() {
    const char* env = std::getenv("SZMUK_STAGE_TRIGGER");
    return env ? std::string(env) : std::string("radius");
}

// NOTE: no ctcsEnforcementMode() here — the interval inequality is hard-wired
// (SZMUK_CTCS_MODE is deliberately ignored; see file header).

static bool useRhFeedbackWarmStart() {
    const char* env = std::getenv("SZMUK_RH_FEEDBACK_WARM_START");
    return !(env && std::string(env) == "0");
}

// Faithful to quad_single_horizon_noaug_stc.cpp: the landing trigger is the
// RAW altitude margin posPart(ALT_TRIG - z), alt-only — this is the trigger
// the noaug_basin champion tuning (EPS/Y_SCALE) was found with. The RH-clean
// normalized capture-AND form ((trig_alt/ALT_TRIG) * trig_cap/cap^2) is kept
// behind an opt-in env for A/B only; it rescales the eps budget and zeroes
// the landing block outside CAP_LAND_RADIUS, deviating from the benchmark.
static bool useRhCaptureLandingTrigger() {
    const char* env = std::getenv("SZMUK_RH_CAPTURE_LANDING_TRIGGER");
    return env && std::string(env) != "0";
}

// Density rescaling (rocket Y_SCALE pattern, kept for tuning parity with the
// benchmark arm): the constraint sees integral(H/Y_SCALE dt) <= EPS/Y_SCALE,
// so the zero set is unchanged; there is no y row to condition here, but the
// scaled units keep the noaug_basin (eps, yscale) tuning table applicable.
// noaug_basin champion: EPS=1.6e-3, Y_SCALE=10.
const double CTCS_Y_SCALE = [] {
    const double s = envOrQ("SZMUK_CTCS_Y_SCALE", 10.0);
    return s > 0.0 ? s : 1.0;
}();
const double CTCS_STEP_EPS = envOrQ("SZMUK_CTCS_STEP_EPS", 1.6e-3);


};
inline const StcLandingParameters defaults{};
inline const auto& GRAVITY=defaults.GRAVITY;
inline constexpr auto NX = StcLandingParameters::NX;
inline constexpr auto NU = StcLandingParameters::NU;
inline constexpr auto NX_SS = StcLandingParameters::NX_SS;
inline constexpr auto NU_SS = StcLandingParameters::NU_SS;
inline constexpr auto IDX_DT = StcLandingParameters::IDX_DT;
inline constexpr auto IDX_THETA = StcLandingParameters::IDX_THETA;
inline constexpr auto MASS = StcLandingParameters::MASS;
inline constexpr auto IXX = StcLandingParameters::IXX;
inline constexpr auto IYY = StcLandingParameters::IYY;
inline constexpr auto IZZ = StcLandingParameters::IZZ;
inline constexpr auto J_SCALE = StcLandingParameters::J_SCALE;
inline constexpr auto ACTUATOR_FMIN = StcLandingParameters::ACTUATOR_FMIN;
inline constexpr auto ACTUATOR_FMAX = StcLandingParameters::ACTUATOR_FMAX;
inline constexpr auto FMIN = StcLandingParameters::FMIN;
inline constexpr auto FMAX = StcLandingParameters::FMAX;
inline constexpr auto L_ARM = StcLandingParameters::L_ARM;
inline const auto& TAU_MAX = defaults.TAU_MAX;
inline const auto& J_B = defaults.J_B;
inline const auto& HORIZON = defaults.HORIZON;
inline const auto& NEX = defaults.NEX;
inline const auto& TH0 = defaults.TH0;
inline const auto& THL = defaults.THL;
inline const auto& THH = defaults.THH;
inline constexpr auto GS_APEX_OFFSET = StcLandingParameters::GS_APEX_OFFSET;
inline constexpr auto VZ_LAND_MAX = StcLandingParameters::VZ_LAND_MAX;
inline const auto& TERM_W_POS_XY = defaults.TERM_W_POS_XY;
inline const auto& TERM_W_POS_Z = defaults.TERM_W_POS_Z;
inline const auto& TERM_W_VEL_XY = defaults.TERM_W_VEL_XY;
inline const auto& TERM_W_VEL_Z = defaults.TERM_W_VEL_Z;
inline const auto& TERM_W_TILT = defaults.TERM_W_TILT;
inline const auto& TERM_W_OM = defaults.TERM_W_OM;
inline const auto& TERM_VZ_REF = defaults.TERM_VZ_REF;
inline const auto& STAGE_W_TIME = defaults.STAGE_W_TIME;
inline const auto& R_THETA = defaults.R_THETA;
inline const auto& THETA_REF = defaults.THETA_REF;
inline constexpr auto HOVER_THRUST = StcLandingParameters::HOVER_THRUST;
inline const auto& R_CTRL_THR = defaults.R_CTRL_THR;
inline const auto& R_CTRL_MOM = defaults.R_CTRL_MOM;
inline const auto& W_TOUCH_VEL = defaults.W_TOUCH_VEL;
inline const auto& TOUCH_ALT = defaults.TOUCH_ALT;
inline const auto& TOUCH_SCALE = defaults.TOUCH_SCALE;
inline const auto& W_STAGE_TARGET_XY = defaults.W_STAGE_TARGET_XY;
inline const auto& W_STAGE_TARGET_Z = defaults.W_STAGE_TARGET_Z;
inline const auto& W_STAGE_TARGET_VEL = defaults.W_STAGE_TARGET_VEL;
inline const auto& THETA_PHASE0_DEG = defaults.THETA_PHASE0_DEG;
inline const auto& OMEGA_PHASE0_MAX = defaults.OMEGA_PHASE0_MAX;
inline const auto& SPD_PHASE0_MAX = defaults.SPD_PHASE0_MAX;
inline const auto& THETA_STC_DEG = defaults.THETA_STC_DEG;
inline const auto& ALPHA_THETA_STC = defaults.ALPHA_THETA_STC;
inline const auto& OMEGA_STC_MAX = defaults.OMEGA_STC_MAX;
inline const auto& GS_CONE_HALF_ANGLE_DEG = defaults.GS_CONE_HALF_ANGLE_DEG;
inline const auto& GS_CONE_TAN = defaults.GS_CONE_TAN;
inline const auto& SPD_STC_MAX = defaults.SPD_STC_MAX;
inline const auto& W_LAND_SPEED = defaults.W_LAND_SPEED;
inline const auto& W_LAND_TILT = defaults.W_LAND_TILT;
inline const auto& W_LAND_OMEGA = defaults.W_LAND_OMEGA;
inline const auto& W_LAND_GS = defaults.W_LAND_GS;
inline const auto& LOS_CONE_HALF_ANGLE_DEG = defaults.LOS_CONE_HALF_ANGLE_DEG;
inline const auto& LOS_CONE_TAN = defaults.LOS_CONE_TAN;
inline const auto& W_STC_LOS = defaults.W_STC_LOS;
inline const auto& LOS_ALT_TRIG = defaults.LOS_ALT_TRIG;
inline const auto& LOS_TRIGGER_SCALE = defaults.LOS_TRIGGER_SCALE;
inline const auto& LOS_TRIGGER_FLOOR = defaults.LOS_TRIGGER_FLOOR;
inline const auto& ALT_TRIG = defaults.ALT_TRIG;
inline constexpr auto CAP_COST_RADIUS = StcLandingParameters::CAP_COST_RADIUS;
inline const auto& CAP_LAND_RADIUS = defaults.CAP_LAND_RADIUS;
inline constexpr auto K_CAP = StcLandingParameters::K_CAP;
inline const auto& Z_STAGE = defaults.Z_STAGE;
inline const auto& STAGE_RADIUS = defaults.STAGE_RADIUS;
inline const auto& STAGE_TRIGGER_SCALE = defaults.STAGE_TRIGGER_SCALE;
inline const auto& STAGE_TRIGGER_FLOOR = defaults.STAGE_TRIGGER_FLOOR;
inline const auto& STAGE_SETTLE_TILT_DEG = defaults.STAGE_SETTLE_TILT_DEG;
inline const auto& STAGE_SETTLE_OMEGA_MAX = defaults.STAGE_SETTLE_OMEGA_MAX;
inline const auto& W_STC_STAGE = defaults.W_STC_STAGE;
inline constexpr auto CAP_DESC_RADIUS = StcLandingParameters::CAP_DESC_RADIUS;
inline constexpr auto K_DESC = StcLandingParameters::K_DESC;
inline const auto& CTCS_Y_SCALE = defaults.CTCS_Y_SCALE;
inline const auto& CTCS_STEP_EPS = defaults.CTCS_STEP_EPS;
inline double envOrQ(const char* n,double d){return StcLandingParameters::envOrQ(n,d);}
inline int envOrInt(const char* n,int d){return StcLandingParameters::envOrInt(n,d);}
inline bool useTriggeredLandingStc(){return StcLandingParameters::useTriggeredLandingStc();}
inline std::string stageTriggerMode(){return StcLandingParameters::stageTriggerMode();}
inline bool useRhFeedbackWarmStart(){return StcLandingParameters::useRhFeedbackWarmStart();}
inline bool useRhCaptureLandingTrigger(){return StcLandingParameters::useRhCaptureLandingTrigger();}
}
