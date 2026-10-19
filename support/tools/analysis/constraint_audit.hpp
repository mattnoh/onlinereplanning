#pragma once
#include "parameters.hpp"
#include <Eigen/Geometry>
#include <array>
#include <algorithm>
#include <ostream>
#include <limits>
#include <cmath>
namespace constraint_audit {
constexpr size_t NC=8;
static const std::array<const char*,NC> names={"altitude_floor_m","speed_m_s","tilt_deg","angular_rate_rad_s","glideslope_m","los_deg","staging_height_m","los_cone_m"};
static const std::array<double,NC> tolerances={1e-4,1e-4,.01,1e-4,1e-4,.01,1e-4,1e-4};
using Values=std::array<double,NC>;
static Values residuals(const Eigen::VectorXd& x,bool ordinary){
 if(x.size()<13||!x.head(13).allFinite()||x.segment<4>(6).norm()<1e-12){
  Values invalid;invalid.fill(std::numeric_limits<double>::infinity());return invalid;
 }
 const auto& p=comando_parameters::defaults.constraints;
 const Eigen::Quaterniond q=Eigen::Quaterniond(x(6),x(7),x(8),x(9)).normalized();
 const double tilt=2*std::asin(std::clamp(std::hypot(q.x(),q.y()),0.,1.))*180/M_PI;
 const double omega=x.segment<3>(10).norm();
 const bool land=ordinary||(p.triggered_stc&&x(2)<p.alt_trig&&(!p.capture_landing_trigger||x.head<2>().norm()<p.cap_land_radius));
 const bool los=ordinary||(p.triggered_stc&&x(2)<p.los_alt_trig);
 const Eigen::Vector3d axis=q*Eigen::Vector3d::UnitZ();
 const double axial=axis.dot(x.head<3>());
 const double lateral=(x.head<3>()-axial*axis).norm();
 const double norm=x.head<3>().norm();
 const double angle=norm<1e-12?0:std::acos(std::clamp((q*Eigen::Vector3d::UnitZ()).dot(x.head<3>())/norm,-1.,1.))*180/M_PI;
 const bool stage=!ordinary&&p.triggered_stc&&(p.stage_trigger=="settle"?(tilt>p.stage_settle_tilt_deg||omega>p.stage_settle_omega_max):x.head<2>().norm()>p.stage_radius);
 Values r={-x(2),x.segment<3>(3).norm()-(land?p.spd_stc_max:p.spd_phase0_max),tilt-(land?p.alpha_theta_stc*p.theta_stc_deg:p.theta_phase0_deg),omega-(land?p.omega_stc_max:p.omega_phase0_max),land?x.head<2>().norm()-std::tan(p.gs_cone_half_angle_deg*M_PI/180)*(x(2)+p.gs_apex_offset):0,los?angle-p.los_cone_half_angle_deg:0,stage?p.z_stage-x(2):0,los?lateral-std::tan(p.los_cone_half_angle_deg*M_PI/180)*axial:0};
 for(auto& v:r)v=std::max(0.,v);return r;
}
struct Stats {Values maxima{},duration{};long samples=0;double total=0;
 void add(const Values& r,double dt){++samples;total+=dt;for(size_t i=0;i<NC;++i){maxima[i]=std::max(maxima[i],r[i]);if(r[i]>tolerances[i])duration[i]+=dt;}}
 void write(std::ostream& out)const {out<<"{\"samples\":"<<samples<<",\"duration_sum_over_horizons_sec\":"<<total<<",\"constraints\":{";for(size_t i=0;i<NC;++i){if(i)out<<',';out<<'"'<<names[i]<<"\":{\"max_violation\":"<<maxima[i]<<",\"violation_duration_sum_sec\":"<<duration[i]<<",\"tolerance\":"<<tolerances[i]<<'}';}out<<"}}";}
};
}
