// Dense physical audit of logged accepted horizons using the selected OCP's model.
#include "planner_core/types.hpp"
#include "parameters.hpp"
#include <Eigen/Geometry>
#include <array>
#include <fstream>
#include <iostream>
#include <sstream>
#include <set>
#include <map>
#include <algorithm>
using Row=std::map<std::string,std::string>;
static std::vector<std::string> split(const std::string& line){std::vector<std::string> v;std::stringstream s(line);std::string p;while(std::getline(s,p,','))v.push_back(p);return v;}
static std::vector<Row> read(const std::string& path){std::ifstream f(path);if(!f)throw std::runtime_error("Cannot read "+path);std::string line;std::getline(f,line);auto names=split(line);std::vector<Row> out;while(std::getline(f,line)){auto values=split(line);Row row;for(size_t i=0;i<std::min(names.size(),values.size());++i)row[names[i]]=values[i];out.push_back(std::move(row));}return out;}
static double val(const Row& row,const std::string& key){auto it=row.find(key);return it==row.end()?0:std::stod(it->second);}
constexpr size_t NC=8;
static const std::array<const char*,NC> names={"altitude_floor_m","speed_m_s","tilt_deg","angular_rate_rad_s","glideslope_m","los_deg","staging_height_m","los_cone_m"};
static const std::array<double,NC> tolerances={1e-4,1e-4,.01,1e-4,1e-4,.01,1e-4,1e-4};
using Values=std::array<double,NC>;
static Values residuals(const Eigen::VectorXd& x,bool ordinary){
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
int main(int argc,char** argv){try{
 if(argc!=3)throw std::runtime_error("Usage: constraint_audit OCP RUN_DIRECTORY");
 const std::string type=argv[1],dir=argv[2];const auto& descriptor=OCPRegistry::getDescriptor(type);
 if(!descriptor.make_model)throw std::runtime_error("OCP has no propagation model");
 std::set<int> accepted;for(const auto& r:read(dir+"/solver_events.csv"))if(r.at("event")=="accepted")accepted.insert(static_cast<int>(val(r,"solve_num")));
 std::map<int,std::vector<Row>> solves;for(const auto& r:read(dir+"/all_solves.csv")){const int n=static_cast<int>(val(r,"solve_num"));if(accepted.count(n))solves[n].push_back(r);}
 Stats nodes,initial,local,rollout;double defect=0;int initial_infeasible=0;const bool ordinary=type.rfind("landing_",0)==0;
 const std::array<std::string,13> columns={"px","py","pz","vx","vy","vz","qw","qx","qy","qz","wx","wy","wz"};
 for(auto& [number,rows]:solves){
  std::sort(rows.begin(),rows.end(),[](const Row& a,const Row& b){return val(a,"node")<val(b,"node");});
  const Eigen::Vector3d acceleration(val(rows[0],"tgt_ax"),val(rows[0],"tgt_ay"),val(rows[0],"tgt_az"));auto model=descriptor.make_model(acceleration);
  auto state=[&](const Row& row){Eigen::VectorXd x=Eigen::VectorXd::Zero(model.nx);for(int i=0;i<13;++i)x(i)=val(row,columns[i]);x(13)=val(row,"t");return x;};
  Eigen::VectorXd continuous=state(rows[0]);auto first=residuals(continuous,ordinary);initial.add(first,0);bool bad=false;for(size_t j=0;j<NC;++j)bad=bad||first[j]>tolerances[j];initial_infeasible+=bad;
  for(size_t k=0;k<rows.size();++k){auto x=state(rows[k]);if(k>0)nodes.add(residuals(x,ordinary),0);if(k+1==rows.size())break;
   auto u=model.hover;for(size_t j=0;j<descriptor.control_names.size();++j)u(j)=val(rows[k],descriptor.control_names[j]);if(model.nu==5)u(4)=val(rows[k],"theta");
   const double interval=val(rows[k+1],"t")-val(rows[k],"t");if(interval<=0)throw std::runtime_error("nonpositive logged interval");
   for(double elapsed=0;elapsed<interval-1e-12;){const double h=std::min(.001,interval-elapsed);x=model.flow(x,u,h,false).x;continuous=model.flow(continuous,u,h,false).x;local.add(residuals(x,ordinary),h);rollout.add(residuals(continuous,ordinary),h);elapsed+=h;}
   auto next=state(rows[k+1]);if(x.segment<4>(6).dot(next.segment<4>(6))<0)next.segment<4>(6)*=-1;defect=std::max(defect,(x.head(13)-next.head(13)).norm());
  }
 }
 std::cout.precision(10);std::cout<<"{\"ocp\":\""<<type<<"\",\"accepted_horizons\":"<<solves.size()<<",\"initial_infeasible_horizons\":"<<initial_infeasible<<",\"max_dense_dynamics_defect_norm\":"<<defect<<",\"initial_nodes\":";initial.write(std::cout);std::cout<<",\"future_nodes\":";nodes.write(std::cout);std::cout<<",\"dense_intervals\":";local.write(std::cout);std::cout<<",\"continuous_rollout\":";rollout.write(std::cout);std::cout<<"}\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
