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
#include "constraint_audit.hpp"
using namespace constraint_audit;
int main(int argc,char** argv){try{
 if(argc!=3)throw std::runtime_error("Usage: constraint_audit OCP RUN_DIRECTORY");
 const std::string type=argv[1],dir=argv[2];const auto& descriptor=OCPRegistry::getDescriptor(type);
 if(!descriptor.make_model)throw std::runtime_error("OCP has no propagation model");
 // all_solves.csv holds accepted horizons in order; solve_num restarts after a
 // re-prime, so a horizon is delimited by its node-0 row, not by its number.
 std::map<int,std::vector<Row>> solves;int horizon=-1;for(const auto& r:read(dir+"/all_solves.csv")){if(val(r,"node")==0)++horizon;if(horizon<0)throw std::runtime_error("all_solves.csv does not start at node 0");solves[horizon].push_back(r);}
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
