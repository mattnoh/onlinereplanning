// Target-motion difficulty over a FIXED window, from the analytic models.
// Bag-derived stats were inconsistent (runs have different durations, which
// gave circle a 211 m "path length" against 23.8 m measured elsewhere).
#include "target/circular_target.hpp"
#include "target/figure8_target.hpp"
#include "target/straight_target.hpp"
#include "target/stop_and_go_target.hpp"
#include "target/segmented_accel_target.hpp"
#include <cstdio>
#include <cmath>
#include <algorithm>
#include <vector>
using namespace target_models;
template<class M> void row(const char* n, const M& m, double T=60.0, double dt=0.01){
  double path=0, vmax=0, amax=0; std::vector<double> vs, as;
  double xmin=1e9,xmax=-1e9,ymin=1e9,ymax=-1e9;
  Eigen::Vector3d prev=m.pos(0.0);
  for(double t=0;t<=T;t+=dt){
    Eigen::Vector3d p=m.pos(t), v=m.vel(t), a=m.accel(t);
    path+=(p-prev).norm(); prev=p;
    double sv=v.norm(), sa=a.norm();
    vs.push_back(sv); as.push_back(sa);
    vmax=std::max(vmax,sv); amax=std::max(amax,sa);
    xmin=std::min(xmin,p.x()); xmax=std::max(xmax,p.x());
    ymin=std::min(ymin,p.y()); ymax=std::max(ymax,p.y());
  }
  std::sort(vs.begin(),vs.end()); std::sort(as.begin(),as.end());
  printf("%s,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.1f\n",n,
         vs[vs.size()/2],vmax,as[as.size()/2],amax,xmax-xmin,ymax-ymin,path);
}
int main(){
  printf("motion,v_med,v_max,a_med,a_max,span_x,span_y,path_m\n");
  Eigen::Vector3d c(0,0,0.2);
  CircularTarget ci; ci.center=c; ci.R=1.0; ci.omega=0.4;              row("circle",ci);
  Figure8Target f;  f.center=c;  f.omega=0.3;                          row("figure8",f);
  StraightTarget s; s.center=c;  s.speed=0.4; s.span=2.0;              row("straight",s);
  StopAndGoTarget g; g.center=c; g.speed=0.4; g.span=2.0;              row("stop_and_go",g);
  row("piecewise_accel", makePiecewiseAccelTarget(c,0.15,2.0,0.5,0.4));
  row("random_accel",    makeRandomAccelTarget(c,0.20,2.0,0.5,0.4,42));
}
