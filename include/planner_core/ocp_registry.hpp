#pragma once
#include "planner_core/types.hpp"
#include <map>
namespace OCPRegistry {
const std::map<std::string,OCPDescriptor>& getTable();
const OCPDescriptor& getDescriptor(const std::string&);
double getDT(const std::string&);
int getDefaultNReplay(const std::string&);
double getDefaultMassKg(const std::string&);
Param getSolverParams(const std::string&);
std::shared_ptr<OptimalControlProblem<double>> create(const std::string&,const OCPCreateArgs&);
}
