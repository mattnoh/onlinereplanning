#pragma once
#include "planner_core/types.hpp"
#include <memory>
namespace planner_core {
// Shared by both transports. Neither this interface nor its implementation uses ROS.
class Controller {
public:
    Controller();
    ~Controller();
    Controller(const Controller&)=delete;
    Controller& operator=(const Controller&)=delete;
    // Optional RH descriptor supports ROS-free plants through the same runtime.
    void configure(const PlannerCoreConfig&, std::shared_ptr<const OCPDescriptor> descriptor={});
    // Frozen simulation time during solves; execute whole stages between solves.
    void configureSynchronous(const PlannerCoreConfig&, std::shared_ptr<const OCPDescriptor> descriptor={});
    PlannerCoreStepResult solveSynchronous(const PlannerCoreInput&);
    void reset();
    void stop();
    void setTerminalTarget(const Eigen::Vector3d&);
    PlannerCoreStepResult trySolve(const PlannerCoreInput&);
    PlannerCoreStepResult solveOpenLoop(const PlannerCoreInput&);
    PlannerCoreStepResult sample(const PlannerCoreInput&);
    ExecutionMode executionMode() const;
    bool isDirect() const {return executionMode()==ExecutionMode::DirectMpc;}
    double commandPeriod() const;
    double solvePollPeriod() const;
    int solveCount() const;
    void openControlLog(const std::string&);
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}
