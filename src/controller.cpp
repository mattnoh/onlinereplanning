#include "planner_core/controller.hpp"
#include "controller_internal.hpp"
#include "synchronous_mpc_internal.hpp"
#include <shared_mutex>
namespace planner_core {
struct Controller::Impl {
    mutable std::shared_mutex lifecycle;
    std::mutex solver;
    PlannerCoreConfig config;
    PlannerCore rh;
    DirectMpcController direct;
    SynchronousMpcController synchronous;
    bool synchronous_mode=false;
    ExecutionMode mode=ExecutionMode::RhReplanning;
    int accepted=0;
    bool configured=false;
};
Controller::Controller():impl_(std::make_unique<Impl>()){}
Controller::~Controller()=default;
void Controller::configure(const PlannerCoreConfig& c,std::shared_ptr<const OCPDescriptor> descriptor) {
    std::unique_lock<std::shared_mutex> guard(impl_->lifecycle);
    impl_->direct.stop();impl_->synchronous.stop();impl_->synchronous_mode=false;impl_->config=c;impl_->accepted=0;
    impl_->mode=(descriptor?*descriptor:OCPRegistry::getDescriptor(c.ocp_type)).execution_mode;
    if(descriptor&&impl_->mode==ExecutionMode::DirectMpc)
        throw std::invalid_argument("descriptor injection supports RH execution only");
    if(impl_->mode==ExecutionMode::DirectMpc) impl_->direct.configure(c);
    else impl_->rh.configure(c,std::move(descriptor));
    impl_->configured=true;
}
void Controller::configureSynchronous(const PlannerCoreConfig& c,std::shared_ptr<const OCPDescriptor> descriptor) {
    std::unique_lock<std::shared_mutex> guard(impl_->lifecycle);
    impl_->direct.stop();impl_->synchronous.stop();impl_->rh.reset();impl_->config=c;impl_->accepted=0;
    impl_->synchronous_mode=false;impl_->configured=false;
    impl_->synchronous.configure(c,std::move(descriptor));
    impl_->synchronous_mode=true;impl_->mode=impl_->synchronous.executionMode();impl_->configured=true;
}
PlannerCoreStepResult Controller::solveSynchronous(const PlannerCoreInput& input) {
    std::shared_lock<std::shared_mutex> guard(impl_->lifecycle);
    std::lock_guard<std::mutex> solve(impl_->solver);
    if(!impl_->configured||!impl_->synchronous_mode){PlannerCoreStepResult out;out.rejection_reason="synchronous_mpc_not_configured";return out;}
    auto out=impl_->synchronous.solve(input);if(out.solve_accepted)++impl_->accepted;return out;
}
void Controller::reset(){std::unique_lock<std::shared_mutex> guard(impl_->lifecycle);impl_->accepted=0;if(!impl_->configured)return;if(impl_->synchronous_mode)impl_->synchronous.reset();else if(impl_->mode==ExecutionMode::DirectMpc)impl_->direct.reset();else impl_->rh.reset();}
void Controller::resetForRelativeHold(){reset();std::unique_lock<std::shared_mutex> guard(impl_->lifecycle);if(impl_->configured&&!impl_->synchronous_mode&&impl_->mode!=ExecutionMode::DirectMpc)impl_->rh.setRelativeHoldStart(true);}
void Controller::stop(){std::unique_lock<std::shared_mutex> guard(impl_->lifecycle);impl_->direct.stop();impl_->synchronous.stop();impl_->rh.reset();impl_->configured=false;}
void Controller::setTerminalTarget(const Eigen::Vector3d& p){std::unique_lock<std::shared_mutex> guard(impl_->lifecycle);impl_->config.terminal_position_abs=p;if(impl_->configured && impl_->mode==ExecutionMode::RhReplanning)impl_->rh.setTerminalTarget(p);}
PlannerCoreStepResult Controller::trySolve(const PlannerCoreInput& in){
    std::shared_lock<std::shared_mutex> guard(impl_->lifecycle);if(!impl_->configured)return {};
    std::lock_guard<std::mutex> solve(impl_->solver);
    if(impl_->synchronous_mode){PlannerCoreStepResult out;out.rejection_reason="use_solveSynchronous";return out;}
    auto out=impl_->mode==ExecutionMode::DirectMpc?impl_->direct.trySolve(in):impl_->rh.trySolve(in);
    if(out.solve_accepted)++impl_->accepted;
    return out;
}
PlannerCoreStepResult Controller::solveOpenLoop(const PlannerCoreInput& in){std::shared_lock<std::shared_mutex> guard(impl_->lifecycle);std::lock_guard<std::mutex> solve(impl_->solver);if(impl_->mode==ExecutionMode::DirectMpc){PlannerCoreStepResult out;out.rejection_reason="direct_mpc_requires_feedback";return out;}return impl_->rh.solveOpenLoop(in);}
PlannerCoreStepResult Controller::sample(const PlannerCoreInput& in){std::shared_lock<std::shared_mutex> guard(impl_->lifecycle);if(!impl_->configured)return {};if(impl_->synchronous_mode){std::lock_guard<std::mutex> solve(impl_->solver);return impl_->synchronous.sample(in);}return impl_->mode==ExecutionMode::DirectMpc?impl_->direct.sample(in.now,in.current_state,in.target_snapshot):impl_->rh.sampleReplay(in.now,in.target_snapshot);}
ExecutionMode Controller::executionMode() const {std::shared_lock<std::shared_mutex> guard(impl_->lifecycle);return impl_->mode;}
double Controller::commandPeriod() const {return comando_parameters::command_dt;}
double Controller::solvePollPeriod() const {return .001;}
int Controller::solveCount() const {std::shared_lock<std::shared_mutex> guard(impl_->lifecycle);std::lock_guard<std::mutex> solve(impl_->solver);return impl_->accepted;}
void Controller::openControlLog(const std::string& path){std::unique_lock<std::shared_mutex> guard(impl_->lifecycle);if(impl_->mode==ExecutionMode::DirectMpc)impl_->direct.openControlLog(path);}
}

// Shared OCP solver implementation.

#include <algorithm>
#include <iostream>

using namespace std;

OcpSolver::OcpSolver(const Config& config) : config_(config) {
    try {
        solver_params_ = descriptor().getSolverParams();
    } catch (const std::runtime_error& e) {
        std::cerr << "ERROR: " << e.what() << "\n";
        throw;
    }
}

const OCPDescriptor& OcpSolver::descriptor() const {
    return config_.descriptor?*config_.descriptor:OCPRegistry::getDescriptor(config_.ocp_type);
}

double OcpSolver::getOcpDt() const {
    return descriptor().dt;
}

OcpSolver::WarmStartSnapshot OcpSolver::snapshotWarmStart() const {
    WarmStartSnapshot snapshot;
    snapshot.prev_X = prev_X_;
    snapshot.prev_U = prev_U_;
    snapshot.prev_K = prev_K_;
    snapshot.last_solve_ms = last_solve_ms_;
    return snapshot;
}

void OcpSolver::restoreWarmStart(const WarmStartSnapshot& snapshot) {
    prev_X_ = snapshot.prev_X;
    prev_U_ = snapshot.prev_U;
    prev_K_ = snapshot.prev_K;
    last_solve_ms_ = snapshot.last_solve_ms;
    next_warm_start_unshifted_ = false;
    next_warm_start_shift_ = -1;
}

void OcpSolver::keepLatestAsUnexecutedWarmStart() {
    next_warm_start_unshifted_ = true;
}

std::vector<Eigen::VectorXd> OcpSolver::makeUwarm(int n_shift) const {
    if (prev_U_.empty()) return {};

    const int N = static_cast<int>(prev_U_.size());
    if (N == 1) return prev_U_;

    const int NEX = std::max(0, std::min(n_shift, N - 1));
    const auto desc = descriptor();
    if (NEX == 0) {
        std::vector<Eigen::VectorXd> uw = prev_U_;
        for (auto& u : uw) {
            if (desc.sanitize_warm_control) {
                desc.sanitize_warm_control(u);
            }
        }
        return uw;
    }

    Eigen::VectorXd u_tail = prev_U_[N - 1 - NEX];
    if (u_tail.size() >= 4) {
        u_tail(1) = 0.0;
        u_tail(2) = 0.0;
        u_tail(3) = 0.0;
    }
    if (desc.sanitize_warm_control) {
        desc.sanitize_warm_control(u_tail);
    }

    std::vector<Eigen::VectorXd> uw(N);
    for (int i = 0; i < N; ++i) {
        if (i + NEX < N - NEX) {
            uw[i] = prev_U_[i + NEX];
        } else {
            uw[i] = u_tail;
        }
        if (desc.sanitize_warm_control) {
            desc.sanitize_warm_control(uw[i]);
        }
    }
    return uw;
}

std::vector<Eigen::VectorXd> OcpSolver::makeXshifted(int n_shift) const {
    if (prev_X_.empty()) return {};

    const int Nx = static_cast<int>(prev_X_.size());
    const int NEX = std::max(0, n_shift);

    std::vector<Eigen::VectorXd> xs(Nx);
    for (int i = 0; i < Nx; ++i) {
        const int source=i+NEX;
        if(source<Nx)xs[i]=prev_X_[source];
        else if(problem_&&!prev_U_.empty()){
            const auto& previous=i==0?prev_X_.back():xs[i-1];
            xs[i]=problem_->getDynamics(problem_->getHorizon()-1)->f(previous,prev_U_.back());
        }else xs[i]=prev_X_.back();
    }
    return xs;
}

std::vector<Eigen::MatrixXd> OcpSolver::makeKshifted(int n_shift) const {
    if (prev_K_.empty()) return {};

    const int Nk = static_cast<int>(prev_K_.size());
    const int NEX = std::max(0, n_shift);

    std::vector<Eigen::MatrixXd> ks(Nk);
    for (int i = 0; i < Nk; ++i) {
        ks[i] = prev_K_[std::min(i + NEX, Nk - 1)];
    }
    return ks;
}

void OcpSolver::setupProblem(const OCPCreateArgs& args) {
    try {
        solver_.reset();
        problem_ = descriptor().create(args);
    } catch (const std::runtime_error& e) {
        std::cerr << "ERROR: " << e.what() << "\n";
        throw;
    }
}

OcpSolver::Result OcpSolver::solve(const Eigen::VectorXd& current_state,
                                           const Eigen::Vector3d& target_accel,
                                           const std::any& extra_params,
                                           double t_abs)
{
    Result result;
    result.success = false;
    auto t0 = chrono::steady_clock::now();
    result.solve_start_time = t0;
    result.solve_timestamp = t0;

    try {
        target_accel_ = target_accel;

        std::vector<Eigen::VectorXd> warm_u;
        std::vector<Eigen::VectorXd> warm_x;
        std::vector<Eigen::MatrixXd> warm_k;
        if (!config_.use_native_initial_guess) {
            warm_u = prev_U_; warm_x = prev_X_; warm_k = prev_K_;
        }

        if (!prev_U_.empty() && !config_.use_native_initial_guess) {
            auto desc = descriptor();
            const int warm_shift = next_warm_start_unshifted_ ? 0 :
                (next_warm_start_shift_ >= 0 ? next_warm_start_shift_ : config_.n_shift);
            if (desc.warm_start == OCPDescriptor::WarmStart::Feedback) {
                warm_u = makeUwarm(warm_shift);
                warm_x = makeXshifted(warm_shift);
                warm_k = makeKshifted(warm_shift);
            } else {
                // Shift-only means a control initial guess, not an old
                // feedback policy evaluated against a different measured root.
                warm_k.clear();
                const int Nu = static_cast<int>(prev_U_.size());
                const int Nx = static_cast<int>(prev_X_.size());
                const int shift = std::max(0, std::min(warm_shift, std::max(1, Nu - 1)));

                std::vector<Eigen::VectorXd> su(Nu);
                for (int i = 0; i < Nu; ++i) {
                    su[i] = prev_U_[std::min(i + shift, Nu - 1)];
                }
                warm_u = std::move(su);

                if (Nx > 0) {
                    warm_x = makeXshifted(shift);
                }
            }
        }

        OCPCreateArgs args;
        args.current_state = current_state;
        args.terminal_state = config_.terminal_state;
        args.prev_U = warm_u;
        args.prev_X = warm_x;
        args.prev_K = warm_k;
        args.target_accel = target_accel;
        args.extra = extra_params;
        args.t_abs = t_abs;
        args.horizon = next_horizon_;
        next_horizon_ = 0;

        setupProblem(args);

        // Each measured-root request starts fresh IP/AL numerical state.
        solver_ = make_shared<ALIPDDP<double>>(*problem_);
        solver_->init(solver_params_);
        solver_->solve();

        auto X_result = solver_->getResX();
        auto U_result = solver_->getResU();
        auto K_result = solver_->getResK();
        result.primal = auditPrimal(*problem_,X_result,U_result,solver_params_);
        result.termination = solver_->getTerminationReason();
        result.optimality_error = solver_->getOptError();
        result.converged = result.termination == "Optimal Solution" &&
            std::isfinite(result.optimality_error) &&
            result.optimality_error <= solver_params_.tolerance &&
            result.primal.feasible(solver_params_.tolerance);

        const auto t_finish = chrono::steady_clock::now();
        result.solve_finish_time = t_finish;
        result.solve_time_ms = chrono::duration<double, milli>(
            t_finish - t0).count();
        result.solve_timestamp = result.solve_start_time;
        result.constraint_error = solver_->getError();
        result.solve_iters = static_cast<int>(solver_->getAllCost().size());
        result.extra_params = extra_params;
        last_solve_ms_ = result.solve_time_ms;

        if (X_result.size() > 1) {
            auto desc = descriptor();
            int state_dim = desc.state_dim;
            if (X_result[1].size() >= state_dim) {
                result.next_state = X_result[1].head(state_dim);
            } else {
                result.next_state = X_result[1];
            }
            result.state_trajectory = X_result;
            result.control_trajectory = U_result;
            result.feedback_gains = K_result;
            result.success = result.primal.feasible(solver_params_.tolerance);
            if(!result.success)std::cerr<<"[MPC] infeasible returned primal: constraints="
                <<result.primal.constraints<<" dynamics="<<result.primal.dynamics
                <<" termination="<<result.termination<<"\n";

            // A failed iterate is not a previous solution. In particular, a
            // failed first solve must not enable the short warm-update budget.
            if(result.success){
                prev_X_ = X_result;
                prev_U_ = U_result;
                prev_K_ = K_result;
            }
            next_warm_start_unshifted_ = false;
            next_warm_start_shift_ = -1;

            std::cout << "[MPC] solve " << result.solve_time_ms
                      << "ms iters=" << result.solve_iters
                      << " n_shift=" << config_.n_shift << "\n";
        } else {
            std::cerr << "ERROR: Empty trajectory\n";
            last_solve_ms_ = 0.0;
        }

    } catch (const exception& e) {
        std::cerr << "ERROR in solve: " << e.what() << "\n";
        last_solve_ms_ = 0.0;
        result.solve_finish_time = chrono::steady_clock::now();
    }

    return result;
}

void OcpSolver::setTerminalState(const Eigen::VectorXd& terminal) {
    if (terminal.size() > 0) {
        config_.terminal_state = terminal;
        solver_.reset();
        prev_U_.clear();
        prev_X_.clear();
        prev_K_.clear();
        last_solve_ms_ = 0.0;
    }
}
