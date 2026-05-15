// HTN sandbox — a self-contained, throwaway program to understand the search
// algorithm before we wire it into tiny_cell.
//
// Domain is intentionally trivial: 1-D world, three task kinds (Palletize,
// Transport, DetectPresence), four strategies (Arm, Push, Belt, Laser).
// Everything tunable lives in input.json — edit it, re-run the exe, watch
// the search trace change. No threepp, no glm, no entt. Just nlohmann/json
// and the standard library.
//
// Once you trust the algorithm, we'll port the structure (not the code)
// back into tiny_cell with the real cost model and catalog underneath.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

// ── Domain types ────────────────────────────────────────────────────────────

enum class TaskKind { Palletize, Transport, DetectPresence };

const char* task_name(TaskKind k) {
    switch (k) {
        case TaskKind::Palletize:      return "Palletize";
        case TaskKind::Transport:      return "Transport";
        case TaskKind::DetectPresence: return "DetectPresence";
    }
    return "?";
}

struct Task {
    TaskKind kind = TaskKind::Palletize;

    // Palletize
    float rate_per_minute     = 0.f;
    float source_distance_m   = 0.f;

    // Transport (filled when emitted as a sub-task of Push)
    float transport_distance_m = 0.f;
    float transport_rate_per_minute = 0.f;
};

struct Equipment {
    std::string label;        // e.g. "1×Arm", "Pusher", "Belt 1.0m"
    float       annual_cost_eur = 0.f;
};

// A partial or complete solution. Carries cumulative equipment + cost
// and the sub-tasks still to be solved. Terminal when `remaining` is empty.
struct Proposal {
    TaskKind                  root_task = TaskKind::Palletize;
    std::vector<Equipment>    equipment;
    std::vector<Task>         remaining;
    std::string               strategy_name;
    float                     cost_so_far_eur    = 0.f;
    float                     cost_lb_eur        = 0.f;
};

bool is_terminal(const Proposal& p) { return p.remaining.empty(); }

// ── Strategy parameters loaded from JSON ────────────────────────────────────

struct StrategyParams {
    // Arm
    float arm_cost_each_eur            = 1000.f;
    float arm_max_rate_per_minute      = 10.f;
    int   arm_max_count                = 3;

    // Push
    float pusher_cost_eur              = 500.f;
    float push_max_rate_per_minute     = 12.f;

    // Belt
    float belt_base_cost_eur           = 100.f;
    float belt_per_metre_eur           = 50.f;

    // Laser
    float laser_cost_eur               = 50.f;
};

// ── Strategy interface (we WILL extract this later if it survives review) ──

class Strategy {
public:
    virtual ~Strategy() = default;
    virtual const char* name()                 const = 0;
    virtual bool can_solve(TaskKind k)         const = 0;
    virtual std::vector<Proposal> propose(const Task& t,
                                          const StrategyParams& sp) const = 0;
};

// ── Concrete strategies ─────────────────────────────────────────────────────

// Arms — solve Palletize directly, no sub-tasks. Returns one proposal per N.
class ArmStrategy final : public Strategy {
public:
    const char* name() const override { return "ArmStrategy"; }
    bool can_solve(TaskKind k) const override { return k == TaskKind::Palletize; }

    std::vector<Proposal> propose(const Task& t,
                                  const StrategyParams& sp) const override
    {
        std::vector<Proposal> out;
        for (int n = 1; n <= sp.arm_max_count; ++n) {
            const float capacity = float(n) * sp.arm_max_rate_per_minute;
            if (capacity < t.rate_per_minute) continue;        // not enough arms
            Proposal p;
            p.root_task     = TaskKind::Palletize;
            p.strategy_name = "ArmStrategy(N=" + std::to_string(n) + ")";
            p.equipment.push_back({std::to_string(n) + "×Arm",
                                   float(n) * sp.arm_cost_each_eur});
            p.cost_so_far_eur = p.cost_lb_eur = float(n) * sp.arm_cost_each_eur;
            out.push_back(std::move(p));
        }
        return out;
    }
};

// Push — solve Palletize with a pusher + sub-tasks (Transport + DetectPresence).
class PushStrategy final : public Strategy {
public:
    const char* name() const override { return "PushStrategy"; }
    bool can_solve(TaskKind k) const override { return k == TaskKind::Palletize; }

    std::vector<Proposal> propose(const Task& t,
                                  const StrategyParams& sp) const override
    {
        if (t.rate_per_minute > sp.push_max_rate_per_minute) return {};

        Proposal p;
        p.root_task     = TaskKind::Palletize;
        p.strategy_name = "PushStrategy";
        p.equipment.push_back({"Pusher", sp.pusher_cost_eur});
        p.cost_so_far_eur = sp.pusher_cost_eur;

        // Sub-task: transport items from source over the source distance.
        Task transport;
        transport.kind                       = TaskKind::Transport;
        transport.transport_distance_m       = t.source_distance_m;
        transport.transport_rate_per_minute  = t.rate_per_minute;
        p.remaining.push_back(transport);

        // Sub-task: detect when an item is at the pusher.
        Task detect;
        detect.kind = TaskKind::DetectPresence;
        p.remaining.push_back(detect);

        // Lower bound = own + minimum-possible-remaining. We hard-code a small
        // floor for the open sub-tasks (a Belt of length 0 and a Laser at
        // catalog price). In real code, each strategy would publish its own
        // floor; here we just stub it inline.
        p.cost_lb_eur = sp.pusher_cost_eur + sp.belt_base_cost_eur + sp.laser_cost_eur;
        return {p};
    }
};

// Belt — solve Transport directly. Cost scales with distance.
class BeltStrategy final : public Strategy {
public:
    const char* name() const override { return "BeltStrategy"; }
    bool can_solve(TaskKind k) const override { return k == TaskKind::Transport; }

    std::vector<Proposal> propose(const Task& t,
                                  const StrategyParams& sp) const override
    {
        Proposal p;
        p.root_task     = TaskKind::Transport;
        p.strategy_name = "BeltStrategy";
        const float cost = sp.belt_base_cost_eur + sp.belt_per_metre_eur * t.transport_distance_m;
        char label[64];
        std::snprintf(label, sizeof(label), "Belt %.1fm", t.transport_distance_m);
        p.equipment.push_back({label, cost});
        p.cost_so_far_eur = p.cost_lb_eur = cost;
        return {p};
    }
};

// Laser — solve DetectPresence directly.
class LaserStrategy final : public Strategy {
public:
    const char* name() const override { return "LaserStrategy"; }
    bool can_solve(TaskKind k) const override { return k == TaskKind::DetectPresence; }

    std::vector<Proposal> propose(const Task& /*t*/,
                                  const StrategyParams& sp) const override
    {
        Proposal p;
        p.root_task     = TaskKind::DetectPresence;
        p.strategy_name = "LaserStrategy";
        p.equipment.push_back({"Laser", sp.laser_cost_eur});
        p.cost_so_far_eur = p.cost_lb_eur = sp.laser_cost_eur;
        return {p};
    }
};

// ── Search algorithm ────────────────────────────────────────────────────────
//
// Best-first by `cost_lb_eur`. Maintain an "open" set of partial proposals.
// On each step:
//   1. Pick the partial with the lowest lower bound.
//   2. If it's terminal AND its cost ≤ all other open lower bounds, return it.
//   3. Otherwise pick one unresolved sub-task in `remaining`, dispatch to
//      every strategy that can_solve it, splice each returned sub-proposal
//      into the parent and reinsert as a new partial.
//
// Trace-prints every step so you can read along.

struct SearchOutcome {
    bool                    found = false;
    Proposal                solution;
};

// Splice a child proposal (which solves one specific sub-task) into a parent
// partial. Removes that sub-task from `remaining`, appends child equipment,
// adds child cost. Returns the new (potentially still-partial) proposal.
Proposal splice(const Proposal& parent, size_t sub_idx,
                const Proposal& child)
{
    Proposal np = parent;
    np.remaining.erase(np.remaining.begin() + sub_idx);
    for (const auto& e : child.equipment) np.equipment.push_back(e);
    np.cost_so_far_eur += child.cost_so_far_eur;
    // New lower bound: own cost-so-far (already includes child) +
    // minimum-possible-remaining for the still-open sub-tasks.
    // For this sandbox we don't have a per-strategy floor lookup, so we
    // approximate: parent's lb gets reduced by the child's lb (we no longer
    // need to budget for that sub-task) and grown by what the child actually
    // cost.
    np.cost_lb_eur = parent.cost_lb_eur
                   - child.cost_lb_eur               // remove the floor we budgeted for this sub-task
                   + child.cost_so_far_eur;           // add actual realised cost
    return np;
}

void print_proposal(const Proposal& p, const char* prefix = "  ") {
    std::printf("%s[%-22s] cost=%6.1f  lb=%6.1f%s\n",
                prefix, p.strategy_name.c_str(),
                p.cost_so_far_eur, p.cost_lb_eur,
                is_terminal(p) ? "  TERMINAL" : "");
    for (const auto& e : p.equipment) {
        std::printf("%s    + %-18s  €%7.1f\n", prefix, e.label.c_str(), e.annual_cost_eur);
    }
    for (const auto& t : p.remaining) {
        std::printf("%s    ? needs %s\n", prefix, task_name(t.kind));
    }
}

SearchOutcome search(const Task& root_task,
                     const std::vector<std::unique_ptr<Strategy>>& strategies,
                     const StrategyParams& sp)
{
    std::vector<Proposal> open;

    // Seed: every strategy that can solve the root task gets called once.
    std::printf("=== Initial dispatch on root task: %s ===\n",
                task_name(root_task.kind));
    for (const auto& s : strategies) {
        if (!s->can_solve(root_task.kind)) continue;
        auto props = s->propose(root_task, sp);
        for (auto& p : props) {
            print_proposal(p, "  ");
            open.push_back(std::move(p));
        }
    }
    if (open.empty()) {
        std::printf("\nNo strategy solves the root task. STOP.\n");
        return {};
    }

    int step = 0;
    while (!open.empty()) {
        ++step;
        std::printf("\n--- Step %d --- open=%zu\n", step, open.size());

        // Pick the partial with the lowest lower bound.
        auto best_it = std::min_element(open.begin(), open.end(),
            [](const Proposal& a, const Proposal& b){
                return a.cost_lb_eur < b.cost_lb_eur;
            });

        // If the best is terminal: check whether it beats every other open's lb.
        if (is_terminal(*best_it)) {
            const float terminal_cost = best_it->cost_so_far_eur;
            bool another_branch_could_beat_it = false;
            for (auto it = open.begin(); it != open.end(); ++it) {
                if (it == best_it) continue;
                if (it->cost_lb_eur < terminal_cost) {
                    another_branch_could_beat_it = true;
                    break;
                }
            }
            if (!another_branch_could_beat_it) {
                std::printf("  Picked terminal: %s (cost=%.1f). No open partial can beat it. DONE.\n",
                            best_it->strategy_name.c_str(), terminal_cost);
                SearchOutcome out;
                out.found    = true;
                out.solution = *best_it;
                return out;
            }
            // A partial branch might still produce a cheaper terminal — keep going.
            std::printf("  Best is terminal (%s, cost=%.1f) but another branch's lb is lower; explore further.\n",
                        best_it->strategy_name.c_str(), terminal_cost);
            // Move it aside; we'll come back to it if nothing beats.
            // Implementation: just skip it by sorting open and stepping through.
            // Easiest: pick the cheapest non-terminal partial instead.
            best_it = std::min_element(open.begin(), open.end(),
                [](const Proposal& a, const Proposal& b){
                    const float la = is_terminal(a) ? std::numeric_limits<float>::infinity() : a.cost_lb_eur;
                    const float lb = is_terminal(b) ? std::numeric_limits<float>::infinity() : b.cost_lb_eur;
                    return la < lb;
                });
            if (is_terminal(*best_it)) {
                // Nothing else to explore — return the best terminal.
                std::printf("  No non-terminal partials remain.\n");
                auto cheapest_terminal = std::min_element(open.begin(), open.end(),
                    [](const Proposal& a, const Proposal& b){ return a.cost_so_far_eur < b.cost_so_far_eur; });
                SearchOutcome out;
                out.found    = true;
                out.solution = *cheapest_terminal;
                return out;
            }
        }

        // Expand a non-terminal partial: dispatch one of its sub-tasks.
        Proposal partial = *best_it;
        open.erase(best_it);
        std::printf("  Expanding partial:\n");
        print_proposal(partial, "    ");

        // Pick the first unresolved sub-task.
        const size_t sub_idx = 0;
        Task sub = partial.remaining[sub_idx];
        std::printf("  Dispatch sub-task: %s\n", task_name(sub.kind));

        bool any_match = false;
        for (const auto& s : strategies) {
            if (!s->can_solve(sub.kind)) continue;
            auto child_props = s->propose(sub, sp);
            for (const auto& cp : child_props) {
                any_match = true;
                Proposal spliced = splice(partial, sub_idx, cp);
                print_proposal(cp,      "      child ");
                print_proposal(spliced, "      ->    ");
                open.push_back(std::move(spliced));
            }
        }
        if (!any_match) {
            std::printf("  No strategy solves %s. Branch dead.\n", task_name(sub.kind));
        }
    }

    return {};
}

// ── JSON loader ─────────────────────────────────────────────────────────────

StrategyParams load_strategy_params(const nlohmann::json& j) {
    StrategyParams sp;
    if (j.contains("arm")) {
        const auto& a = j["arm"];
        sp.arm_cost_each_eur       = a.value("cost_each_eur", sp.arm_cost_each_eur);
        sp.arm_max_rate_per_minute = a.value("max_rate_per_minute", sp.arm_max_rate_per_minute);
        sp.arm_max_count           = a.value("max_count", sp.arm_max_count);
    }
    if (j.contains("push")) {
        const auto& p = j["push"];
        sp.pusher_cost_eur          = p.value("pusher_cost_eur", sp.pusher_cost_eur);
        sp.push_max_rate_per_minute = p.value("max_rate_per_minute", sp.push_max_rate_per_minute);
    }
    if (j.contains("belt")) {
        const auto& b = j["belt"];
        sp.belt_base_cost_eur = b.value("base_cost_eur", sp.belt_base_cost_eur);
        sp.belt_per_metre_eur = b.value("per_metre_eur", sp.belt_per_metre_eur);
    }
    if (j.contains("laser")) {
        sp.laser_cost_eur = j["laser"].value("cost_eur", sp.laser_cost_eur);
    }
    return sp;
}

Task load_task(const nlohmann::json& j) {
    Task t;
    t.kind                = TaskKind::Palletize;
    t.rate_per_minute     = j.value("rate_per_minute", 6.f);
    t.source_distance_m   = j.value("source_distance_m", 1.f);
    return t;
}

// ── Main ────────────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
    const std::string path = argc > 1 ? argv[1] : "experiments/htn_sandbox/input.json";
    std::ifstream f(path);
    if (!f) {
        std::fprintf(stderr, "Cannot open %s\n", path.c_str());
        return 1;
    }
    nlohmann::json doc;
    f >> doc;

    Task task            = load_task(doc.at("task"));
    StrategyParams sp    = load_strategy_params(doc.value("strategies", nlohmann::json::object()));

    std::printf("Task: Palletize(rate=%.1f/min, source_distance=%.2fm)\n\n",
                task.rate_per_minute, task.source_distance_m);

    std::vector<std::unique_ptr<Strategy>> strategies;
    strategies.push_back(std::make_unique<ArmStrategy>());
    strategies.push_back(std::make_unique<PushStrategy>());
    strategies.push_back(std::make_unique<BeltStrategy>());
    strategies.push_back(std::make_unique<LaserStrategy>());

    auto outcome = search(task, strategies, sp);

    std::printf("\n========================================\n");
    if (!outcome.found) {
        std::printf("No feasible solution.\n");
        return 2;
    }
    std::printf("WINNING SOLUTION\n");
    std::printf("  Strategy:       %s\n", outcome.solution.strategy_name.c_str());
    std::printf("  Annual cost:    €%.1f\n", outcome.solution.cost_so_far_eur);
    std::printf("  Equipment:\n");
    for (const auto& e : outcome.solution.equipment) {
        std::printf("    • %-20s  €%7.1f\n", e.label.c_str(), e.annual_cost_eur);
    }
    return 0;
}
