// HTN sandbox — iteration 2: now with geometry, footprints, and multi-task
// claims. Strategies emit (position, footprint, work_area) along with cost.
// The solver maintains an occupancy map; new proposals must not overlap
// existing footprints. Unclaimed tasks get pushed out of newly placed
// footprints. A strategy whose work area covers multiple unclaimed tasks
// can claim all of them in a single proposal (shared equipment emerges
// from geometry).
//
// All 1-D. Positions are scalar; footprints are intervals. The point is to
// make the architecture legible — not to model real space.
//
// Build: built by the htn_sandbox target in the root CMakeLists.txt.
// Run:   ./build/Debug/htn_sandbox.exe [path/to/input.json]

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

// ── Intervals (1-D) ─────────────────────────────────────────────────────────

struct Interval {
    float a = 0.f;     // inclusive
    float b = 0.f;     // inclusive; a <= b
    bool contains(float p) const { return p >= a && p <= b; }
    bool overlaps(const Interval& o) const { return a <= o.b && o.a <= b; }
    float length() const { return b - a; }
};

static bool overlaps_any(const Interval& iv, const std::vector<Interval>& list) {
    for (const auto& x : list) if (iv.overlaps(x)) return true;
    return false;
}

// Find the closest position to `current` that lies in NONE of the given
// occupancy intervals, within bounds. Greedy: scan outward.
static float nearest_feasible_position(float current,
                                       const std::vector<Interval>& occupancy,
                                       float bounds_min, float bounds_max)
{
    auto blocked = [&](float p) {
        for (const auto& iv : occupancy)
            if (p > iv.a - 1e-4f && p < iv.b + 1e-4f) return true;
        return false;
    };
    if (!blocked(current)) return current;
    const float step = 0.1f;
    for (float d = step; d < (bounds_max - bounds_min); d += step) {
        if (current + d <= bounds_max && !blocked(current + d)) return current + d;
        if (current - d >= bounds_min && !blocked(current - d)) return current - d;
    }
    return current;  // fall back; will be detected as infeasible
}

// ── Domain types ────────────────────────────────────────────────────────────

enum class TaskKind { Palletize, Assemble, Transport, DetectPresence };

static const char* task_name(TaskKind k) {
    switch (k) {
        case TaskKind::Palletize:      return "Palletize";
        case TaskKind::Assemble:       return "Assemble";
        case TaskKind::Transport:      return "Transport";
        case TaskKind::DetectPresence: return "DetectPresence";
    }
    return "?";
}

static TaskKind kind_from_string(const std::string& s) {
    if (s == "Palletize")      return TaskKind::Palletize;
    if (s == "Assemble")       return TaskKind::Assemble;
    if (s == "Transport")      return TaskKind::Transport;
    if (s == "DetectPresence") return TaskKind::DetectPresence;
    throw std::runtime_error("unknown TaskKind: " + s);
}

struct Task {
    std::string id;
    TaskKind    kind = TaskKind::Palletize;
    float       position         = 0.f;
    float       mobility         = 0.f;   // can move within ±mobility; 0 = pinned
    float       rate_per_minute  = 0.f;

    // Transport sub-task only:
    float       transport_distance_m   = 0.f;
    float       transport_rate         = 0.f;
    float       transport_source_pos   = 0.f;
};

struct Equipment {
    std::string label;
    float       annual_cost_eur   = 0.f;
    float       footprint_length  = 0.f;
};

struct ProposalClaim {
    std::string task_id;   // for v0, claim is binary (full); no fractions
};

struct Proposal {
    std::vector<ProposalClaim> claims;
    std::vector<Equipment>     equipment;
    std::vector<Task>          remaining;       // sub-tasks (e.g., Belt, Laser for Push)
    std::string                strategy_name;

    // Geometry
    float position        = 0.f;
    float footprint_min   = 0.f;
    float footprint_max   = 0.f;
    float work_area_min   = 0.f;
    float work_area_max   = 0.f;

    float cost_so_far_eur = 0.f;
    float cost_lb_eur     = 0.f;
};

static bool is_terminal(const Proposal& p) { return p.remaining.empty(); }

// ── Strategy parameters loaded from JSON ────────────────────────────────────

struct StrategyParams {
    // Arm
    float arm_cost_each_eur            = 1000.f;
    float arm_max_rate_per_minute      = 10.f;
    int   arm_max_count                = 3;
    float arm_reach                    = 3.f;     // work-area half-width
    float arm_footprint_length         = 2.f;

    // Push
    float pusher_cost_eur              = 500.f;
    float push_max_rate_per_minute     = 12.f;
    float push_footprint_length        = 1.f;

    // Belt
    float belt_base_cost_eur           = 100.f;
    float belt_per_metre_eur           = 50.f;
    float belt_footprint_per_metre     = 1.f;     // belt occupies its length

    // Laser
    float laser_cost_eur               = 50.f;
    float laser_footprint_length       = 0.2f;
};

// ── Workflow / bounds ───────────────────────────────────────────────────────

struct Workflow {
    float source_position = 0.f;
    float sink_position   = 20.f;
    std::vector<Task> tasks;     // user-declared tasks (Palletize, Assemble)
};

// ── Solver state (the running, partial solution) ────────────────────────────

struct SolverState {
    std::vector<Interval>             occupancy;       // sorted-ish (we don't enforce strictly)
    std::map<std::string, float>      task_positions;  // current positions of unclaimed tasks
    std::set<std::string>             pinned_tasks;
    std::set<std::string>             claimed_tasks;
    std::vector<Proposal>             applied;         // history; pretty-printed at end
    float                             cost_so_far_eur = 0.f;
};

// ── Strategy interface ──────────────────────────────────────────────────────

class Strategy {
public:
    virtual ~Strategy() = default;
    virtual const char* name()                 const = 0;
    virtual bool can_solve(TaskKind k)         const = 0;
    virtual std::vector<Proposal> propose(const Task& task,
                                          const SolverState& state,
                                          const Workflow& wf,
                                          const StrategyParams& sp) const = 0;
};

// ── ArmStrategy ─────────────────────────────────────────────────────────────
//
// For a given task at position P, enumerate candidate anchor positions:
//   - over the task itself                (covers only this task)
//   - midpoint between this task and every OTHER unclaimed task within
//     2 × reach                           (potentially covers both)
// For each anchor, identify all unclaimed tasks within reach; check that
// their combined rate fits arm capacity; emit a proposal with all of them
// as claims. Footprint = [anchor - L/2, anchor + L/2].

class ArmStrategy final : public Strategy {
public:
    const char* name() const override { return "ArmStrategy"; }
    bool can_solve(TaskKind k) const override {
        return k == TaskKind::Palletize || k == TaskKind::Assemble;
    }

    std::vector<Proposal> propose(const Task& task,
                                  const SolverState& state,
                                  const Workflow& wf,
                                  const StrategyParams& sp) const override
    {
        std::vector<Proposal> out;

        // Candidate anchors: this task's own position, plus midpoints
        // with every other unclaimed and arm-solvable task within 2× reach.
        std::vector<float> anchors;
        anchors.push_back(task.position);

        for (const auto& other : wf.tasks) {
            if (other.id == task.id) continue;
            if (state.claimed_tasks.count(other.id)) continue;
            if (!can_solve(other.kind)) continue;
            const float other_pos = state.task_positions.count(other.id)
                ? state.task_positions.at(other.id) : other.position;
            const float dist = std::abs(other_pos - task.position);
            if (dist <= 2.f * sp.arm_reach) {
                anchors.push_back(0.5f * (task.position + other_pos));
            }
        }

        for (float anchor : anchors) {
            const float fp_a = anchor - 0.5f * sp.arm_footprint_length;
            const float fp_b = anchor + 0.5f * sp.arm_footprint_length;
            const float wa_a = anchor - sp.arm_reach;
            const float wa_b = anchor + sp.arm_reach;

            Interval footprint{fp_a, fp_b};
            if (overlaps_any(footprint, state.occupancy)) continue;
            // Reject footprints that exceed the workflow bounds.
            if (fp_a < wf.source_position || fp_b > wf.sink_position) continue;

            // Which tasks fall within work_area at this anchor?
            std::vector<std::string> covered;
            float total_rate = 0.f;
            for (const auto& t : wf.tasks) {
                if (state.claimed_tasks.count(t.id)) continue;
                if (!can_solve(t.kind)) continue;
                const float p = state.task_positions.count(t.id)
                    ? state.task_positions.at(t.id) : t.position;
                if (p >= wa_a && p <= wa_b) {
                    covered.push_back(t.id);
                    total_rate += t.rate_per_minute;
                }
            }
            if (covered.empty()) continue;                       // serves nothing
            if (total_rate > sp.arm_max_rate_per_minute) continue;  // capacity blown

            Proposal p;
            p.strategy_name = "ArmStrategy";
            p.position      = anchor;
            p.footprint_min = fp_a;  p.footprint_max = fp_b;
            p.work_area_min = wa_a;  p.work_area_max = wa_b;
            for (const auto& cid : covered) p.claims.push_back({cid});
            p.equipment.push_back({
                "1×Arm",
                sp.arm_cost_each_eur,
                sp.arm_footprint_length
            });
            p.cost_so_far_eur = p.cost_lb_eur = sp.arm_cost_each_eur;
            out.push_back(std::move(p));
        }
        return out;
    }
};

// ── PushStrategy ────────────────────────────────────────────────────────────
//
// Solves a single Palletize task (no Assemble — pushers don't assemble).
// Emits sub-tasks: a Transport (source → task position) and a DetectPresence.
// Anchor: the task position itself.

class PushStrategy final : public Strategy {
public:
    const char* name() const override { return "PushStrategy"; }
    bool can_solve(TaskKind k) const override { return k == TaskKind::Palletize; }

    std::vector<Proposal> propose(const Task& task,
                                  const SolverState& state,
                                  const Workflow& wf,
                                  const StrategyParams& sp) const override
    {
        if (task.rate_per_minute > sp.push_max_rate_per_minute) return {};

        const float anchor = task.position;
        const float fp_a = anchor - 0.5f * sp.push_footprint_length;
        const float fp_b = anchor + 0.5f * sp.push_footprint_length;
        if (overlaps_any({fp_a, fp_b}, state.occupancy)) return {};
        if (fp_a < wf.source_position || fp_b > wf.sink_position) return {};

        Proposal p;
        p.strategy_name = "PushStrategy";
        p.position      = anchor;
        p.footprint_min = fp_a;  p.footprint_max = fp_b;
        p.work_area_min = fp_a;  p.work_area_max = fp_b;  // pusher doesn't reach beyond itself
        p.claims.push_back({task.id});
        p.equipment.push_back({"Pusher", sp.pusher_cost_eur, sp.push_footprint_length});
        p.cost_so_far_eur = sp.pusher_cost_eur;

        // Sub-task: transport from workflow source to task position.
        Task transport;
        transport.id = task.id + "/transport";
        transport.kind = TaskKind::Transport;
        transport.transport_distance_m  = std::abs(task.position - wf.source_position);
        transport.transport_rate        = task.rate_per_minute;
        transport.transport_source_pos  = wf.source_position;
        transport.position              = 0.5f * (wf.source_position + task.position);
        transport.mobility              = 0.f;  // belts are pinned by their endpoints
        p.remaining.push_back(transport);

        // Sub-task: detect when an item is at the pusher.
        Task detect;
        detect.id       = task.id + "/detect";
        detect.kind     = TaskKind::DetectPresence;
        detect.position = anchor;          // co-located with the pusher
        detect.mobility = 0.f;
        p.remaining.push_back(detect);

        // Floor on remaining cost: a cheap belt (just base) + a laser.
        p.cost_lb_eur =
            sp.pusher_cost_eur
          + sp.belt_base_cost_eur + sp.belt_per_metre_eur * transport.transport_distance_m
          + sp.laser_cost_eur;
        return {p};
    }
};

// ── BeltStrategy ────────────────────────────────────────────────────────────
//
// Solves Transport sub-tasks. Placed along the source-to-destination interval.

class BeltStrategy final : public Strategy {
public:
    const char* name() const override { return "BeltStrategy"; }
    bool can_solve(TaskKind k) const override { return k == TaskKind::Transport; }

    std::vector<Proposal> propose(const Task& task,
                                  const SolverState& state,
                                  const Workflow& /*wf*/,
                                  const StrategyParams& sp) const override
    {
        // Belt spans [source, source + distance]. Footprint = belt itself.
        const float a = task.transport_source_pos;
        const float b = a + task.transport_distance_m;

        Interval footprint{std::min(a, b), std::max(a, b)};
        // The belt is allowed to overlap with the pusher footprint at the end —
        // they're co-located. We won't check overlap strictly for belts (real
        // model would handle adjacency); simplification for v0 demo.

        const float cost = sp.belt_base_cost_eur
                         + sp.belt_per_metre_eur * task.transport_distance_m;

        Proposal p;
        p.strategy_name = "BeltStrategy";
        p.position      = 0.5f * (a + b);
        p.footprint_min = footprint.a;  p.footprint_max = footprint.b;
        p.work_area_min = footprint.a;  p.work_area_max = footprint.b;
        p.claims.push_back({task.id});
        char label[64];
        std::snprintf(label, sizeof(label), "Belt %.1fm", task.transport_distance_m);
        p.equipment.push_back({label, cost, footprint.length()});
        p.cost_so_far_eur = p.cost_lb_eur = cost;
        (void)state;
        return {p};
    }
};

// ── LaserStrategy ───────────────────────────────────────────────────────────

class LaserStrategy final : public Strategy {
public:
    const char* name() const override { return "LaserStrategy"; }
    bool can_solve(TaskKind k) const override { return k == TaskKind::DetectPresence; }

    std::vector<Proposal> propose(const Task& task,
                                  const SolverState& /*state*/,
                                  const Workflow& /*wf*/,
                                  const StrategyParams& sp) const override
    {
        const float a = task.position - 0.5f * sp.laser_footprint_length;
        const float b = task.position + 0.5f * sp.laser_footprint_length;

        Proposal p;
        p.strategy_name = "LaserStrategy";
        p.position      = task.position;
        p.footprint_min = a;  p.footprint_max = b;
        p.work_area_min = a;  p.work_area_max = b;
        p.claims.push_back({task.id});
        p.equipment.push_back({"Laser", sp.laser_cost_eur, sp.laser_footprint_length});
        p.cost_so_far_eur = p.cost_lb_eur = sp.laser_cost_eur;
        return {p};
    }
};

// ── Apply a proposal: update state ──────────────────────────────────────────

static SolverState apply(SolverState st, const Proposal& p,
                         const Workflow& wf)
{
    st.occupancy.push_back({p.footprint_min, p.footprint_max});
    for (const auto& claim : p.claims) {
        st.claimed_tasks.insert(claim.task_id);
        st.pinned_tasks.insert(claim.task_id);
    }
    // Push out any mobile, unclaimed tasks whose current position is now
    // inside the new footprint.
    Interval new_fp{p.footprint_min, p.footprint_max};
    for (auto& [tid, pos] : st.task_positions) {
        if (st.claimed_tasks.count(tid)) continue;
        if (st.pinned_tasks.count(tid)) continue;
        if (new_fp.contains(pos)) {
            pos = nearest_feasible_position(pos, st.occupancy,
                                            wf.source_position, wf.sink_position);
        }
    }
    st.cost_so_far_eur += p.cost_so_far_eur;
    st.applied.push_back(p);
    return st;
}

// ── ASCII layout printer ────────────────────────────────────────────────────

static void print_layout(const SolverState& st, const Workflow& wf,
                         const std::map<std::string, Task>& task_lookup,
                         const char* prefix = "    ")
{
    const float lo = wf.source_position, hi = wf.sink_position;
    const int   W  = 60;
    auto col = [&](float p) {
        const float t = (p - lo) / (hi - lo);
        return std::max(0, std::min(W - 1, int(t * (W - 1) + 0.5f)));
    };

    std::string line(W, ' ');
    auto put = [&](float p, char c) { line[col(p)] = c; };

    // Source / sink anchors
    put(wf.source_position, '|');
    put(wf.sink_position,   '|');

    // Occupancy (placed footprints) — '#'
    std::string occ(W, ' ');
    for (const auto& iv : st.occupancy) {
        const int a = col(iv.a), b = col(iv.b);
        for (int i = a; i <= b; ++i) occ[i] = '#';
    }
    // Work areas — '='
    std::string wa(W, ' ');
    for (const auto& applied : st.applied) {
        const int a = col(applied.work_area_min), b = col(applied.work_area_max);
        for (int i = a; i <= b; ++i) if (wa[i] == ' ') wa[i] = '=';
    }
    // Tasks
    std::string tasks_line(W, ' ');
    for (const auto& [tid, t] : task_lookup) {
        float pos = st.task_positions.count(tid) ? st.task_positions.at(tid) : t.position;
        char  ch  = st.claimed_tasks.count(tid) ? '*' : 'o';
        if (tasks_line[col(pos)] == ' ') tasks_line[col(pos)] = ch;
    }

    std::printf("%s%-8s %s\n", prefix, "tasks:", tasks_line.c_str());
    std::printf("%s%-8s %s\n", prefix, "workzn:", wa.c_str());
    std::printf("%s%-8s %s\n", prefix, "footpr:", occ.c_str());
    std::printf("%s%-8s %s\n", prefix, "axis:",   line.c_str());
    // ruler
    std::string ruler(W, '-');
    for (int i = 0; i < W; i += 10) {
        char buf[8]; std::snprintf(buf, sizeof(buf), "%d", int(lo + (hi - lo) * i / (W - 1) + 0.5f));
        for (size_t j = 0; j < std::strlen(buf) && i + j < (size_t)W; ++j) ruler[i + j] = buf[j];
    }
    std::printf("%s%-8s %s\n", prefix, "",  ruler.c_str());
}

static void print_proposal(const Proposal& p, const char* prefix = "  ") {
    std::printf("%s[%-13s] pos=%5.1f fp=[%5.1f..%5.1f] wa=[%5.1f..%5.1f] "
                "cost=%6.1f%s\n",
                prefix, p.strategy_name.c_str(), p.position,
                p.footprint_min, p.footprint_max,
                p.work_area_min, p.work_area_max,
                p.cost_so_far_eur,
                is_terminal(p) ? "  TERMINAL" : "");
    for (const auto& e : p.equipment) {
        std::printf("%s    + %-18s  €%7.1f  (fp=%.1f)\n",
                    prefix, e.label.c_str(), e.annual_cost_eur, e.footprint_length);
    }
    for (const auto& c : p.claims) {
        std::printf("%s    * claims %s\n", prefix, c.task_id.c_str());
    }
    for (const auto& t : p.remaining) {
        std::printf("%s    ? needs %s at %.1f\n",
                    prefix, task_name(t.kind), t.position);
    }
}

// ── Search (DFS with branch-and-bound) ──────────────────────────────────────

struct RunContext {
    float        best_cost  = std::numeric_limits<float>::infinity();
    SolverState  best_state;
    bool         verbose    = true;   // print the trace as we go
};

static Task pick_next_task(const SolverState& st,
                           const std::vector<Task>& all)
{
    for (const auto& t : all) {
        if (!st.claimed_tasks.count(t.id)) return t;
    }
    return Task{};
}

static void search_recursive(const SolverState& st,
                             const std::vector<Task>& root_tasks,
                             const std::vector<Task>& pending_subtasks,
                             const Workflow& wf,
                             const std::vector<std::unique_ptr<Strategy>>& strategies,
                             const StrategyParams& sp,
                             RunContext& ctx,
                             std::string indent)
{
    // Prune by best so far.
    if (st.cost_so_far_eur >= ctx.best_cost) return;

    // Complete?
    bool all_root_claimed = true;
    for (const auto& t : root_tasks) {
        if (!st.claimed_tasks.count(t.id)) { all_root_claimed = false; break; }
    }
    if (all_root_claimed && pending_subtasks.empty()) {
        if (st.cost_so_far_eur < ctx.best_cost) {
            ctx.best_cost  = st.cost_so_far_eur;
            ctx.best_state = st;
            if (ctx.verbose) {
                std::printf("%s>> COMPLETE solution cost=%.1f (best so far)\n",
                            indent.c_str(), st.cost_so_far_eur);
            }
        }
        return;
    }

    // Pick next task (sub-task pending first, else next unclaimed root).
    Task next;
    std::vector<Task> remaining_sub = pending_subtasks;
    if (!remaining_sub.empty()) {
        next = remaining_sub.front();
        remaining_sub.erase(remaining_sub.begin());
    } else {
        next = pick_next_task(st, root_tasks);
        if (next.id.empty()) return;
    }

    if (ctx.verbose) {
        std::printf("%sDispatch %s (%s) at %.1f\n",
                    indent.c_str(), next.id.c_str(), task_name(next.kind), next.position);
    }

    // Gather all proposals from applicable strategies.
    std::vector<Proposal> all_proposals;
    for (const auto& s : strategies) {
        if (!s->can_solve(next.kind)) continue;
        auto props = s->propose(next, st, wf, sp);
        for (auto& p : props) all_proposals.push_back(std::move(p));
    }
    if (all_proposals.empty()) {
        if (ctx.verbose) {
            std::printf("%s  (no strategy yields a feasible proposal)\n", indent.c_str());
        }
        return;
    }

    // Sort by cost-so-far ascending so we explore cheap branches first.
    std::sort(all_proposals.begin(), all_proposals.end(),
        [](const Proposal& a, const Proposal& b){
            return a.cost_so_far_eur < b.cost_so_far_eur;
        });

    for (const auto& p : all_proposals) {
        if (ctx.verbose) {
            std::printf("%s  Try:\n", indent.c_str());
            print_proposal(p, (indent + "    ").c_str());
        }
        SolverState ns = apply(st, p, wf);
        std::vector<Task> new_pending = remaining_sub;
        for (const auto& sub : p.remaining) new_pending.push_back(sub);
        search_recursive(ns, root_tasks, new_pending, wf, strategies, sp,
                         ctx, indent + "  ");
    }
}

// Run the inner search for one specific assignment of task positions.
// `wf` carries the assigned positions; returns the best state + cost found.
static RunContext run_inner_search(const Workflow& wf,
                                   const std::vector<std::unique_ptr<Strategy>>& strategies,
                                   const StrategyParams& sp,
                                   bool verbose)
{
    SolverState s0;
    for (const auto& t : wf.tasks) {
        s0.task_positions[t.id] = t.position;
        if (t.mobility == 0.f) s0.pinned_tasks.insert(t.id);
    }
    RunContext ctx;
    ctx.verbose = verbose;
    search_recursive(s0, wf.tasks, {}, wf, strategies, sp, ctx, "");
    return ctx;
}

// ── Outer search: enumerate task position combinations ─────────────────────
//
// Tasks are declared in workflow order. The first task sits between source
// and second task; the second between first and third; ...; the last
// between previous and sink. Outer loop is a grid enumeration over those
// constrained positions. For each combination, run the inner search and
// remember its result.

struct OuterTrial {
    std::vector<std::pair<std::string, float>> positions;
    float                                       cost;
};

static void enumerate_positions(std::vector<Task>& tasks,
                                size_t             idx,
                                float              lower_bound,
                                float              upper_bound,
                                float              grid_step,
                                float              edge_margin,
                                std::vector<std::vector<float>>& out)
{
    if (idx == tasks.size()) {
        std::vector<float> snapshot(tasks.size());
        for (size_t i = 0; i < tasks.size(); ++i) snapshot[i] = tasks[i].position;
        out.push_back(std::move(snapshot));
        return;
    }
    const float lo = lower_bound + edge_margin;
    const float hi = upper_bound - edge_margin;
    // Reserve room for remaining tasks (each needs > edge_margin headroom).
    const size_t remaining_after = tasks.size() - idx - 1;
    const float  reserve_right   = edge_margin * remaining_after
                                 + grid_step  * remaining_after;
    for (float pos = lo; pos <= hi - reserve_right + 1e-4f; pos += grid_step) {
        tasks[idx].position = pos;
        enumerate_positions(tasks, idx + 1, pos, upper_bound,
                            grid_step, edge_margin, out);
    }
}

// ── JSON loader ─────────────────────────────────────────────────────────────

static StrategyParams load_strategy_params(const nlohmann::json& j) {
    StrategyParams sp;
    if (j.contains("arm")) {
        const auto& a = j["arm"];
        sp.arm_cost_each_eur       = a.value("cost_each_eur",       sp.arm_cost_each_eur);
        sp.arm_max_rate_per_minute = a.value("max_rate_per_minute", sp.arm_max_rate_per_minute);
        sp.arm_max_count           = a.value("max_count",           sp.arm_max_count);
        sp.arm_reach               = a.value("reach",               sp.arm_reach);
        sp.arm_footprint_length    = a.value("footprint_length",    sp.arm_footprint_length);
    }
    if (j.contains("push")) {
        const auto& p = j["push"];
        sp.pusher_cost_eur          = p.value("pusher_cost_eur",     sp.pusher_cost_eur);
        sp.push_max_rate_per_minute = p.value("max_rate_per_minute", sp.push_max_rate_per_minute);
        sp.push_footprint_length    = p.value("footprint_length",    sp.push_footprint_length);
    }
    if (j.contains("belt")) {
        const auto& b = j["belt"];
        sp.belt_base_cost_eur       = b.value("base_cost_eur",       sp.belt_base_cost_eur);
        sp.belt_per_metre_eur       = b.value("per_metre_eur",       sp.belt_per_metre_eur);
        sp.belt_footprint_per_metre = b.value("footprint_per_metre", sp.belt_footprint_per_metre);
    }
    if (j.contains("laser")) {
        const auto& l = j["laser"];
        sp.laser_cost_eur            = l.value("cost_eur",         sp.laser_cost_eur);
        sp.laser_footprint_length    = l.value("footprint_length", sp.laser_footprint_length);
    }
    return sp;
}

static Workflow load_workflow(const nlohmann::json& doc) {
    Workflow wf;
    if (doc.contains("workflow")) {
        wf.source_position = doc["workflow"].value("source_position", wf.source_position);
        wf.sink_position   = doc["workflow"].value("sink_position",   wf.sink_position);
    }
    for (const auto& tj : doc.at("tasks")) {
        Task t;
        t.id              = tj.at("id").get<std::string>();
        t.kind            = kind_from_string(tj.at("kind").get<std::string>());
        t.position        = tj.value("position",        0.f);
        t.mobility        = tj.value("mobility",        0.f);
        t.rate_per_minute = tj.value("rate_per_minute", 0.f);
        wf.tasks.push_back(t);
    }
    return wf;
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

    Workflow       wf = load_workflow(doc);
    StrategyParams sp = load_strategy_params(doc.value("strategies", nlohmann::json::object()));

    // Outer-loop tuning.
    const float grid_step   = doc.value("outer_grid_step",   2.0f);
    const float edge_margin = doc.value("outer_edge_margin", 1.0f);

    std::printf("Workflow:\n");
    std::printf("  Source @ %.1f, Sink @ %.1f\n", wf.source_position, wf.sink_position);
    std::printf("  Tasks (positions to be SOLVED, not declared):\n");
    for (const auto& t : wf.tasks) {
        std::printf("    %s (%s)  rate=%.1f/min\n",
                    t.id.c_str(), task_name(t.kind), t.rate_per_minute);
    }
    std::printf("  Outer grid step = %.1f, edge margin = %.1f\n",
                grid_step, edge_margin);

    std::map<std::string, Task> task_lookup;
    for (const auto& t : wf.tasks) task_lookup[t.id] = t;

    // Strategies
    std::vector<std::unique_ptr<Strategy>> strategies;
    strategies.push_back(std::make_unique<ArmStrategy>());
    strategies.push_back(std::make_unique<PushStrategy>());
    strategies.push_back(std::make_unique<BeltStrategy>());
    strategies.push_back(std::make_unique<LaserStrategy>());

    // Enumerate all topologically-valid position combinations.
    std::vector<std::vector<float>> all_positions;
    std::vector<Task> tasks_scratch = wf.tasks;
    enumerate_positions(tasks_scratch, 0,
                        wf.source_position, wf.sink_position,
                        grid_step, edge_margin, all_positions);

    std::printf("\n=== Outer search: %zu task-position combinations ===\n",
                all_positions.size());

    // Run inner search for each combination, silently.
    std::vector<OuterTrial> trials;
    trials.reserve(all_positions.size());
    Workflow w_iter = wf;
    for (const auto& positions : all_positions) {
        for (size_t i = 0; i < w_iter.tasks.size(); ++i) {
            w_iter.tasks[i].position = positions[i];
        }
        RunContext rc = run_inner_search(w_iter, strategies, sp, /*verbose=*/false);
        OuterTrial trial;
        trial.cost = rc.best_cost;
        for (size_t i = 0; i < w_iter.tasks.size(); ++i) {
            trial.positions.emplace_back(w_iter.tasks[i].id, positions[i]);
        }
        trials.push_back(std::move(trial));
    }

    // Sort by cost ascending. Infinity (= infeasible) sinks to the end.
    std::sort(trials.begin(), trials.end(),
        [](const OuterTrial& a, const OuterTrial& b){ return a.cost < b.cost; });

    // Compact summary: top 5 cheapest + bottom 3 + a feasibility headcount.
    auto print_row = [&](const OuterTrial& t) {
        for (const auto& [id, pos] : t.positions) std::printf("%s=%.1f  ", id.c_str(), pos);
        if (std::isfinite(t.cost)) std::printf("→ €%.1f\n", t.cost);
        else                       std::printf("→ INFEASIBLE\n");
    };
    int feasible = 0;
    for (const auto& t : trials) if (std::isfinite(t.cost)) ++feasible;

    std::printf("\nFeasible: %d / %zu\n", feasible, trials.size());
    std::printf("\nTop 5 cheapest:\n");
    for (size_t i = 0; i < trials.size() && i < 5; ++i) {
        std::printf("  ");
        print_row(trials[i]);
    }
    std::printf("\nTop 3 most expensive (feasible only):\n");
    int shown = 0;
    for (size_t i = trials.size(); i > 0 && shown < 3; --i) {
        const auto& t = trials[i - 1];
        if (!std::isfinite(t.cost)) continue;
        std::printf("  ");
        print_row(t);
        ++shown;
    }

    if (trials.empty() || !std::isfinite(trials.front().cost)) {
        std::printf("\nNo feasible configuration found.\n");
        return 2;
    }

    // Re-run the cheapest combination WITH verbose trace + final layout.
    const auto& winner = trials.front();
    Workflow w_win = wf;
    for (auto& t : w_win.tasks) {
        for (const auto& [id, pos] : winner.positions) {
            if (t.id == id) t.position = pos;
        }
    }
    std::printf("\n========================================\n");
    std::printf("Detailed trace for the winning configuration:\n  ");
    print_row(winner);
    std::printf("\n");
    RunContext final_run = run_inner_search(w_win, strategies, sp, /*verbose=*/true);

    std::printf("\n========================================\n");
    std::printf("WINNING SOLUTION  €%.1f/year\n", final_run.best_cost);
    std::printf("\nFinal layout:\n");
    print_layout(final_run.best_state, w_win, task_lookup);
    std::printf("\nEquipment:\n");
    for (const auto& p : final_run.best_state.applied) {
        std::printf("  [%s] pos=%.1f\n", p.strategy_name.c_str(), p.position);
        for (const auto& e : p.equipment) {
            std::printf("    • %-20s  €%7.1f  fp=%.1f m\n",
                        e.label.c_str(), e.annual_cost_eur, e.footprint_length);
        }
    }
    return 0;
}
