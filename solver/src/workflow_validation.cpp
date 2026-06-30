#include "tinycell/solver/workflow_validation.hpp"

#include <algorithm>
#include <set>
#include <unordered_map>
#include <utility>

#include <tinycell/model/port.hpp>

namespace tinycell::solver {

namespace {
namespace tc = tinycell::core;

const char* dir_name(tc::PortDirection d) {
    return d == tc::PortDirection::Input ? "input" : "output";
}

// Find the logical port named `port_name` on a task's port set; nullptr
// if the task declares no such port.
const tc::LogicalPort* find_port(const std::vector<tc::LogicalPort>& ports,
                                 const std::string& port_name) {
    for (const auto& p : ports) {
        if (p.name == port_name) return &p;
    }
    return nullptr;
}

} // namespace

bool WorkflowValidation::ok() const { return error_count() == 0; }

std::size_t WorkflowValidation::error_count() const {
    return static_cast<std::size_t>(std::count_if(
        issues.begin(), issues.end(), [](const WorkflowIssue& i) {
            return i.severity == WorkflowIssueSeverity::Error;
        }));
}

std::size_t WorkflowValidation::warning_count() const {
    return static_cast<std::size_t>(std::count_if(
        issues.begin(), issues.end(), [](const WorkflowIssue& i) {
            return i.severity == WorkflowIssueSeverity::Warning;
        }));
}

WorkflowValidation validate_workflow(std::span<const tc::Task> workflow) {
    WorkflowValidation result;
    auto err = [&](std::string task_id, std::string msg) {
        result.issues.push_back(
            {WorkflowIssueSeverity::Error, std::move(task_id), std::move(msg)});
    };
    auto warn = [&](std::string task_id, std::string msg) {
        result.issues.push_back(
            {WorkflowIssueSeverity::Warning, std::move(task_id), std::move(msg)});
    };

    // 1. Index tasks by id; report duplicates (first occurrence kept as
    //    the canonical binding so later checks still resolve).
    std::unordered_map<std::string, std::size_t> id_to_index;
    for (std::size_t i = 0; i < workflow.size(); ++i) {
        const auto& id = workflow[i].id;
        if (!id_to_index.try_emplace(id, i).second) {
            err(id, "duplicate task id '" + id + "' — task references are ambiguous");
        }
    }

    // 2. For every Transport edge: record which (task, port) endpoints it
    //    connects, and check that its source/sink references resolve to an
    //    existing task + port with the correct polarity (port.hpp: a
    //    source must be an OUTPUT, a sink must be an INPUT).
    std::set<std::pair<std::string, std::string>> connected_outputs;  // transport sources
    std::set<std::pair<std::string, std::string>> connected_inputs;   // transport sinks

    for (const auto& task : workflow) {
        if (task.kind() != tc::TaskKind::Transport) continue;
        const auto& tp = std::get<tc::TransportParams>(task.params);

        // A transport must move material BETWEEN two tasks. A self-loop
        // (source task == sink task) is physically nonsensical, and if it
        // were counted it would mark the task's own input as "fed" by its
        // own output — silently masking a genuine unfed-input gap. Reject
        // it and skip recording its connectivity, so it neither validates
        // nor satisfies any port (the real unfed/undrained issue still
        // surfaces in step 3).
        if (tp.source_task_id == tp.sink_task_id) {
            err(task.id, "transport '" + task.id + "' connects task '" +
                             tp.source_task_id +
                             "' to itself (source and sink are the same task)");
            continue;
        }

        connected_outputs.emplace(tp.source_task_id, tp.source_port_name);
        connected_inputs.emplace(tp.sink_task_id, tp.sink_port_name);

        auto check_endpoint = [&](const std::string& ref_task_id,
                                  const std::string& ref_port_name,
                                  tc::PortDirection required, const char* end) {
            const auto it = id_to_index.find(ref_task_id);
            if (it == id_to_index.end()) {
                err(task.id, "transport '" + task.id + "' " + end +
                                 " names unknown task '" + ref_task_id + "'");
                return;
            }
            const auto ports = tc::task_ports(workflow[it->second]);
            const auto* port = find_port(ports, ref_port_name);
            if (port == nullptr) {
                err(task.id, "transport '" + task.id + "' " + end + " port '" +
                                 ref_port_name + "' does not exist on task '" +
                                 ref_task_id + "'");
                return;
            }
            if (port->direction != required) {
                err(task.id, "transport '" + task.id + "' " + end + " port '" +
                                 ref_task_id + "." + ref_port_name + "' is an " +
                                 dir_name(port->direction) + " but must be an " +
                                 dir_name(required));
            }
        };
        check_endpoint(tp.source_task_id, tp.source_port_name,
                       tc::PortDirection::Output, "source");
        check_endpoint(tp.sink_task_id, tp.sink_port_name,
                       tc::PortDirection::Input, "sink");
    }

    // 3. Connectivity: every INPUT logical port must be fed by some
    //    transport (Error — nothing arrives there), every OUTPUT must be
    //    drained by some transport (Warning — produced material with
    //    nowhere to go). Transport tasks declare no ports of their own and
    //    fall through (task_ports returns empty).
    for (const auto& task : workflow) {
        for (const auto& port : tc::task_ports(task)) {
            const auto key = std::make_pair(task.id, port.name);
            if (port.direction == tc::PortDirection::Input) {
                if (!connected_inputs.contains(key)) {
                    err(task.id, "input port '" + task.id + "." + port.name +
                                     "' has no transport feeding it");
                }
            } else {
                if (!connected_outputs.contains(key)) {
                    warn(task.id, "output port '" + task.id + "." + port.name +
                                      "' has no transport draining it");
                }
            }
        }
    }

    return result;
}

} // namespace tinycell::solver
