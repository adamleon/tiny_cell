#pragma once

#include <stdexcept>
#include <string>
#include <tinycell/units.hpp>
#include <variant>

namespace tinycell::core {

struct BoxSpec {
    Length width;
    Length length;
    Length height;
    Mass mass;
};

struct PalletSpec {
    Length width;
    Length length;
};

// Task kind == one of these param variants. Each kind names a *goal*, never the
// equipment used to achieve it (decisions.md — strategy class reuse).
struct PalletizeParams {
    std::string item_id;
    BoxSpec item;
    PalletSpec pallet;
    int box_count;
};

using TaskParams = std::variant<PalletizeParams>;

enum class TaskKind { Palletize };

struct Task {
    std::string id;
    TaskParams params;

    TaskKind kind() const {
        if (std::holds_alternative<PalletizeParams>(params)) {
            return TaskKind::Palletize;
        }
        throw std::logic_error("tinycell::core::Task::kind(): unknown variant alternative");
    }
};

} // namespace tinycell::core
