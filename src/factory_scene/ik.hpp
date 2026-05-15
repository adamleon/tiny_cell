#pragma once
#include <optional>
#include <vector>
#include "pose_component.hpp"

namespace factory::ik {

// Target end-effector pose. Position is required; orientation is optional —
// leave it unset for position-only IK (solver minimises only the 3-D position
// error, picking any wrist orientation that fits). Set it when you actually
// want the tool aligned (e.g., gripper pointing down for a pick).
//
// Frames: ECS world coordinates (Z-up, mm) and (when set) ECS world
// orientation. Solver implementations convert internally.
struct Target {
    Vec3                 position;
    std::optional<Quat>  orientation;
};

// Abstract IK solver. Backed by DLS / TRAC-IK / Kine / etc. behind the same
// interface. Caller owns the solver instance for the lifetime of its robot.
class Solver {
public:
    virtual ~Solver() = default;

    // Solve for joint values that put the end-effector at `target`. Best-effort
    // when out of reach. `initial_joints` is a warm-start (typically the current
    // joint state); its length must equal `dof()`. Returned vector has length
    // `dof()`.
    virtual std::vector<float> solve(const Target& target,
                                     const std::vector<float>& initial_joints) = 0;

    virtual size_t dof() const = 0;
};

}  // namespace factory::ik
