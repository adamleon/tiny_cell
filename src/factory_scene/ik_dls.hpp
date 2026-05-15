#pragma once
#include <array>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <vector>

#include <threepp/math/Matrix4.hpp>
#include <threepp/math/Quaternion.hpp>
#include <threepp/math/Vector3.hpp>
#include <threepp/objects/Robot.hpp>

#include "ik.hpp"

namespace factory::ik {

// ── Damped least-squares 6-DOF IK using threepp::Robot's FK ─────────────────
//
// Iterates Δq = (Jᵀ J + λ²I)⁻¹ Jᵀ e on a 6-vector error of (position, axis×angle
// orientation). Numerical Jacobian computed by perturbing each joint and
// reading the full end-effector transform from threepp's Robot::compute­End­
// EffectorTransform — so the URDF's joint origins (including rpy) are
// honoured automatically, no parallel kinematic model required.
//
// Frames:
//   - API Target is in ECS world coords (Z-up, mm). The solver converts to
//     threepp world coords (Y-up, m) internally for FK comparison.
//   - threepp::Robot is expected to be placed and oriented in the scene; the
//     solver respects its world transform.
//
// Convergence settings are exposed as members for tuning; defaults work for
// our KUKA-class arms at visualization quality (~60Hz).

class DLSSolver final : public Solver {
public:
    explicit DLSSolver(std::shared_ptr<threepp::Robot> robot)
        : robot_(std::move(robot))
    {
        if (robot_) joint_values_buffer_.assign(robot_->numDOF(), 0.f);
    }

    size_t dof() const override {
        return robot_ ? robot_->numDOF() : 0;
    }

    // Tunables
    float damping        = 0.08f;   // λ; larger = more stable, less precise
    float position_eps_m = 1e-3f;   // 1 mm
    float rot_eps_rad    = 1e-2f;   // ~0.6°
    int   max_iter       = 30;
    float fd_eps_rad     = 1e-3f;   // finite-difference perturbation per joint

    std::vector<float> solve(const Target& target,
                             const std::vector<float>& initial_joints) override
    {
        if (!robot_ || dof() == 0) return initial_joints;

        const size_t n = dof();
        std::vector<float> q = initial_joints;
        if (q.size() != n) q.assign(n, 0.f);

        const bool   use_orient = target.orientation.has_value();
        const size_t m          = use_orient ? 6 : 3;

        // Target in threepp world frame (Y-up, m). ECS → threepp: swap Y/Z,
        // mm → m.
        const threepp::Vector3 tgt_pos = ecs_pos_to_threepp(target.position);
        threepp::Quaternion tgt_quat;
        if (use_orient) tgt_quat = ecs_quat_to_threepp(*target.orientation);

        for (int iter = 0; iter < max_iter; ++iter) {
            // Current pose at q
            threepp::Vector3 cur_pos;
            threepp::Quaternion cur_quat;
            fk_pose(q, cur_pos, cur_quat);

            // Build error vector (3 or 6 entries)
            std::array<float, 6> e{};
            e[0] = tgt_pos.x - cur_pos.x;
            e[1] = tgt_pos.y - cur_pos.y;
            e[2] = tgt_pos.z - cur_pos.z;
            if (use_orient) {
                const auto ang_err = quat_axis_angle_error(tgt_quat, cur_quat);
                e[3] = ang_err[0];
                e[4] = ang_err[1];
                e[5] = ang_err[2];
            }

            const float pos_err = std::sqrt(e[0]*e[0] + e[1]*e[1] + e[2]*e[2]);
            const float rot_err = use_orient
                ? std::sqrt(e[3]*e[3] + e[4]*e[4] + e[5]*e[5]) : 0.f;
            if (pos_err < position_eps_m && rot_err < rot_eps_rad) break;

            // Numerical Jacobian via central differences: 2 FK calls per
            // column. We pass enforceLimits=false to the FK so a joint at
            // its limit still produces a meaningful perturbation — otherwise
            // threepp clamps q[i]±ε back to the limit and the column comes
            // back as zero, making that joint look immovable to the solver.
            std::vector<std::array<float, 6>> J(n);
            for (size_t i = 0; i < n; ++i) {
                const float saved = q[i];

                q[i] = saved + fd_eps_rad;
                threepp::Vector3 p_plus;
                threepp::Quaternion qr_plus;
                fk_pose_unclamped(q, p_plus, qr_plus);

                q[i] = saved - fd_eps_rad;
                threepp::Vector3 p_minus;
                threepp::Quaternion qr_minus;
                fk_pose_unclamped(q, p_minus, qr_minus);

                q[i] = saved;

                const float two_eps = 2.f * fd_eps_rad;
                J[i][0] = (p_plus.x - p_minus.x) / two_eps;
                J[i][1] = (p_plus.y - p_minus.y) / two_eps;
                J[i][2] = (p_plus.z - p_minus.z) / two_eps;
                if (use_orient) {
                    const auto col_ang = quat_axis_angle_error(qr_plus, qr_minus);
                    J[i][3] = col_ang[0] / two_eps;
                    J[i][4] = col_ang[1] / two_eps;
                    J[i][5] = col_ang[2] / two_eps;
                }
            }

            // Build (JᵀJ + λ²I) — n×n — and Jᵀe — n
            std::vector<float> A(n * n, 0.f);
            std::vector<float> b(n, 0.f);
            for (size_t i = 0; i < n; ++i) {
                for (size_t j = 0; j < n; ++j) {
                    float s = 0.f;
                    for (size_t k = 0; k < m; ++k) s += J[i][k] * J[j][k];
                    A[i * n + j] = s;
                }
                A[i * n + i] += damping * damping;
                float bi = 0.f;
                for (size_t k = 0; k < m; ++k) bi += J[i][k] * e[k];
                b[i] = bi;
            }

            // Solve A Δq = b via Gauss-Jordan elimination
            if (!solve_linear(A.data(), b.data(), n)) break;

            // Apply step with joint-limit clamping
            const auto infos = robot_->getArticulatedJointInfo();
            for (size_t i = 0; i < n; ++i) {
                float v = q[i] + b[i];
                if (i < infos.size() && infos[i].range.has_value())
                    v = infos[i].range->clamp(v);
                q[i] = v;
            }
        }
        return q;
    }

private:
    std::shared_ptr<threepp::Robot> robot_;
    std::vector<float>              joint_values_buffer_;

    // ECS (Z-up, mm) → threepp (Y-up, m). Matches existing render-loop swap.
    static threepp::Vector3 ecs_pos_to_threepp(const Vec3& v) {
        return threepp::Vector3(v.x * 0.001f, v.z * 0.001f, v.y * 0.001f);
    }
    static threepp::Quaternion ecs_quat_to_threepp(const Quat& q) {
        // Same Y/Z swap on the quaternion's imaginary part.
        return threepp::Quaternion(q.x, q.z, q.y, q.w);
    }

    // FK at joint config q: pull position and orientation out of the
    // 4×4 returned by threepp's Robot.
    void fk_pose(const std::vector<float>& q,
                 threepp::Vector3& out_pos, threepp::Quaternion& out_quat) const
    {
        const auto m = robot_->computeEndEffectorTransform(q, /*deg*/ false, true);
        const auto& el = m.elements;
        out_pos.set(el[12], el[13], el[14]);
        out_quat.setFromRotationMatrix(m);
    }

    // FK without joint-limit enforcement — used inside Jacobian computation
    // so that ε-perturbing a joint near its limit still produces a non-zero
    // column. The final solver step still clamps the result.
    void fk_pose_unclamped(const std::vector<float>& q,
                           threepp::Vector3& out_pos,
                           threepp::Quaternion& out_quat) const
    {
        const auto m = robot_->computeEndEffectorTransform(q, /*deg*/ false, /*enforce*/ false);
        const auto& el = m.elements;
        out_pos.set(el[12], el[13], el[14]);
        out_quat.setFromRotationMatrix(m);
    }

    // axis × angle representation of the rotation a · b⁻¹.
    // Uses the small-angle linearisation (2 * imag part of the relative quat)
    // — accurate for the small errors that show up near convergence.
    static std::array<float, 3>
    quat_axis_angle_error(const threepp::Quaternion& target,
                          const threepp::Quaternion& current)
    {
        const float cx = -current.x, cy = -current.y, cz = -current.z, cw = current.w;
        // delta = target * conj(current)
        threepp::Quaternion d(
            target.w * cx + target.x * cw + target.y * cz - target.z * cy,
            target.w * cy - target.x * cz + target.y * cw + target.z * cx,
            target.w * cz + target.x * cy - target.y * cx + target.z * cw,
            target.w * cw - target.x * (-cx) - target.y * (-cy) - target.z * (-cz)
        );
        // Shortest-path: flip if w < 0
        float dw = d.w, dx = d.x, dy = d.y, dz = d.z;
        if (dw < 0.f) { dw = -dw; dx = -dx; dy = -dy; dz = -dz; }
        // axis*angle ≈ 2 * imaginary part for small angles
        return {2.f * dx, 2.f * dy, 2.f * dz};
    }

    // Gauss-Jordan elimination on n×n system A x = b, result in b.
    // Returns false if singular.
    static bool solve_linear(float* A, float* b, size_t n) {
        for (size_t i = 0; i < n; ++i) {
            // Partial pivot
            size_t pivot = i;
            float  pmax  = std::abs(A[i * n + i]);
            for (size_t r = i + 1; r < n; ++r) {
                const float v = std::abs(A[r * n + i]);
                if (v > pmax) { pmax = v; pivot = r; }
            }
            if (pmax < 1e-12f) return false;
            if (pivot != i) {
                for (size_t c = 0; c < n; ++c) std::swap(A[i*n+c], A[pivot*n+c]);
                std::swap(b[i], b[pivot]);
            }
            // Normalize row
            const float div = A[i * n + i];
            for (size_t c = 0; c < n; ++c) A[i * n + c] /= div;
            b[i] /= div;
            // Eliminate
            for (size_t r = 0; r < n; ++r) {
                if (r == i) continue;
                const float f = A[r * n + i];
                if (f == 0.f) continue;
                for (size_t c = 0; c < n; ++c) A[r * n + c] -= f * A[i * n + c];
                b[r] -= f * b[i];
            }
        }
        return true;
    }
};

inline std::unique_ptr<Solver> make_dls_solver(std::shared_ptr<threepp::Robot> robot) {
    return std::make_unique<DLSSolver>(std::move(robot));
}

}  // namespace factory::ik
