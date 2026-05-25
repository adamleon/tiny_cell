// Intent: prove NLopt's SLSQP backend builds + links + solves a trivial
// constrained NLP on Windows via vcpkg. Same problem as ipopt_smoke.cpp
// (same constraint, same analytic optimum) so the libraries are compared
// on an identical task:
//
//     minimize    x² + y²
//     subject to  x + y >= 5
//
// Closed form: (x*, y*) = (2.5, 2.5), f* = 12.5.
//
// NLopt expresses inequalities as h(x) <= 0, so the constraint becomes
//     h(x) = 5 - x - y <= 0

#include <nlopt.hpp>

#include <cmath>
#include <cstdio>
#include <vector>

namespace {

double objective(const std::vector<double>& x, std::vector<double>& grad, void*) {
    if (!grad.empty()) {
        grad[0] = 2.0 * x[0];
        grad[1] = 2.0 * x[1];
    }
    return x[0]*x[0] + x[1]*x[1];
}

double constraint(const std::vector<double>& x, std::vector<double>& grad, void*) {
    if (!grad.empty()) {
        grad[0] = -1.0;
        grad[1] = -1.0;
    }
    return 5.0 - x[0] - x[1];
}

} // namespace

int main() {
    try {
        nlopt::opt opt(nlopt::LD_SLSQP, 2);
        opt.set_min_objective(objective, nullptr);
        opt.add_inequality_constraint(constraint, nullptr, 1e-12);
        opt.set_xtol_rel(1e-9);
        opt.set_ftol_rel(1e-12);
        opt.set_maxeval(200);

        std::vector<double> x = {0.0, 0.0};
        double f_final = 0.0;
        auto result = opt.optimize(x, f_final);

        std::printf("NLopt/SLSQP result: x=%.9f y=%.9f f=%.9f (result code=%d)\n",
                    x[0], x[1], f_final, static_cast<int>(result));

        constexpr double tol = 1e-4;
        const bool ok =
            std::abs(x[0] - 2.5) < tol &&
            std::abs(x[1] - 2.5) < tol &&
            std::abs(f_final - 12.5) < tol;

        if (!ok) {
            std::fprintf(stderr, "NLopt/SLSQP: optimum off the analytic answer (2.5, 2.5, 12.5)\n");
            return 3;
        }

        std::printf("NLopt/SLSQP: OK (matches analytic optimum within %.0e)\n", tol);
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "NLopt/SLSQP: exception during solve: %s\n", e.what());
        return 2;
    }
}
