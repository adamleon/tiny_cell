// Intent: prove IPOPT builds + links + solves a trivial constrained NLP
// on Windows via vcpkg. Problem chosen so the constraint is active at
// the optimum (rules out "solver returned the unconstrained min and we
// didn't notice"):
//
//     minimize    x² + y²
//     subject to  x + y >= 5
//
// Closed form: (x*, y*) = (2.5, 2.5), f* = 12.5.
//
// Success = converges to that point within tolerance. Any build/link
// pain on the way to this binary is the actual deliverable of the
// spike, not the numerical answer.

#include <IpIpoptApplication.hpp>
#include <IpTNLP.hpp>

#include <cmath>
#include <cstdio>
#include <cstdlib>

using Ipopt::Index;
using Ipopt::Number;

namespace {

class TrivialNlp final : public Ipopt::TNLP {
public:
    Number x_final = 0.0;
    Number y_final = 0.0;
    Number f_final = 0.0;

    bool get_nlp_info(Index& n, Index& m, Index& nnz_jac_g, Index& nnz_h_lag,
                      IndexStyleEnum& index_style) override {
        n = 2;            // (x, y)
        m = 1;            // x + y >= 5
        nnz_jac_g = 2;    // dense 1x2 jacobian
        nnz_h_lag = 2;    // diag(2,2) — lower triangle
        index_style = TNLP::C_STYLE;
        return true;
    }

    bool get_bounds_info(Index n, Number* x_l, Number* x_u, Index m,
                         Number* g_l, Number* g_u) override {
        for (Index i = 0; i < n; ++i) {
            x_l[i] = -1e19;
            x_u[i] =  1e19;
        }
        g_l[0] = 5.0;
        g_u[0] = 1e19;
        return true;
    }

    bool get_starting_point(Index n, bool init_x, Number* x, bool, Number*, Number*,
                            Index, bool, Number*) override {
        if (init_x) {
            x[0] = 0.0;
            x[1] = 0.0;
        }
        return true;
    }

    bool eval_f(Index, const Number* x, bool, Number& obj_value) override {
        obj_value = x[0]*x[0] + x[1]*x[1];
        return true;
    }

    bool eval_grad_f(Index, const Number* x, bool, Number* grad_f) override {
        grad_f[0] = 2.0 * x[0];
        grad_f[1] = 2.0 * x[1];
        return true;
    }

    bool eval_g(Index, const Number* x, bool, Index, Number* g) override {
        g[0] = x[0] + x[1];
        return true;
    }

    bool eval_jac_g(Index, const Number*, bool, Index, Index nele_jac,
                    Index* iRow, Index* jCol, Number* values) override {
        if (values == nullptr) {
            iRow[0] = 0; jCol[0] = 0;
            iRow[1] = 0; jCol[1] = 1;
        } else {
            values[0] = 1.0;
            values[1] = 1.0;
        }
        (void)nele_jac;
        return true;
    }

    bool eval_h(Index, const Number*, bool, Number obj_factor, Index,
                const Number*, bool, Index nele_hess, Index* iRow, Index* jCol,
                Number* values) override {
        if (values == nullptr) {
            iRow[0] = 0; jCol[0] = 0;
            iRow[1] = 1; jCol[1] = 1;
        } else {
            values[0] = 2.0 * obj_factor;
            values[1] = 2.0 * obj_factor;
        }
        (void)nele_hess;
        return true;
    }

    void finalize_solution(Ipopt::SolverReturn, Index, const Number* x, const Number*,
                           const Number*, Index, const Number*, const Number*,
                           Number obj_value, const Ipopt::IpoptData*,
                           Ipopt::IpoptCalculatedQuantities*) override {
        x_final = x[0];
        y_final = x[1];
        f_final = obj_value;
    }
};

} // namespace

int main() {
    auto nlp = new TrivialNlp();
    Ipopt::SmartPtr<Ipopt::TNLP> tnlp(nlp);

    Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();
    app->Options()->SetNumericValue("tol", 1e-9);
    // vcpkg's coin-or-ipopt doesn't ship libhsl.dll (HSL is closed-source
    // and not redistributable); default linear_solver=ma27 fails at
    // runtime with "DLL not found". MUMPS is the FOSS fallback that IS
    // bundled. Explicit pick — finding for the spike report.
    app->Options()->SetStringValue("linear_solver", "mumps");
    app->Options()->SetStringValue("sb", "yes");
    app->Options()->SetIntegerValue("print_level", 0);

    auto status = app->Initialize();
    if (status != Ipopt::Solve_Succeeded) {
        std::fprintf(stderr, "IPOPT: initialize failed (status=%d)\n", static_cast<int>(status));
        return 1;
    }

    status = app->OptimizeTNLP(tnlp);
    if (status != Ipopt::Solve_Succeeded && status != Ipopt::Solved_To_Acceptable_Level) {
        std::fprintf(stderr, "IPOPT: solve failed (status=%d)\n", static_cast<int>(status));
        return 2;
    }

    std::printf("IPOPT result: x=%.9f y=%.9f f=%.9f\n", nlp->x_final, nlp->y_final, nlp->f_final);

    constexpr double tol = 1e-4;
    const bool ok =
        std::abs(nlp->x_final - 2.5) < tol &&
        std::abs(nlp->y_final - 2.5) < tol &&
        std::abs(nlp->f_final - 12.5) < tol;

    if (!ok) {
        std::fprintf(stderr, "IPOPT: optimum off the analytic answer (2.5, 2.5, 12.5)\n");
        return 3;
    }

    std::printf("IPOPT: OK (matches analytic optimum within %.0e)\n", tol);
    return 0;
}
