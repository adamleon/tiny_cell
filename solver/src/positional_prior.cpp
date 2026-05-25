#include <tinycell/solver/positional_prior.hpp>

#include <mp-units/systems/si.h>

namespace tinycell::solver {

NominalLayout positional_prior(std::span<const core::Task> workflow,
                               core::Vec2 input_anchor,
                               core::Vec2 output_anchor) {
    using mp_units::si::metre;
    NominalLayout out;
    if (workflow.empty()) return out;

    const double ix = input_anchor.x.numerical_value_in(metre);
    const double iy = input_anchor.y.numerical_value_in(metre);
    const double ox = output_anchor.x.numerical_value_in(metre);
    const double oy = output_anchor.y.numerical_value_in(metre);
    const double n_plus_one = static_cast<double>(workflow.size() + 1);

    out.entries.reserve(workflow.size());
    for (std::size_t i = 0; i < workflow.size(); ++i) {
        const double t = static_cast<double>(i + 1) / n_plus_one;
        out.entries.push_back(NominalLayout::Entry{
            .task_id = workflow[i].id,
            .nominal = core::Vec2{
                (ix + t * (ox - ix)) * metre,
                (iy + t * (oy - iy)) * metre,
            },
        });
    }
    return out;
}

} // namespace tinycell::solver
