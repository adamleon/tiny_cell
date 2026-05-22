#pragma once

// ParseError — the typed exception every io/ parser throws when authored
// input is rejected. Carries the source file path and a human-readable
// detail so the user can locate and fix the bad entry (engineering.md §3,
// "Authored bad data → reject at parse boundary with a diagnostic").
//
// Never used for solver-produced invariant violations — those assert at
// the sync/ boundary because they're bugs, not user errors (CLAUDE.md §1).

#include <filesystem>
#include <stdexcept>
#include <string>
#include <utility>

namespace tinycell::io {

class ParseError : public std::runtime_error {
public:
    ParseError(std::filesystem::path source, std::string detail)
        : std::runtime_error(format_message(source, detail)),
          source_(std::move(source)),
          detail_(std::move(detail)) {}

    const std::filesystem::path& source() const noexcept { return source_; }
    const std::string& detail() const noexcept { return detail_; }

private:
    static std::string format_message(const std::filesystem::path& p, const std::string& d) {
        return p.string() + ": " + d;
    }

    std::filesystem::path source_;
    std::string detail_;
};

} // namespace tinycell::io
