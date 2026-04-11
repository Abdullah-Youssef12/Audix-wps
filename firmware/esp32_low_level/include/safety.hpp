#pragma once

#include <cstdint>

#include "config.hpp"
#include "shared_state.hpp"

namespace app {

class MotionSafety {
public:
    explicit MotionSafety(SafetyConfig config = kSafetyConfig);
    bool motionAllowed(const CommandState& command, std::uint32_t now_ms) const;

private:
    SafetyConfig config_;
};

}  // namespace app
