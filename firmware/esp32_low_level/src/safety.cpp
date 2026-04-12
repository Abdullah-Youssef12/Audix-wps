#include "safety.hpp"

namespace app {

MotionSafety::MotionSafety(SafetyConfig config) : config_(config) {}

bool MotionSafety::motionAllowed(const CommandState& command, std::uint32_t now_ms) const {
    // Safety layer from the low-level flowchart: disable immediately on false
    // enable state or stale command timing.
    if (!command.enabled) {
        return false;
    }
    return (now_ms - command.last_command_ms) <= config_.command_timeout_ms;
}

}  // namespace app
