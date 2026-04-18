#pragma once

#include <cstdint>

namespace app {

// Called once in setup() - configures serial transport and
// attempts first connection to the micro-ROS agent.
// Returns true if agent was found.
bool initMicroRosTransport();

// Called in commandRxTask loop - spins executor for up to
// timeout_ms, handles reconnection automatically.
void microrosSpinSome(unsigned long timeout_ms);

// Called in telemetryTask - publishes odom, IMU, limit switch.
void publishTelemetry();

// Returns true when micro-ROS agent is connected.
bool microrosConnected();

void commandRxTask(void*);

}  // namespace app
