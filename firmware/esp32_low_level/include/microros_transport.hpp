#pragma once

namespace app {

// TODO: Bind these to the actual micro-ROS transport, executor, publishers,
// and subscriptions once the final ESP firmware stack is selected.
void initializeMicroRosTransport();
void spinMicroRosOnce();

}  // namespace app
