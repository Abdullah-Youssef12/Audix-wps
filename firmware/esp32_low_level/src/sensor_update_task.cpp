#include "odometry.hpp"
#include "shared_state.hpp"

namespace app {

void runSensorUpdateTask(void*) {
    // TODO: Sample encoders, refresh IMU state, read digital sensors, and
    // update the raw odometry estimate used for Pi-side EKF fusion.
}

}  // namespace app
