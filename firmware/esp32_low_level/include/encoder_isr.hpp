#pragma once

namespace app {

// Minimal ISR entry points. Keep encoder interrupt handlers extremely small and
// defer all heavy work to the sensor update task.
void onFrontLeftEncoderEdge();
void onFrontRightEncoderEdge();
void onBackLeftEncoderEdge();
void onBackRightEncoderEdge();

}  // namespace app
