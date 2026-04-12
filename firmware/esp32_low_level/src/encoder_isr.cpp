#include "encoder_isr.hpp"

namespace app {

// TODO: Increment or sample encoder edge state only. Keep ISR work minimal.
void onFrontLeftEncoderEdge() {}
void onFrontRightEncoderEdge() {}
void onBackLeftEncoderEdge() {}
void onBackRightEncoderEdge() {}

}  // namespace app
