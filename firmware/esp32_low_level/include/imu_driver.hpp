#pragma once

#include <cstddef>
#include <cstdint>

#include "shared_state.hpp"

namespace app {

class ImuDriver {
public:
    bool begin();
    bool read(IMUState& imu_state);
    bool isHealthy() const;

private:
    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegisters(uint8_t start_reg, uint8_t* buffer, std::size_t length);
    void calibrateGyroBias();

    bool initialized_ = false;
    bool healthy_ = false;
    float yaw_rad_ = 0.0f;
    float gyro_bias_x_rad_s_ = 0.0f;
    float gyro_bias_y_rad_s_ = 0.0f;
    float gyro_bias_z_rad_s_ = 0.0f;
    uint32_t last_update_us_ = 0U;
};

ImuDriver& imuDriver();

}  // namespace app
