#include "microros_transport.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <Arduino.h>
#include <builtin_interfaces/msg/time.h>
#include <micro_ros_platformio.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <rosidl_runtime_c/string_functions.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/bool.h>

#include "config.hpp"
#include "shared_state.hpp"

namespace app {

namespace {

constexpr std::uint8_t kExecutorHandles = 2U;
constexpr unsigned long kReconnectIntervalMs = 1000UL;
constexpr unsigned long kPingIntervalMs = 1000UL;

struct MicroRosContext {
    bool transport_configured = false;
    bool entities_created = false;
    bool messages_initialized = false;
    unsigned long last_connect_attempt_ms = 0UL;
    unsigned long last_ping_ms = 0UL;

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support = {};
    rcl_node_t node = rcl_get_zero_initialized_node();
    rcl_publisher_t odom_pub = rcl_get_zero_initialized_publisher();
    rcl_publisher_t imu_pub = rcl_get_zero_initialized_publisher();
    rcl_publisher_t limit_switch_pub = rcl_get_zero_initialized_publisher();
    rcl_subscription_t cmd_vel_sub = rcl_get_zero_initialized_subscription();
    rcl_subscription_t robot_enable_sub = rcl_get_zero_initialized_subscription();
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();

    geometry_msgs__msg__Twist cmd_vel_msg;
    std_msgs__msg__Bool robot_enable_msg;
    nav_msgs__msg__Odometry odom_msg;
    sensor_msgs__msg__Imu imu_msg;
    std_msgs__msg__Bool limit_switch_msg;
};

MicroRosContext g_microros;

void resetMessageStorage() {
    g_microros.cmd_vel_msg = {};
    g_microros.robot_enable_msg = {};
    g_microros.odom_msg = {};
    g_microros.imu_msg = {};
    g_microros.limit_switch_msg = {};
    g_microros.messages_initialized = false;
}

void resetZeroInitializedHandles() {
    g_microros.support = {};
    g_microros.node = rcl_get_zero_initialized_node();
    g_microros.odom_pub = rcl_get_zero_initialized_publisher();
    g_microros.imu_pub = rcl_get_zero_initialized_publisher();
    g_microros.limit_switch_pub = rcl_get_zero_initialized_publisher();
    g_microros.cmd_vel_sub = rcl_get_zero_initialized_subscription();
    g_microros.robot_enable_sub = rcl_get_zero_initialized_subscription();
    g_microros.executor = rclc_executor_get_zero_initialized_executor();
    resetMessageStorage();
}

void setCovarianceDiagonal(double* covariance, std::size_t element_count, const double* diagonal, std::size_t diagonal_count) {
    std::fill(covariance, covariance + element_count, 0.0);
    for (std::size_t index = 0; index < diagonal_count; ++index) {
        covariance[index * (diagonal_count + 1U)] = diagonal[index];
    }
}

void fillQuaternionFromYaw(float yaw, geometry_msgs__msg__Quaternion& quaternion) {
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = std::sin(yaw * 0.5f);
    quaternion.w = std::cos(yaw * 0.5f);
}

builtin_interfaces__msg__Time currentRosTime() {
    builtin_interfaces__msg__Time stamp;
    stamp.sec = 0;
    stamp.nanosec = 0U;

    const int64_t epoch_ms = rmw_uros_epoch_millis();
    if (epoch_ms <= 0) {
        const std::uint32_t ms = millis();
        stamp.sec = static_cast<std::int32_t>(ms / 1000U);
        stamp.nanosec = (ms % 1000U) * 1000000UL;
        return stamp;
    }

    stamp.sec = static_cast<std::int32_t>(epoch_ms / 1000LL);
    stamp.nanosec = static_cast<std::uint32_t>((epoch_ms % 1000LL) * 1000000LL);
    return stamp;
}

void cmdVelCallback(const void* message) {
    const auto* twist = static_cast<const geometry_msgs__msg__Twist*>(message);
    setCommandVelocity(
        static_cast<float>(twist->linear.x),
        static_cast<float>(twist->linear.y),
        static_cast<float>(twist->angular.z),
        millis());
}

void robotEnableCallback(const void* message) {
    const auto* enabled = static_cast<const std_msgs__msg__Bool*>(message);
    setRobotEnabled(enabled->data);
}

void destroyEntities() {
    if (!g_microros.entities_created) {
        resetZeroInitializedHandles();
        return;
    }

    rmw_context_t* rmw_context = rcl_context_get_rmw_context(&g_microros.support.context);
    if (rmw_context != nullptr) {
        (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
    }

    [[maybe_unused]] const rcl_ret_t odom_pub_fini =
        rcl_publisher_fini(&g_microros.odom_pub, &g_microros.node);
    [[maybe_unused]] const rcl_ret_t imu_pub_fini =
        rcl_publisher_fini(&g_microros.imu_pub, &g_microros.node);
    [[maybe_unused]] const rcl_ret_t limit_switch_pub_fini =
        rcl_publisher_fini(&g_microros.limit_switch_pub, &g_microros.node);
    [[maybe_unused]] const rcl_ret_t cmd_vel_sub_fini =
        rcl_subscription_fini(&g_microros.cmd_vel_sub, &g_microros.node);
    [[maybe_unused]] const rcl_ret_t robot_enable_sub_fini =
        rcl_subscription_fini(&g_microros.robot_enable_sub, &g_microros.node);
    (void)rclc_executor_fini(&g_microros.executor);
    [[maybe_unused]] const rcl_ret_t node_fini = rcl_node_fini(&g_microros.node);
    (void)rclc_support_fini(&g_microros.support);

    if (g_microros.messages_initialized) {
        nav_msgs__msg__Odometry__fini(&g_microros.odom_msg);
        sensor_msgs__msg__Imu__fini(&g_microros.imu_msg);
        geometry_msgs__msg__Twist__fini(&g_microros.cmd_vel_msg);
        std_msgs__msg__Bool__fini(&g_microros.robot_enable_msg);
        std_msgs__msg__Bool__fini(&g_microros.limit_switch_msg);
    }

    g_microros.entities_created = false;
    resetZeroInitializedHandles();
}

bool initMessages() {
    resetMessageStorage();

    const double odom_pose_diag[6] = {0.02, 0.02, 1e6, 1e6, 1e6, 0.08};
    const double odom_twist_diag[6] = {0.05, 0.05, 1e6, 1e6, 1e6, 0.10};
    const double imu_orientation_diag[3] = {1e6, 1e6, 0.12};
    const double imu_angular_velocity_diag[3] = {0.05, 0.05, 0.03};
    const double imu_linear_accel_diag[3] = {0.30, 0.30, 0.30};

    bool twist_initialized = false;
    bool robot_enable_initialized = false;
    bool odom_initialized = false;
    bool imu_initialized = false;
    bool limit_switch_initialized = false;

    if (!geometry_msgs__msg__Twist__init(&g_microros.cmd_vel_msg)) {
        return false;
    }
    twist_initialized = true;

    if (!std_msgs__msg__Bool__init(&g_microros.robot_enable_msg)) {
        goto fail;
    }
    robot_enable_initialized = true;

    if (!nav_msgs__msg__Odometry__init(&g_microros.odom_msg)) {
        goto fail;
    }
    odom_initialized = true;

    if (!sensor_msgs__msg__Imu__init(&g_microros.imu_msg)) {
        goto fail;
    }
    imu_initialized = true;

    if (!std_msgs__msg__Bool__init(&g_microros.limit_switch_msg)) {
        goto fail;
    }
    limit_switch_initialized = true;

    if (!rosidl_runtime_c__String__assign(&g_microros.odom_msg.header.frame_id, "odom")) {
        goto fail;
    }
    if (!rosidl_runtime_c__String__assign(&g_microros.odom_msg.child_frame_id, "base_link")) {
        goto fail;
    }
    if (!rosidl_runtime_c__String__assign(&g_microros.imu_msg.header.frame_id, "imu_link")) {
        goto fail;
    }

    setCovarianceDiagonal(g_microros.odom_msg.pose.covariance, 36U, odom_pose_diag, 6U);
    setCovarianceDiagonal(g_microros.odom_msg.twist.covariance, 36U, odom_twist_diag, 6U);
    setCovarianceDiagonal(g_microros.imu_msg.orientation_covariance, 9U, imu_orientation_diag, 3U);
    setCovarianceDiagonal(g_microros.imu_msg.angular_velocity_covariance, 9U, imu_angular_velocity_diag, 3U);
    setCovarianceDiagonal(g_microros.imu_msg.linear_acceleration_covariance, 9U, imu_linear_accel_diag, 3U);

    g_microros.messages_initialized = true;
    return true;

fail:
    if (limit_switch_initialized) {
        std_msgs__msg__Bool__fini(&g_microros.limit_switch_msg);
    }
    if (imu_initialized) {
        sensor_msgs__msg__Imu__fini(&g_microros.imu_msg);
    }
    if (odom_initialized) {
        nav_msgs__msg__Odometry__fini(&g_microros.odom_msg);
    }
    if (robot_enable_initialized) {
        std_msgs__msg__Bool__fini(&g_microros.robot_enable_msg);
    }
    if (twist_initialized) {
        geometry_msgs__msg__Twist__fini(&g_microros.cmd_vel_msg);
    }
    resetMessageStorage();
    return false;
}

bool createEntities() {
    if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
        return false;
    }

    g_microros.entities_created = true;
    g_microros.allocator = rcl_get_default_allocator();
    if (rclc_support_init(&g_microros.support, 0, nullptr, &g_microros.allocator) != RCL_RET_OK) {
        destroyEntities();
        return false;
    }

    if (rclc_node_init_default(&g_microros.node, "audix_esp32_node", "", &g_microros.support) != RCL_RET_OK) {
        destroyEntities();
        return false;
    }

    if (rclc_publisher_init_default(
            &g_microros.odom_pub,
            &g_microros.node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
            "/odom") != RCL_RET_OK) {
        destroyEntities();
        return false;
    }

    if (rclc_publisher_init_default(
            &g_microros.imu_pub,
            &g_microros.node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
            "/imu") != RCL_RET_OK) {
        destroyEntities();
        return false;
    }

    if (rclc_publisher_init_default(
            &g_microros.limit_switch_pub,
            &g_microros.node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
            "/limit_switch") != RCL_RET_OK) {
        destroyEntities();
        return false;
    }

    if (rclc_subscription_init_default(
            &g_microros.cmd_vel_sub,
            &g_microros.node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
            "/cmd_vel") != RCL_RET_OK) {
        destroyEntities();
        return false;
    }

    if (rclc_subscription_init_default(
            &g_microros.robot_enable_sub,
            &g_microros.node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
            "/robot_enable") != RCL_RET_OK) {
        destroyEntities();
        return false;
    }

    if (!initMessages()) {
        destroyEntities();
        return false;
    }

    if (rclc_executor_init(
            &g_microros.executor,
            &g_microros.support.context,
            kExecutorHandles,
            &g_microros.allocator) != RCL_RET_OK) {
        destroyEntities();
        return false;
    }

    if (rclc_executor_add_subscription(
            &g_microros.executor,
            &g_microros.cmd_vel_sub,
            &g_microros.cmd_vel_msg,
            &cmdVelCallback,
            ON_NEW_DATA) != RCL_RET_OK) {
        destroyEntities();
        return false;
    }

    if (rclc_executor_add_subscription(
            &g_microros.executor,
            &g_microros.robot_enable_sub,
            &g_microros.robot_enable_msg,
            &robotEnableCallback,
            ON_NEW_DATA) != RCL_RET_OK) {
        destroyEntities();
        return false;
    }

    rmw_uros_sync_session(1000);
    g_microros.last_ping_ms = millis();
    return true;
}

void populateTelemetryMessages() {
    const builtin_interfaces__msg__Time stamp = currentRosTime();
    const OdometryState odometry_state = getOdometryState();
    const IMUState imu_state = getImuState();
    const SensorState sensor_state = getSensorState();

    g_microros.odom_msg.header.stamp = stamp;
    g_microros.odom_msg.pose.pose.position.x = odometry_state.x;
    g_microros.odom_msg.pose.pose.position.y = odometry_state.y;
    g_microros.odom_msg.pose.pose.position.z = 0.0;
    fillQuaternionFromYaw(odometry_state.theta, g_microros.odom_msg.pose.pose.orientation);
    g_microros.odom_msg.twist.twist.linear.x = odometry_state.vx;
    g_microros.odom_msg.twist.twist.linear.y = odometry_state.vy;
    g_microros.odom_msg.twist.twist.linear.z = 0.0;
    g_microros.odom_msg.twist.twist.angular.x = 0.0;
    g_microros.odom_msg.twist.twist.angular.y = 0.0;
    g_microros.odom_msg.twist.twist.angular.z = odometry_state.wtheta;

    g_microros.imu_msg.header.stamp = stamp;
    fillQuaternionFromYaw(imu_state.orientation_z, g_microros.imu_msg.orientation);
    g_microros.imu_msg.angular_velocity.x = imu_state.gyro_x;
    g_microros.imu_msg.angular_velocity.y = imu_state.gyro_y;
    g_microros.imu_msg.angular_velocity.z = imu_state.gyro_z;
    g_microros.imu_msg.linear_acceleration.x = imu_state.accel_x;
    g_microros.imu_msg.linear_acceleration.y = imu_state.accel_y;
    g_microros.imu_msg.linear_acceleration.z = imu_state.accel_z;

    g_microros.limit_switch_msg.data = sensor_state.limit_switch_pressed;
}

}  // namespace

bool initMicroRosTransport() {
    if (!g_microros.transport_configured) {
        set_microros_serial_transports(Serial);
        g_microros.transport_configured = true;
        resetZeroInitializedHandles();
    }

    return createEntities();
}

void microrosSpinSome(unsigned long timeout_ms) {
    if (!g_microros.transport_configured) {
        initMicroRosTransport();
    }

    const unsigned long now_ms = millis();
    if (!g_microros.entities_created) {
        if ((now_ms - g_microros.last_connect_attempt_ms) >= kReconnectIntervalMs) {
            g_microros.last_connect_attempt_ms = now_ms;
            createEntities();
        }
        return;
    }

    if ((now_ms - g_microros.last_ping_ms) >= kPingIntervalMs) {
        g_microros.last_ping_ms = now_ms;
        if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
            destroyEntities();
            return;
        }
    }

    const rcl_ret_t spin_result = rclc_executor_spin_some(&g_microros.executor, RCL_MS_TO_NS(timeout_ms));
    if ((spin_result != RCL_RET_OK) && (spin_result != RCL_RET_TIMEOUT)) {
        destroyEntities();
    }
}

void publishTelemetry() {
    if (!g_microros.entities_created) {
        return;
    }

    populateTelemetryMessages();

    const bool publish_failed =
        (rcl_publish(&g_microros.odom_pub, &g_microros.odom_msg, nullptr) != RCL_RET_OK) ||
        (rcl_publish(&g_microros.imu_pub, &g_microros.imu_msg, nullptr) != RCL_RET_OK) ||
        (rcl_publish(&g_microros.limit_switch_pub, &g_microros.limit_switch_msg, nullptr) != RCL_RET_OK);

    if (publish_failed) {
        destroyEntities();
    }
}

bool microrosConnected() {
    return g_microros.entities_created;
}

}  // namespace app
