#ifndef CUBE_HW_SIM_HPP
#define CUBE_HW_SIM_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>
#include <chrono>

#include "stonefish_ros2/srv/authority_node_sign_in.hpp"
#include "stonefish_ros2/srv/authority_node_sign_out.hpp"
#include "stonefish_ros2/msg/authority_node_vote.hpp"
#include "stonefish_ros2/msg/authority_node_step.hpp"


namespace stonefish_ros2
{

/**
 * @brief Simulated Cube flight-controller node.
 *
 * Bridges ArduPilot SITL (JSON backend, UDP) to/from a Stonefish ROS 2
 * simulation.
 *
 * ## SITL protocol (ArduPilot JSON backend)
 *
 *   SITL → Node : binary struct  { uint16 magic, uint16 frame_rate,
 *                                   uint32 frame_count, uint16 pwm[16] }
 *
 *   Node → SITL : newline-delimited JSON
 *                 {
 *                   "timestamp"  : double,
 *                   "imu"        : { "gyro": [p,q,r], "accel_body": [ax,ay,az] },
 *                   "position"   : [x, y, z],
 *                   "attitude"   : [roll, pitch, yaw],
 *                   "velocity"   : [vn, ve, vd, p, q, r]
 *                 }
 *
 * ## Frame conventions
 *
 *   Stonefish/ROS 2 : ENU world frame, FLU body frame
 *   ArduPilot SITL  : NED world frame, FRD body frame
 *
 *   FLU → FRD  :  (ax, ay, az) → (ax, -ay, -az)  |  (p,q,r) → (p,-q,-r)
 *   ENU → NED  :  (vE, vN, vU) → (vN, vE, -vU)
 *
 * ## Stonefish publishers (per-channel, grouped by (topic, type))
 *
 *   Each output channel maps ONE index of the 16-channel SITL PWM packet
 *   (`sitl_index`) to ONE slot (`output_index`) of an output message on `topic`.
 *   The channel `type` selects the message and how the value is applied:
 *
 *     type = "thruster"    → std_msgs/Float64MultiArray (thrusters). The value is
 *                         written at `output_index`; the array is zero-filled to
 *                         max(output_index)+1. Values ∈ [-1, 1].
 *     type = "position" → sensor_msgs/JointState (servos), `position[]` filled.
 *     type = "velocity" → sensor_msgs/JointState (servos), `velocity[]` filled.
 *
 *   For JointState groups Stonefish matches servos BY JOINT NAME (`joint`), not by
 *   slot, and auto-detects position vs velocity mode from which vector is filled.
 *   Channels sharing the same (topic, type) are aggregated into a SINGLE message.
 *   Values are normalised to [-1, 1] via per-channel min/center/max and then
 *   scaled by `multiplier`. A channel with an empty topic is disabled.
 *
 * ## ROS 2 parameters
 *
 *   sitl_port          (int,    9002)
 *   sitl_host          (string, "")
 *   pwm_deadband       (int,    10)     µs deadband around center → 0.0
 *   velocity_deadband  (double, 0.05)   m/s deadband on NED velocities
 *
 *   channels           (string[])       list of channel names; each name below
 *                                       has its own sub-parameters:
 *     <name>.sitl_index    (int)        source index into the 16-ch SITL packet
 *     <name>.topic         (string)     destination topic ("" = disabled)
 *     <name>.output_index  (int)        slot / ordering in the (topic,type) group
 *     <name>.joint         (string)     Stonefish joint name (position/velocity only)
 *     <name>.pwm_min       (int)        raw PWM mapping to the low end
 *     <name>.pwm_center    (int)        raw PWM mapping to 0.0
 *     <name>.pwm_max       (int)        raw PWM mapping to the high end
 *     <name>.type          (string)     "thruster" | "position" | "velocity"
 *     <name>.multiplier    (double)     scales normalised [-1,1] (e.g. pi)
 */

struct ChannelConfig {
  std::string name;         // channel identifier (debug / traceability)
  int         sitl_index;   // SOURCE: index into the 16-channel SITL PWM packet
  std::string topic;        // DESTINATION topic ("" disables the channel)
  int         output_index; // DESTINATION: slot / ordering in the (topic,type) group
  std::string joint;        // Stonefish joint name (JointState groups); unused for thruster
  int         pwm_min;
  int         pwm_center;
  int         pwm_max;
  std::string type;         // "thruster" | "position" | "velocity" (grouping key)
  double      multiplier;   // scales the normalised [-1,1] value (e.g. M_PI)
};

/**
 * @brief One published message: all channels sharing the same (topic, type).
 *
 * For type=="thruster": thr_pub holds a Float64MultiArray publisher and `width` is
 * the array length (max output_index + 1). For type=="position"/"velocity":
 * srv_pub holds a JointState publisher and channels are emitted compactly in
 * output_index order. `channel_idx` are indices into CubeHwSim::channels_.
 */
struct OutputGroup {
  std::string      topic;
  std::string      type;
  std::size_t      width{0};
  std::vector<std::size_t> channel_idx;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr thr_pub;   // "thruster"
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr     srv_pub;   // "position"/"velocity"
};

class CubeHwSim : public rclcpp::Node
{
public:
  explicit CubeHwSim(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CubeHwSim() override;

  double time_of_last_iteration_sim_time_ = 0.0;
  std::chrono::steady_clock::time_point time_of_last_iteration_wall_time_;

private:
  // ── SITL protocol constants ──────────────────────────────────────────── //
  static constexpr uint16_t SITL_MAGIC     = 18458;
  static constexpr size_t   PWM_CHANNELS   = 16;
  double   LOOP_RATE_HZ   = 400.0;

  std::string sim_namespace_;
  bool lock_step_;
  int64_t authority_id_ = -1;
  std::atomic<bool> signed_in_{false};
  bool was_holding_ = false;

  // Packed struct matching the ArduPilot SITL binary format (little-endian).
  // Total: 2+2+4+16*2 = 40 bytes.
#pragma pack(push, 1)
  struct SitlPacket
  {
    uint16_t magic;
    uint16_t frame_rate_hz;
    uint32_t frame_count;
    uint16_t pwm[PWM_CHANNELS];
  };
#pragma pack(pop)

  // ── Callbacks ────────────────────────────────────────────────────────── //
  void cbImu(sensor_msgs::msg::Imu::SharedPtr msg);
  void cbGps(sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void cbOdom(nav_msgs::msg::Odometry::SharedPtr msg);

  rclcpp::Client<stonefish_ros2::srv::AuthorityNodeSignIn>::SharedPtr  signin_client_;
  rclcpp::Client<stonefish_ros2::srv::AuthorityNodeSignOut>::SharedPtr signout_client_;
 
  rclcpp::Publisher<stonefish_ros2::msg::AuthorityNodeVote>::SharedPtr vote_pub_;
  rclcpp::Publisher<stonefish_ros2::msg::AuthorityNodeStep>::SharedPtr step_pub_;

  rclcpp::TimerBase::SharedPtr signin_timer_;

  // ── Main loop ────────────────────────────────────────────────────────── //
  void loop();

  // ── Helpers ──────────────────────────────────────────────────────────── //

  /**
   * Read per-channel parameters, build channels_, group them by (topic, type)
   * and create one publisher per group (Float64MultiArray for thruster groups,
   * JointState for position/velocity groups). Assumes debug_ is already set.
   */
  void configureChannels();

  /**
   * Fill and publish one message per output group from the raw SITL PWM packet.
   * `pwm` points to the 16-element PWM array of the received packet.
   */
  void publishActuators(const uint16_t * pwm);

  /** Normalise a raw PWM value to [-1, 1] using per-channel min/center/max. */
  double normalisePwm(uint16_t raw, const ChannelConfig & cfg) const;

  /**
   * Build the ArduPilot SITL JSON state string from current sensor data.
   * Returns std::nullopt if required data is not yet available.
   */
  std::optional<std::string> buildSitlJson() const;

  /** ZYX Euler angles from a unit quaternion (no external dependency). */
  static void eulerFromQuaternion(
    double qx, double qy, double qz, double qw,
    double & roll, double & pitch, double & yaw);

  static double deadband(double x, double eps)
  {
    return (std::abs(x) < eps) ? 0.0 : x;
  }

  // Helpers
  std::string makeServiceName(const std::string & name) const;
  void signIn();
  void signOutSync();      // for shutdown
  void allow_sim(bool permission_to_step);
  void grant_sim(int64_t advance_ns);

  
  double  advance_seconds_ = 0.005;   // sim-time granted per exchange; MUST be a multiple of the sim step
  int64_t advance_ns_      = 0;

  bool         grant_pending_ = false;
  rclcpp::Time grant_target_{0, 0, RCL_ROS_TIME};


  // ── Parameters ───────────────────────────────────────────────────────── //
  int                  sitl_port_;
  std::string          sitl_host_;
  std::vector<ChannelConfig> channels_;
  int pwm_deadband_;
  double velocity_deadband_;
  std::string          imu_topic_;
  std::string          gps_topic_;
  std::string          odom_topic_;
  std::string          imu_frame_;
  std::string          odom_frame_;
  std::string          world_frame_;
  bool                 awaiting_response_;
  bool                 debug_;

  // ── ROS 2 interfaces ─────────────────────────────────────────────────── //
  std::vector<OutputGroup> groups_;   // one publisher per (topic, type)

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr        sub_imu_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr  sub_gps_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr      sub_odom_;

  rclcpp::TimerBase::SharedPtr timer_;

  // ── Sensor state ─────────────────────────────────────────────────────── //
  sensor_msgs::msg::Imu::SharedPtr        imu_;
  sensor_msgs::msg::NavSatFix::SharedPtr  gps_;
  nav_msgs::msg::Odometry::SharedPtr      odom_;

  // ── UDP socket ───────────────────────────────────────────────────────── //
  int                sock_fd_  {-1};
  struct sockaddr_in sitl_addr_ {};
  bool               sitl_addr_known_ {false};
};

}  // namespace stonefish_ros2

#endif  // CUBE_HW_SIM_HPP