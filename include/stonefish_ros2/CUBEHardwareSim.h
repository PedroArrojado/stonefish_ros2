#ifndef CUBE_HW_SIM_HPP
#define CUBE_HW_SIM_HPP

#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

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
 * ## Execution model (lock-step)
 *
 * There is no ROS timer. The node is driven entirely by the simulation clock
 * and runs the SITL exchange on a dedicated thread:
 *
 *   executor thread          exchange thread
 *   ---------------          ---------------
 *   cbClock(t):              wait until latest_sim_ns_ >= grant_target_ns_
 *     latest_sim_ns_ = t       snapshot IMU/odom
 *     if t >= target:          build JSON, drain socket, sendto
 *       notify                 blocking recv (poll) for the PWM reply
 *     else: return             publishActuators(pwm)   <-- only on a real exchange
 *                              grant_target_ns_ += advance     (BEFORE granting)
 *                              grant_sim(advance_ns_)
 *
 * `grant_target_ns_` is advanced *before* the grant is published, otherwise a
 * `/clock` message can race ahead of the bookkeeping and the trigger is lost
 * (which deadlocks: the sim is parked at the gate and publishes no further
 * clock, so there is no second chance).
 *
 * Because the sim stops producing `/clock` while it is parked, a dropped clock
 * message is a permanent stall. The clock subscription therefore defaults to
 * RELIABLE QoS with real depth, and `clock_stall_ms` acts as a backstop.
 *
 * ## SITL protocol (ArduPilot JSON backend)
 *
 *   SITL -> Node : binary struct  { uint16 magic, uint16 frame_rate,
 *                                   uint32 frame_count, uint16 pwm[16] }
 *
 *   Node -> SITL : newline-delimited JSON
 *                 {
 *                   "timestamp"  : double,
 *                   "imu"        : { "gyro": [p,q,r], "accel_body": [ax,ay,az] },
 *                   "position"   : [x, y, z],
 *                   "quaternion" : [w, x, y, z],
 *                   "velocity"   : [vn, ve, vd]
 *                 }
 *
 *   `frame_count` is monotonic on ArduPilot's side. Any delta other than 1
 *   means ArduPilot ran a frame we did not drive (one of the early-return
 *   paths in recv_fdm, or its ~1 s servo resend) and the 1:1 invariant is
 *   broken even though our own send/recv pairing still looks consistent.
 *
 * ## Frame conventions
 *
 *   Stonefish/ROS 2 : ENU world frame, FLU body frame
 *   ArduPilot SITL  : NED world frame, FRD body frame
 *
 *   FLU -> FRD  :  (ax, ay, az) -> (ax, -ay, -az)  |  (p,q,r) -> (p,-q,-r)
 *   ENU -> NED  :  (vE, vN, vU) -> (vN, vE, -vU)
 *
 * ## Stonefish publishers (per-channel, grouped by (topic, type))
 *
 *   Each output channel maps ONE index of the 16-channel SITL PWM packet
 *   (`sitl_index`) to ONE slot (`output_index`) of an output message on `topic`.
 *   The channel `type` selects the message and how the value is applied:
 *
 *     type = "thruster" -> std_msgs/Float64MultiArray (thrusters). The value is
 *                         written at `output_index`; the array is zero-filled to
 *                         max(output_index)+1. Values in [-1, 1].
 *     type = "position" -> sensor_msgs/JointState (servos), `position[]` filled.
 *     type = "velocity" -> sensor_msgs/JointState (servos), `velocity[]` filled.
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
 *   pwm_deadband       (int,    10)     us deadband around center -> 0.0
 *   velocity_deadband  (double, 0.05)   m/s deadband on NED velocities
 *
 *   loop_rate          (double, 400.0)  exchanges per simulated second; the
 *                                       granted advance is 1/loop_rate and MUST
 *                                       be an exact multiple of the Stonefish
 *                                       physics step
 *   sim_step_hz        (double, 0.0)    if > 0, validated against loop_rate
 *   clock_topic        (string, "/clock")
 *   clock_reliable     (bool,   true)   RELIABLE QoS on /clock; must match the
 *                                       simulator's publisher or nothing binds
 *   clock_depth        (int,    50)
 *   clock_stall_ms     (int,    1000)   warn if no qualifying clock tick lands
 *   recv_timeout_ms    (int,    20)     per-attempt wait for the PWM reply
 *   recv_retries       (int,    3)      retransmits of the identical frame
 *   stats_period_s     (double, 5.0)    0 disables the periodic health line
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

private:
  // ── SITL protocol constants ──────────────────────────────────────────── //
  static constexpr uint16_t SITL_MAGIC   = 18458;
  static constexpr size_t   PWM_CHANNELS = 16;

  // Packed struct matching the ArduPilot SITL binary format (little-endian).
  // Total: 2+2+4+16*2 = 40 bytes. ArduPilot switches to a 32-channel variant
  // when any SERVOn_FUNCTION above 16 is assigned; the first 16 channels have
  // identical layout, so a truncated read stays correct, but the size check
  // below will reject it. Keep servo functions within 1..16.
#pragma pack(push, 1)
  struct SitlPacket
  {
    uint16_t magic;
    uint16_t frame_rate_hz;
    uint32_t frame_count;
    uint16_t pwm[PWM_CHANNELS];
  };
#pragma pack(pop)

  /** Consistent view of the vehicle state taken under sensor_mtx_. */
  struct StateSnapshot
  {
    sensor_msgs::msg::Imu      imu;
    nav_msgs::msg::Odometry    odom;
    bool valid{false};
  };

  // ── Subscription callbacks (executor thread) ─────────────────────────── //
  void cbClock(const rosgraph_msgs::msg::Clock::SharedPtr msg);
  void cbImu(sensor_msgs::msg::Imu::SharedPtr msg);
  void cbGps(sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void cbOdom(nav_msgs::msg::Odometry::SharedPtr msg);

  // ── Exchange thread ──────────────────────────────────────────────────── //

  /** Thread body: bootstrap, then one exchange per consumed grant, forever. */
  void exchangeLoop();

  /**
   * Block until the granted advance has been fully consumed by the simulator.
   * @param[out] sim_at_send  simulation time that triggered this exchange (ns)
   * @return false if the node is shutting down.
   */
  bool waitForBudget(int64_t & sim_at_send);

  /**
   * One complete SITL exchange: snapshot -> JSON -> send -> blocking recv ->
   * publishActuators. Does NOT grant.
   * @return true if a valid PWM packet was received and applied.
   */
  bool doExchange(int64_t sim_at_send);

  /**
   * Advance grant_target_ns_ and publish the grant, in that order.
   * Reversing the order loses trigger events.
   */
  void releaseBudget(int64_t sim_at_send);

  /** Take a consistent copy of the latest IMU + odometry. */
  StateSnapshot snapshotState() const;

  /** Discard any datagrams queued before this exchange. Returns count. */
  int drainSocket();

  /** Wait up to timeout_ms for a valid SITL packet. Learns sitl_addr_. */
  bool recvPacket(SitlPacket & pkt, int timeout_ms);

  /** Verify ArduPilot's frame_count advanced by exactly one. */
  void checkFrameContinuity(const SitlPacket & pkt);

  /** Throttled health line: exchange count, retries, frame gaps, staleness. */
  void reportStats(int64_t sim_at_send);

  // ── Helpers (UNCHANGED from your original implementation) ────────────── //
  void configureChannels();
  void publishActuators(const uint16_t * pwm);
  double normalisePwm(uint16_t raw, const ChannelConfig & cfg) const;
  std::string makeServiceName(const std::string & name) const;
  void signIn();
  void signOutSync();
  void allow_sim(bool permission_to_step);
  void grant_sim(int64_t advance_ns);

  /** Build the SITL JSON from an explicit snapshot and an explicit sim time. */
  std::optional<std::string> buildSitlJson(
    const sensor_msgs::msg::Imu & imu,
    const nav_msgs::msg::Odometry & odom,
    int64_t sim_ns) const;

  static void eulerFromQuaternion(
    double qx, double qy, double qz, double qw,
    double & roll, double & pitch, double & yaw);

  static double deadband(double x, double eps)
  {
    return (std::abs(x) < eps) ? 0.0 : x;
  }

  // ── Lock-step gating state (guarded by gate_mtx_) ────────────────────── //
  mutable std::mutex      gate_mtx_;
  std::condition_variable gate_cv_;
  int64_t latest_sim_ns_   {0};
  int64_t grant_target_ns_ {INT64_MAX};   // INT64_MAX until bootstrap completes
  uint64_t credit_runaways_{0};
  bool    clock_seen_      {false};

  int credit_windows_{3};        // grants kept outstanding; 1 == current behaviour

  std::atomic<bool> stop_{false};
  std::thread       exchange_thread_;

  // ── Authority ────────────────────────────────────────────────────────── //
  std::string sim_namespace_;
  bool        lock_step_{true};
  int64_t     authority_id_{-1};
  std::atomic<bool> signed_in_{false};

  rclcpp::Client<stonefish_ros2::srv::AuthorityNodeSignIn>::SharedPtr  signin_client_;
  rclcpp::Client<stonefish_ros2::srv::AuthorityNodeSignOut>::SharedPtr signout_client_;
  rclcpp::Publisher<stonefish_ros2::msg::AuthorityNodeVote>::SharedPtr vote_pub_;
  rclcpp::Publisher<stonefish_ros2::msg::AuthorityNodeStep>::SharedPtr step_pub_;
  rclcpp::TimerBase::SharedPtr signin_timer_;

  // ── Timing configuration ─────────────────────────────────────────────── //
  double  exchange_rate_hz_{400.0};
  double  advance_seconds_ {0.0025};
  int64_t advance_ns_      {0};
  int     recv_timeout_ms_ {20};
  int     recv_retries_    {3};
  int     clock_stall_ms_  {1000};
  double  stats_period_s_  {5.0};

  // ── Diagnostics ──────────────────────────────────────────────────────── //
  uint64_t exchanges_       {0};
  uint64_t retransmits_     {0};
  uint64_t timeouts_        {0};
  uint64_t bad_packets_     {0};
  uint64_t drained_packets_ {0};
  uint64_t frame_gaps_      {0};
  uint64_t skipped_no_state_{0};
  uint32_t last_frame_count_{0};
  bool     have_frame_count_{false};
  uint16_t ap_frame_rate_hz_{0};
  int64_t  worst_imu_lag_ns_{0};
  int64_t  worst_rtt_ns_    {0};
  std::chrono::steady_clock::time_point last_stats_;

  // ── Parameters ───────────────────────────────────────────────────────── //
  int                        sitl_port_{9002};
  std::string                sitl_host_;
  std::vector<ChannelConfig> channels_;
  int                        pwm_deadband_{10};
  double                     velocity_deadband_{0.05};
  std::string                imu_topic_;
  std::string                gps_topic_;
  std::string                odom_topic_;
  std::string                imu_frame_;
  std::string                odom_frame_;
  std::string                world_frame_;
  bool                       debug_{false};

  // ── ROS 2 interfaces ─────────────────────────────────────────────────── //
  std::vector<OutputGroup> groups_;

  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr    sub_clock_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr        sub_imu_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr  sub_gps_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr      sub_odom_;

  // ── Sensor state (guarded by sensor_mtx_) ────────────────────────────── //
  mutable std::mutex                     sensor_mtx_;
  sensor_msgs::msg::Imu::SharedPtr       imu_;
  sensor_msgs::msg::NavSatFix::SharedPtr gps_;
  nav_msgs::msg::Odometry::SharedPtr     odom_;

  // ── UDP socket (exchange thread only, after construction) ────────────── //
  int                sock_fd_{-1};
  struct sockaddr_in sitl_addr_{};
  bool               sitl_addr_known_{false};
};

}  // namespace stonefish_ros2

#endif  // CUBE_HW_SIM_HPP