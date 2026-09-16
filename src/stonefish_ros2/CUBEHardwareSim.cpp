#include "stonefish_ros2/CUBEHardwareSim.h"

#include <arpa/inet.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace stonefish_ros2
{

namespace
{
constexpr int64_t kNsPerSec = 1000000000LL;

inline int64_t toNanos(const builtin_interfaces::msg::Time & t)
{
  return static_cast<int64_t>(t.sec) * kNsPerSec + static_cast<int64_t>(t.nanosec);
}
}  // namespace

// ============================================================================
// Construction / destruction
// ============================================================================

CubeHwSim::CubeHwSim(const rclcpp::NodeOptions & options)
: Node("cube_hw_sim", options)
{
  // ── Declare & read parameters ─────────────────────────────────────────── //
  sitl_port_         = declare_parameter<int>        ("sitl_port",         9002);
  sitl_host_         = declare_parameter<std::string>("sitl_host",         "");

  pwm_deadband_      = declare_parameter<int>        ("pwm_deadband",      10);
  velocity_deadband_ = declare_parameter<double>     ("velocity_deadband", 0.05);

  imu_topic_  = declare_parameter<std::string>("imu_topic",  "imu");
  gps_topic_  = declare_parameter<std::string>("gps_topic",  "gps");
  odom_topic_ = declare_parameter<std::string>("odom_topic", "odometry");

  imu_frame_   = declare_parameter<std::string>("imu_frame",      "FLU");
  odom_frame_  = declare_parameter<std::string>("odometry_frame", "FLU");
  world_frame_ = declare_parameter<std::string>("world_frame",    "NED");

  sim_namespace_ = declare_parameter<std::string>("sim_namespace", "");
  lock_step_     = declare_parameter<bool>       ("lock_step",     true);

  exchange_rate_hz_ = declare_parameter<double>("loop_rate", 400.0);
  if (exchange_rate_hz_ <= 0.0)
    throw std::invalid_argument("[cube_hw_sim] loop_rate must be > 0");
  advance_seconds_ = 1.0 / exchange_rate_hz_;
  advance_ns_      = std::llround(advance_seconds_ * 1e9);

  // The granted advance must land exactly on a physics step boundary, otherwise
  // the simulator overshoots the target and the accounting re-anchors every
  // cycle (harmless but it silently changes the effective exchange rate).
  const double sim_step_hz = declare_parameter<double>("sim_step_hz", 0.0);
  if (sim_step_hz > 0.0) {
    const double steps = sim_step_hz / exchange_rate_hz_;
    if (std::abs(steps - std::round(steps)) > 1e-6) {
      RCLCPP_WARN(get_logger(),
        "[cube_hw_sim] loop_rate %.3f Hz is not an integer divisor of "
        "sim_step_hz %.3f Hz (%.4f steps per exchange). The grant will not "
        "align with the physics step.", exchange_rate_hz_, sim_step_hz, steps);
    } else {
      RCLCPP_INFO(get_logger(), "[cube_hw_sim] %.0f physics step(s) per exchange",
                  std::round(steps));
    }
  }

  const std::string clock_topic = declare_parameter<std::string>("clock_topic", "/clock");
  const bool clock_reliable     = declare_parameter<bool>("clock_reliable", true);
  const int  clock_depth        = declare_parameter<int> ("clock_depth",    50);
  clock_stall_ms_               = declare_parameter<int> ("clock_stall_ms", 1000);
  recv_timeout_ms_              = declare_parameter<int> ("recv_timeout_ms", 20);
  recv_retries_                 = declare_parameter<int> ("recv_retries",    3);
  stats_period_s_               = declare_parameter<double>("stats_period_s", 5.0);

  // Validate at startup so misconfiguration is caught early
  const std::set<std::string> valid_imu      = {"FLU", "FRD"};
  const std::set<std::string> valid_odometry = {"FLU", "FRD"};
  const std::set<std::string> valid_world    = {"ENU", "NWU", "NED"};

  if (!valid_imu.count(imu_frame_))
    throw std::invalid_argument("[cube_hw_sim] imu_frame must be FLU or FRD, got: " + imu_frame_);
  if (!valid_odometry.count(odom_frame_))
    throw std::invalid_argument("[cube_hw_sim] odometry_frame must be FLU or FRD, got: " + odom_frame_);
  if (!valid_world.count(world_frame_))
    throw std::invalid_argument("[cube_hw_sim] world_frame must be ENU, NWU or NED, got: " + world_frame_);

  debug_ = declare_parameter<bool>("debug", false);

  RCLCPP_INFO(get_logger(),
    "[cube_hw_sim] imu_frame=%s world_frame=%s | %.1f exchanges/sim-s (%.3f ms grant)",
    imu_frame_.c_str(), world_frame_.c_str(), exchange_rate_hz_, advance_seconds_ * 1e3);

  configureChannels();

  // ── Subscribers ───────────────────────────────────────────────────────── //
  // Depth 1 on the sensors: we always want the freshest sample, never a queue.
  sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
    imu_topic_, rclcpp::QoS(1),
    [this](sensor_msgs::msg::Imu::SharedPtr msg) { cbImu(msg); });

  sub_gps_ = create_subscription<sensor_msgs::msg::NavSatFix>(
    gps_topic_, rclcpp::QoS(1),
    [this](sensor_msgs::msg::NavSatFix::SharedPtr msg) { cbGps(msg); });

  sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::QoS(1),
    [this](nav_msgs::msg::Odometry::SharedPtr msg) { cbOdom(msg); });

  if (lock_step_) {
    // The simulator stops publishing /clock while it is parked at the gate, so
    // a dropped clock message is a permanent stall rather than a hiccup.
    // RELIABLE by default; must match whatever the simulator publishes with.
    rclcpp::QoS clock_qos(clock_depth);
    if (clock_reliable) clock_qos.reliable();
    else                clock_qos.best_effort();

    sub_clock_ = create_subscription<rosgraph_msgs::msg::Clock>(
      clock_topic, clock_qos,
      [this](rosgraph_msgs::msg::Clock::SharedPtr msg) { cbClock(msg); });

    signin_client_  = create_client<stonefish_ros2::srv::AuthorityNodeSignIn>(
                        makeServiceName("authority_node_signin"));
    signout_client_ = create_client<stonefish_ros2::srv::AuthorityNodeSignOut>(
                        makeServiceName("authority_node_signout"));
    vote_pub_       = create_publisher<stonefish_ros2::msg::AuthorityNodeVote>(
                        makeServiceName("authority_node_vote"), rclcpp::QoS(1));
    step_pub_       = create_publisher<stonefish_ros2::msg::AuthorityNodeStep>(
                        makeServiceName("authority_node_step"),
                        rclcpp::QoS(10).reliable());

    // Deferred sign-in: kick it off after spin starts so the executor can
    // process the async callback. One-shot WALL timer (must not depend on the
    // sim clock, which is not running yet).
    signin_timer_ = create_wall_timer(
      std::chrono::milliseconds(200),
      [this]() {
        signin_timer_->cancel();
        signIn();
      });

    rclcpp::on_shutdown([this]() {
      if (signed_in_) signOutSync();
    });

    RCLCPP_INFO(get_logger(),
      "[cube_hw_sim] Lock-step enabled, driven by '%s' (%s, depth %d). Authority namespace '%s'",
      clock_topic.c_str(), clock_reliable ? "reliable" : "best_effort", clock_depth,
      sim_namespace_.empty() ? "<node-local>" : sim_namespace_.c_str());
  } else {
    RCLCPP_INFO(get_logger(),
      "[cube_hw_sim] Lock-step DISABLED: free-running at %.1f Hz wall clock. "
      "Consider sending no_time_sync=true so ArduPilot defaults AHRS_EKF_TYPE to 10.",
      exchange_rate_hz_);
  }

  // ── UDP socket ────────────────────────────────────────────────────────── //
  sock_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock_fd_ < 0)
    throw std::runtime_error("[cube_hw_sim] Failed to create UDP socket");

  // Non-blocking; the exchange thread uses poll() to wait, so it blocks
  // deliberately and with a bounded timeout rather than spinning.
  const int flags = fcntl(sock_fd_, F_GETFL, 0);
  fcntl(sock_fd_, F_SETFL, flags | O_NONBLOCK);

  sockaddr_in bind_addr{};
  bind_addr.sin_family      = AF_INET;
  bind_addr.sin_port        = htons(static_cast<uint16_t>(sitl_port_));
  bind_addr.sin_addr.s_addr =
    sitl_host_.empty() ? INADDR_ANY : inet_addr(sitl_host_.c_str());

  if (bind(sock_fd_, reinterpret_cast<sockaddr *>(&bind_addr), sizeof(bind_addr)) < 0)
    throw std::runtime_error("[cube_hw_sim] Failed to bind UDP socket on port "
                             + std::to_string(sitl_port_));

  RCLCPP_INFO(get_logger(), "[cube_hw_sim] UDP socket bound on port %d", sitl_port_);

  // ── Exchange thread ───────────────────────────────────────────────────── //
  last_stats_      = std::chrono::steady_clock::now();
  exchange_thread_ = std::thread([this]() { exchangeLoop(); });
}

CubeHwSim::~CubeHwSim()
{
  stop_ = true;
  gate_cv_.notify_all();
  if (exchange_thread_.joinable())
    exchange_thread_.join();
  if (sock_fd_ >= 0)
    close(sock_fd_);
}

// ============================================================================
// Subscription callbacks — executor thread
// ============================================================================

void CubeHwSim::cbImu(sensor_msgs::msg::Imu::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(sensor_mtx_);
  imu_ = std::move(msg);
}

void CubeHwSim::cbGps(sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(sensor_mtx_);
  gps_ = std::move(msg);
}

void CubeHwSim::cbOdom(nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(sensor_mtx_);
  odom_ = std::move(msg);
}

/**
 * The only trigger in the system. Records the clock and wakes the exchange
 * thread only when the granted advance has actually been consumed; every other
 * tick returns immediately without touching the condition variable.
 */
void CubeHwSim::cbClock(const rosgraph_msgs::msg::Clock::SharedPtr msg)
{
  const int64_t sim_ns = toNanos(msg->clock);

  {
    std::lock_guard<std::mutex> lk(gate_mtx_);
    latest_sim_ns_ = sim_ns;
    clock_seen_    = true;
    if (sim_ns < grant_target_ns_)
      return;                       // grant not yet consumed: nothing to do
  }
  gate_cv_.notify_one();
}

// ============================================================================
// Exchange thread
// ============================================================================

void CubeHwSim::exchangeLoop()
{
  // ── Bootstrap ───────────────────────────────────────────────────────────
  // ArduPilot sends a servo packet before it waits for any state, and resends
  // roughly every second while it hears nothing, so we can always learn its
  // address by listening. We do not block the simulator on that: the priming
  // grant goes out as soon as we are signed in and the clock is alive, which
  // is what gets the first sensor messages produced.
  if (lock_step_) {
    while (!stop_ && !signed_in_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    std::unique_lock<std::mutex> lk(gate_mtx_);
    gate_cv_.wait_for(lk, std::chrono::seconds(5), [this] { return stop_ || clock_seen_; });
    if (stop_) return;
    if (!clock_seen_) {
      RCLCPP_WARN(get_logger(),
        "[cube_hw_sim] No clock received yet; priming the grant anyway. If the "
        "simulator never steps, check the /clock QoS match and the authority sign-in.");
    }
    const int64_t sim_now = latest_sim_ns_;
    grant_target_ns_ = sim_now + advance_ns_;
    lk.unlock();  
    for (int i = 0; i < credit_windows_; ++i) grant_sim(grant_target_ns_);

    grant_sim(advance_ns_);
    RCLCPP_INFO(get_logger(), "[cube_hw_sim] Priming grant issued, entering lock-step");
  }

  // ── Steady state ────────────────────────────────────────────────────────
  auto next_free_run = std::chrono::steady_clock::now();

  while (!stop_) {
  int64_t sim_at_send = 0;

  if (lock_step_) {
    if (!waitForBudget(sim_at_send)) break;

    // Top up credit BEFORE exchanging, so the simulator never reaches zero
    // and never parks. Accumulate — re-anchoring leaks windows if we fall behind.
    {
      std::lock_guard<std::mutex> lk(gate_mtx_);
      grant_target_ns_ += advance_ns_;

      const int64_t behind = latest_sim_ns_ - grant_target_ns_;
      if (behind > static_cast<int64_t>(credit_windows_) * advance_ns_) {
        ++credit_runaways_;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "[cube_hw_sim] Credit runaway: %.1f windows behind the sim; not gating.",
          static_cast<double>(behind) / advance_ns_);
      }
    }
    grant_sim(advance_ns_);

  } else {
    next_free_run += std::chrono::nanoseconds(advance_ns_);
    std::this_thread::sleep_until(next_free_run);
    if (stop_) break;
    std::lock_guard<std::mutex> lk(gate_mtx_);
    sim_at_send = latest_sim_ns_;
  }

  doExchange(sim_at_send);
  reportStats(sim_at_send);
}

  RCLCPP_INFO(get_logger(), "[cube_hw_sim] Exchange thread exiting after %lu exchanges",
              static_cast<unsigned long>(exchanges_));
}

bool CubeHwSim::waitForBudget(int64_t & sim_at_send)
{
  std::unique_lock<std::mutex> lk(gate_mtx_);

  const auto pred = [this] { return stop_ || latest_sim_ns_ >= grant_target_ns_; };

  while (!pred()) {
    if (clock_stall_ms_ > 0) {
      if (!gate_cv_.wait_for(lk, std::chrono::milliseconds(clock_stall_ms_), pred)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "[cube_hw_sim] No qualifying clock tick for %d ms (sim=%.6f target=%.6f). "
          "Simulator parked, clock QoS mismatch, or the authority watchdog dropped us.",
          clock_stall_ms_, latest_sim_ns_ * 1e-9, grant_target_ns_ * 1e-9);
      }
    } else {
      gate_cv_.wait(lk, pred);
    }
  }

  if (stop_) return false;
  sim_at_send = latest_sim_ns_;
  return true;
}

bool CubeHwSim::doExchange(int64_t sim_at_send)
{
  // ── Snapshot ────────────────────────────────────────────────────────────
  const StateSnapshot snap = snapshotState();
  if (!snap.valid) {
    ++skipped_no_state_;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "[cube_hw_sim] Waiting for %s and %s; granting time without exchanging",
      imu_topic_.c_str(), odom_topic_.c_str());
    return false;
  }

  // Sensor content should carry the same stamp as the clock that triggered us.
  // A persistent lag means the simulator publishes its sensors after the
  // authority gate, so the state we ship is older than the timestamp we put
  // on it — a fixed phase error straight into ArduPilot's rate loop.
  const int64_t imu_lag = sim_at_send - toNanos(snap.imu.header.stamp);
  worst_imu_lag_ns_ = std::max(worst_imu_lag_ns_, imu_lag);

  if (!sitl_addr_known_) {
    // Listen briefly so ArduPilot's unsolicited servo packet can teach us where
    // it lives. It resends about once a second until it hears from us.
    SitlPacket boot{};
    if (recvPacket(boot, recv_timeout_ms_)) {
      RCLCPP_INFO(get_logger(), "[cube_hw_sim] SITL discovered at %s:%d (frame_rate %u Hz)",
                  inet_ntoa(sitl_addr_.sin_addr), ntohs(sitl_addr_.sin_port), boot.frame_rate_hz);
      ap_frame_rate_hz_ = boot.frame_rate_hz;
      last_frame_count_ = boot.frame_count;
      have_frame_count_ = true;
      publishActuators(boot.pwm);
      ++exchanges_;
      return true;
    }
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "[cube_hw_sim] No SITL peer yet on port %d", sitl_port_);
    return false;
  }

  // ── Build ───────────────────────────────────────────────────────────────
  const auto payload = buildSitlJson(snap.imu, snap.odom, sim_at_send);
  if (!payload) return false;
  const std::string frame = "\n" + *payload + "\n";

  // Anything already queued predates this exchange and is stale by definition.
  // Reading the oldest datagram instead of the newest is how a single extra
  // ArduPilot frame turns into a permanent one-frame offset.
  const int stale = drainSocket();
  if (stale > 0) {
    drained_packets_ += static_cast<uint64_t>(stale);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "[cube_hw_sim] Discarded %d stale packet(s) before sending", stale);
  }

  // ── Send / receive ──────────────────────────────────────────────────────
  SitlPacket pkt{};
  bool got = false;
  const auto t_send = std::chrono::steady_clock::now();

  for (int attempt = 0; attempt <= recv_retries_ && !stop_; ++attempt) {
    if (attempt > 0) {
      ++retransmits_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "[cube_hw_sim] No PWM reply within %d ms, retransmitting (attempt %d/%d)",
        recv_timeout_ms_, attempt, recv_retries_);
    }

    // Retransmit the *identical* frame. ArduPilot computes deltat from the
    // timestamp; a duplicate yields deltat == 0, which it ignores for time
    // advance, so a retry cannot corrupt its clock.
    const ssize_t sent = sendto(sock_fd_, frame.c_str(), frame.size(), 0,
                                reinterpret_cast<const sockaddr *>(&sitl_addr_),
                                sizeof(sitl_addr_));
    if (sent != static_cast<ssize_t>(frame.size())) {
      RCLCPP_WARN(get_logger(), "[cube_hw_sim] sendto: %s", strerror(errno));
      continue;
    }

    if (recvPacket(pkt, recv_timeout_ms_)) { got = true; break; }
    ++timeouts_;
  }

  if (!got) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
      "[cube_hw_sim] Exchange failed after %d attempts; skipping actuator update",
      recv_retries_ + 1);
    return false;
  }

  const int64_t rtt = std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::steady_clock::now() - t_send).count();
  worst_rtt_ns_ = std::max(worst_rtt_ns_, rtt);

  checkFrameContinuity(pkt);
  ap_frame_rate_hz_ = pkt.frame_rate_hz;

  // Only now, with a real packet in hand, do the setpoints go out.
  publishActuators(pkt.pwm);
  ++exchanges_;
  return true;
}

void CubeHwSim::releaseBudget(int64_t sim_at_send)
{
  // Order matters. The target must be visible to cbClock() before the grant is
  // published, otherwise the simulator can step, publish /clock and have that
  // tick compared against the stale target — the trigger is consumed against
  // the old value, the thread never wakes, and the sim parks forever.
  {
    std::lock_guard<std::mutex> lk(gate_mtx_);
    grant_target_ns_ = sim_at_send + advance_ns_;
  }
  grant_sim(advance_ns_);
}

CubeHwSim::StateSnapshot CubeHwSim::snapshotState() const
{
  StateSnapshot s;
  std::lock_guard<std::mutex> lk(sensor_mtx_);
  if (!imu_ || !odom_) return s;
  s.imu   = *imu_;
  s.odom  = *odom_;
  s.valid = true;
  return s;
}

int CubeHwSim::drainSocket()
{
  int dropped = 0;
  SitlPacket scratch{};
  sockaddr_in sender{};
  socklen_t sender_len = sizeof(sender);

  while (recvfrom(sock_fd_, &scratch, sizeof(scratch), 0,
                  reinterpret_cast<sockaddr *>(&sender), &sender_len) > 0) {
    ++dropped;
    sender_len = sizeof(sender);
    if (dropped > 64) break;   // pathological; stop rather than spin
  }
  return dropped;
}

bool CubeHwSim::recvPacket(SitlPacket & pkt, int timeout_ms)
{
  const auto deadline = std::chrono::steady_clock::now()
                      + std::chrono::milliseconds(timeout_ms);

  while (!stop_) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) return false;

    const auto remain_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                             deadline - now).count();

    struct pollfd pfd{sock_fd_, POLLIN, 0};
    const int pr = ::poll(&pfd, 1, static_cast<int>(std::max<int64_t>(remain_ms, 1)));
    if (pr < 0) {
      if (errno == EINTR) continue;
      RCLCPP_WARN(get_logger(), "[cube_hw_sim] poll: %s", strerror(errno));
      return false;
    }
    if (pr == 0) return false;   // timed out

    sockaddr_in sender{};
    socklen_t sender_len = sizeof(sender);
    const ssize_t n = recvfrom(sock_fd_, &pkt, sizeof(pkt), 0,
                               reinterpret_cast<sockaddr *>(&sender), &sender_len);
    if (n < 0) {
      if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) continue;
      RCLCPP_WARN(get_logger(), "[cube_hw_sim] recvfrom: %s", strerror(errno));
      return false;
    }

    if (static_cast<size_t>(n) != sizeof(SitlPacket) || pkt.magic != SITL_MAGIC) {
      ++bad_packets_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "[cube_hw_sim] Bad packet: size=%zd (want %zu) magic=0x%04X",
        n, sizeof(SitlPacket), pkt.magic);
      continue;
    }

    sitl_addr_       = sender;
    sitl_addr_known_ = true;
    return true;
  }
  return false;
}

void CubeHwSim::checkFrameContinuity(const SitlPacket & pkt)
{
  if (have_frame_count_) {
    const uint32_t delta = pkt.frame_count - last_frame_count_;
    if (delta != 1u) {
      ++frame_gaps_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "[cube_hw_sim] ArduPilot frame_count jumped by %u (expected 1). It ran "
        "%u frame(s) we did not drive: the 1:1 lock-step invariant is broken.",
        delta, delta > 1u ? delta - 1u : 0u);
    }
  }
  last_frame_count_ = pkt.frame_count;
  have_frame_count_ = true;
}

void CubeHwSim::reportStats(int64_t sim_at_send)
{
  if (stats_period_s_ <= 0.0) return;

  const auto now = std::chrono::steady_clock::now();
  const double dt = std::chrono::duration<double>(now - last_stats_).count();
  if (dt < stats_period_s_) return;

  static int64_t last_sim_ns = 0;
  const double sim_elapsed = (sim_at_send - last_sim_ns) * 1e-9;
  last_sim_ns = sim_at_send;
  last_stats_ = now;

  RCLCPP_INFO(get_logger(),
    "[cube_hw_sim] RTF %.3f | exchanges %lu | ap_rate %u Hz | gaps %lu | "
    "retx %lu | timeouts %lu | bad %lu | stale %lu | no-state %lu | "
    "max rtt %.0f us | max imu lag %.0f us",
    sim_elapsed / dt,
    static_cast<unsigned long>(exchanges_),
    ap_frame_rate_hz_,
    static_cast<unsigned long>(frame_gaps_),
    static_cast<unsigned long>(retransmits_),
    static_cast<unsigned long>(timeouts_),
    static_cast<unsigned long>(bad_packets_),
    static_cast<unsigned long>(drained_packets_),
    static_cast<unsigned long>(skipped_no_state_),
    worst_rtt_ns_ * 1e-3,
    worst_imu_lag_ns_ * 1e-3);

  worst_rtt_ns_     = 0;
  worst_imu_lag_ns_ = 0;
}

// ============================================================================
// Authority
// ============================================================================

void CubeHwSim::grant_sim(int64_t advance_ns)
{
  if (!signed_in_) return;

  stonefish_ros2::msg::AuthorityNodeStep step;
  step.id              = authority_id_;
  step.advance.sec     = static_cast<int32_t>(advance_ns / kNsPerSec);
  step.advance.nanosec = static_cast<uint32_t>(advance_ns % kNsPerSec);
  step_pub_->publish(step);
}

void CubeHwSim::allow_sim(bool permission_to_step)
{
  if (!signed_in_) return;

  stonefish_ros2::msg::AuthorityNodeVote vote;
  vote.id   = authority_id_;
  vote.vote = permission_to_step;
  vote_pub_->publish(vote);
}

void CubeHwSim::signOutSync()
{
  auto req = std::make_shared<stonefish_ros2::srv::AuthorityNodeSignOut::Request>();
  req->id = authority_id_;
  signout_client_->async_send_request(req);

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  RCLCPP_INFO(get_logger(), "[cube_hw_sim] Sign-out request sent for ID %ld", authority_id_);
  signed_in_ = false;
}

// ============================================================================
// State serialisation
// ============================================================================

std::optional<std::string> CubeHwSim::buildSitlJson(
  const sensor_msgs::msg::Imu & imu,
  const nav_msgs::msg::Odometry & odom,
  int64_t sim_ns) const
{
  // ── IMU → FRD ─────────────────────────────────────────────────────────── //
  double ax, ay, az, p, q, r;

  if (imu_frame_ == "FLU") {
    ax =  imu.linear_acceleration.x;
    ay = -imu.linear_acceleration.y;
    az = -imu.linear_acceleration.z;
    p  =  imu.angular_velocity.x;
    q  = -imu.angular_velocity.y;
    r  = -imu.angular_velocity.z;
  } else {
    ax = imu.linear_acceleration.x;
    ay = imu.linear_acceleration.y;
    az = imu.linear_acceleration.z;
    p  = imu.angular_velocity.x;
    q  = imu.angular_velocity.y;
    r  = imu.angular_velocity.z;
  }

  // ── Attitude: quaternion → ZYX Euler ──────────────────────────────────── //
  double roll{}, pitch{}, yaw{};

  const double qw   = odom.pose.pose.orientation.w;
  const double qx_q = odom.pose.pose.orientation.x;
  const double qy_q = odom.pose.pose.orientation.y;
  const double qz_q = odom.pose.pose.orientation.z;

  eulerFromQuaternion(qx_q, qy_q, qz_q, qw, roll, pitch, yaw);

  // ── Position → NED ────────────────────────────────────────────────────── //
  double px, py, pz;
  double qw_ned, qx_ned, qy_ned, qz_ned;
  const auto & pos = odom.pose.pose.position;

  if (world_frame_ == "ENU") {
    px =  pos.y;   // North
    py =  pos.x;   // East
    pz = -pos.z;   // Down

    qw_ned = qw; qx_ned = qx_q; qy_ned = qy_q; qz_ned = qz_q;
  } else if (world_frame_ == "NWU") {
    px =  pos.x;
    py = -pos.y;
    pz = -pos.z;

    qw_ned =  qw; qx_ned =  qx_q; qy_ned = -qy_q; qz_ned = -qz_q;
  } else {
    px = pos.x; py = pos.y; pz = pos.z;
    qw_ned = qw; qx_ned = qx_q; qy_ned = qy_q; qz_ned = qz_q;
  }

  // ── Velocity: body frame → NED ────────────────────────────────────────── //
  const double cy = std::cos(yaw),   sy = std::sin(yaw);
  const double cp = std::cos(pitch), sp = std::sin(pitch);
  const double cr = std::cos(roll),  sr = std::sin(roll);

  const double vxB = odom.twist.twist.linear.x;
  const double vyB = odom.twist.twist.linear.y;
  const double vzB = odom.twist.twist.linear.z;

  const double vxW = (cy*cp)*vxB + (cy*sp*sr - sy*cr)*vyB + (cy*sp*cr + sy*sr)*vzB;
  const double vyW = (sy*cp)*vxB + (sy*sp*sr + cy*cr)*vyB + (sy*sp*cr - cy*sr)*vzB;
  const double vzW = -(sp)  *vxB + (cp*sr)           *vyB + (cp*cr)           *vzB;

  double vn, ve, vd;
  if (world_frame_ == "ENU") {
    vn = deadband( vyW, velocity_deadband_);
    ve = deadband( vxW, velocity_deadband_);
    vd = deadband(-vzW, velocity_deadband_);
  } else if (world_frame_ == "NWU") {
    vn = deadband( vxW, velocity_deadband_);
    ve = deadband(-vyW, velocity_deadband_);
    vd = deadband(-vzW, velocity_deadband_);
  } else {
    vn = deadband(vxW, velocity_deadband_);
    ve = deadband(vyW, velocity_deadband_);
    vd = deadband(vzW, velocity_deadband_);
  }

  // ── Serialise ─────────────────────────────────────────────────────────── //
  // The timestamp is the clock value that triggered this exchange, not a fresh
  // get_clock() read. ArduPilot derives its entire physics rate from the delta
  // between consecutive timestamps, so it has to be the exact gated value.
  const double ts = sim_ns * 1e-9;

  std::ostringstream ss;
  ss << std::fixed;
  ss.precision(9);
  ss << "{"
     << "\"timestamp\":"    << ts << ","
     << "\"imu\":{"
       << "\"gyro\":["       << p  << "," << q  << "," << r  << "],"
       << "\"accel_body\":[" << ax << "," << ay << "," << az << "]"
     << "},"
     << "\"position\":["    << px << "," << py << "," << pz << "],"
     << "\"quaternion\":["  << qw_ned << "," << qx_ned << ","
                            << qy_ned << "," << qz_ned << "],"
     << "\"velocity\":["    << vn << "," << ve << "," << vd << "],"
     << "\"no_time_sync\": " << (lock_step_ ? "false" : "true")
     << "}";
  return ss.str();
}

void CubeHwSim::eulerFromQuaternion(
  double qx, double qy, double qz, double qw,
  double & roll, double & pitch, double & yaw)
{
  const double sinr_cosp = 2.0 * (qw * qx + qy * qz);
  const double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
  roll = std::atan2(sinr_cosp, cosr_cosp);

  const double sinp = 2.0 * (qw * qy - qz * qx);
  pitch = std::asin(std::clamp(sinp, -1.0, 1.0));

  const double siny_cosp = 2.0 * (qw * qz + qx * qy);
  const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
  yaw = std::atan2(siny_cosp, cosy_cosp);
}

// ============================================================================
// UNCHANGED — paste your existing bodies here verbatim.
// These were in the region of the file I could not see; none of them need to
// change for this rewrite. publishActuators() is now called only from the
// exchange thread, and rclcpp publishers are thread-safe, so it is fine as-is.
// ============================================================================

void CubeHwSim::configureChannels()
{
  // List of channel names; each has its own <name>.* sub-parameters.
  const auto names = declare_parameter<std::vector<std::string>>(
      "channels", std::vector<std::string>{});

  if (names.empty()) {
    throw std::invalid_argument("[cube_hw_sim] 'channels' parameter is empty");
  }

  channels_.clear();
  channels_.reserve(names.size());

  for (const auto & n : names) {
    ChannelConfig cfg;
    cfg.name         = n;
    cfg.sitl_index   = declare_parameter<int>        (n + ".sitl_index",   0);
    cfg.topic        = declare_parameter<std::string>(n + ".topic",        "");
    cfg.output_index = declare_parameter<int>        (n + ".output_index", 0);
    cfg.joint        = declare_parameter<std::string>(n + ".joint",        "");
    cfg.pwm_min      = declare_parameter<int>        (n + ".pwm_min",      1000);
    cfg.pwm_center   = declare_parameter<int>        (n + ".pwm_center",   1500);
    cfg.pwm_max      = declare_parameter<int>        (n + ".pwm_max",      2000);
    cfg.type         = declare_parameter<std::string>(n + ".type",         "thruster");
    cfg.multiplier   = declare_parameter<double>     (n + ".multiplier",   1.0);

    if (cfg.type != "thruster" && cfg.type != "position" && cfg.type != "velocity") {
      throw std::invalid_argument(
        "[cube_hw_sim] channel '" + n +
        "': type must be 'thruster', 'position' or 'velocity', got: " + cfg.type);
    }
    if (cfg.sitl_index < 0 || cfg.sitl_index >= static_cast<int>(PWM_CHANNELS)) {
      throw std::invalid_argument(
        "[cube_hw_sim] channel '" + n + "': sitl_index " +
        std::to_string(cfg.sitl_index) + " out of range [0, " +
        std::to_string(PWM_CHANNELS) + ")");
    }
    if (cfg.output_index < 0) {
      throw std::invalid_argument(
        "[cube_hw_sim] channel '" + n + "': output_index must be >= 0");
    }
    if ((cfg.type == "position" || cfg.type == "velocity") && cfg.joint.empty()) {
      throw std::invalid_argument(
        "[cube_hw_sim] channel '" + n + "': type '" + cfg.type +
        "' publishes a JointState and requires a non-empty 'joint' name");
    }
    if (!(cfg.pwm_min <= cfg.pwm_center && cfg.pwm_center <= cfg.pwm_max)) {
      throw std::invalid_argument(
        "[cube_hw_sim] channel '" + n +
        "': require pwm_min <= pwm_center <= pwm_max, got " +
        std::to_string(cfg.pwm_min) + "/" +
        std::to_string(cfg.pwm_center) + "/" +
        std::to_string(cfg.pwm_max));
    }

    channels_.push_back(cfg);
  }

  // ── Group channels by (topic, type) ─────────────────────────────────────
  // Channels sharing the same (topic, type) are aggregated into a single
  // message. type=="thruster" → std_msgs/Float64MultiArray, written at output_index
  // and zero-filled to width. type=="position"/"velocity" → sensor_msgs/JointState,
  // keyed by joint name (Stonefish matches servos by name, not by slot). A channel
  // with an empty topic is disabled: not grouped and nothing is published for it.
  groups_.clear();
  for (size_t i = 0; i < channels_.size(); ++i) {
    const ChannelConfig & cfg = channels_[i];

    if (cfg.topic.empty()) {
      RCLCPP_INFO(get_logger(),
        "[cube_hw_sim]   channel '%s' has empty topic → disabled",
        cfg.name.c_str());
      continue;
    }

    const size_t slot = static_cast<size_t>(cfg.output_index) + 1;

    auto it = std::find_if(groups_.begin(), groups_.end(),
      [&](const OutputGroup & g) { return g.topic == cfg.topic && g.type == cfg.type; });

    if (it == groups_.end()) {
      OutputGroup g;
      g.topic = cfg.topic;
      g.type  = cfg.type;
      g.width = slot;
      g.channel_idx.push_back(i);
      if (cfg.type == "thruster") {
        g.thr_pub = create_publisher<std_msgs::msg::Float64MultiArray>(cfg.topic, rclcpp::QoS(1));
      } else {
        g.srv_pub = create_publisher<sensor_msgs::msg::JointState>(cfg.topic, rclcpp::QoS(1));
      }
      groups_.push_back(std::move(g));
    } else {
      it->channel_idx.push_back(i);
      it->width = std::max(it->width, slot);
    }

    RCLCPP_INFO(get_logger(),
      "[cube_hw_sim]   channel '%s' → SITL ch%d  topic='%s' type='%s' out_idx=%d"
      " joint='%s' min=%d center=%d max=%d mult=%.4f%s",
      cfg.name.c_str(), cfg.sitl_index, cfg.topic.c_str(), cfg.type.c_str(),
      cfg.output_index, cfg.joint.c_str(), cfg.pwm_min, cfg.pwm_center, cfg.pwm_max,
      cfg.multiplier, (cfg.pwm_min == cfg.pwm_center) ? "  (one-way)" : "");
  }

  // Emit each group's channels in ascending output_index order. This makes the
  // JointState name/position arrays deterministic and compact; for thruster groups
  // it is harmless (they are placed by output_index regardless).
  for (auto & g : groups_) {
    std::sort(g.channel_idx.begin(), g.channel_idx.end(),
      [this](size_t a, size_t b) {
        return channels_[a].output_index < channels_[b].output_index;
      });
  }

  RCLCPP_INFO(get_logger(),
    "[cube_hw_sim] %zu channel(s) mapped into %zu output group(s)",
    channels_.size(), groups_.size());
  for (const auto & g : groups_) {
    RCLCPP_INFO(get_logger(),
      "[cube_hw_sim]   group topic='%s' type='%s' msg=%s channels=%zu",
      g.topic.c_str(), g.type.c_str(),
      (g.type == "thruster") ? "Float64MultiArray" : "JointState",
      g.channel_idx.size());
  }
}

void CubeHwSim::publishActuators(const uint16_t * pwm)
{
  for (auto & g : groups_) {
    if (g.type == "thruster") {
      // ── Thruster setpoints → std_msgs/Float64MultiArray ────────────────
      // Slotted by output_index, zero-filled to the group width.
      std_msgs::msg::Float64MultiArray msg;
      msg.data.assign(g.width, 0.0);

      for (size_t idx : g.channel_idx) {
        const ChannelConfig & cfg = channels_[idx];
        // sitl_index is validated in configureChannels() to be within range.
        msg.data[static_cast<size_t>(cfg.output_index)] =
            normalisePwm(pwm[cfg.sitl_index], cfg) * cfg.multiplier;
      }

      g.thr_pub->publish(msg);

      if (debug_) {
        std::string data_str;
        for (double v : msg.data) data_str += std::to_string(v) + " ";
        RCLCPP_INFO(get_logger(), "[cube_hw_sim] %s [thruster]: %s",
          g.topic.c_str(), data_str.c_str());
      }
    } else {
      // ── Servo setpoints → sensor_msgs/JointState ───────────────────────
      // Keyed by joint name. type=="position" fills position[]; type=="velocity"
      // fills velocity[]. Stonefish auto-detects the mode from which vector is
      // populated, so only the relevant one is filled.
      sensor_msgs::msg::JointState msg;
      msg.header.stamp = get_clock()->now();
      msg.name.reserve(g.channel_idx.size());

      std::vector<double> vals;
      vals.reserve(g.channel_idx.size());

      for (size_t idx : g.channel_idx) {   // channel_idx sorted by output_index
        const ChannelConfig & cfg = channels_[idx];
        msg.name.push_back(cfg.joint);
        vals.push_back(normalisePwm(pwm[cfg.sitl_index], cfg) * cfg.multiplier);
      }

      if (g.type == "velocity") {
        msg.velocity = std::move(vals);
      } else {  // "position"
        msg.position = std::move(vals);
      }

      g.srv_pub->publish(msg);

      if (debug_) {
        std::string data_str;
        const auto & out = (g.type == "velocity") ? msg.velocity : msg.position;
        for (size_t k = 0; k < msg.name.size(); ++k) {
          data_str += msg.name[k] + "=" + std::to_string(out[k]) + " ";
        }
        RCLCPP_INFO(get_logger(), "[cube_hw_sim] %s [%s]: %s",
          g.topic.c_str(), g.type.c_str(), data_str.c_str());
      }
    }
  }
}

std::string CubeHwSim::makeServiceName(const std::string & name) const
{
  // Empty namespace → relative name (resolved against this node's namespace).
  // Non-empty → prepend as-is. User passes "/stonefish" for absolute,
  // or "stonefish" for relative.
  if (sim_namespace_.empty()) return name;

  std::string ns = sim_namespace_;
  while (!ns.empty() && ns.back() == '/') ns.pop_back();
  return "/" + ns + "/" + name;
}

void CubeHwSim::signIn()
{
  while (rclcpp::ok() && !signin_client_->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_WARN(get_logger(),
      "[cube_hw_sim] Waiting for sign-in service '%s' to become available...",
      signin_client_->get_service_name());
  }

   auto req = std::make_shared<stonefish_ros2::srv::AuthorityNodeSignIn::Request>();
  req->node_name  = get_name();
  req->timeout_ms = 1000;   // watchdog: manager frees the sim if we go silent for 1 s
  req->mode       = 1;      // STEPPER

  signin_client_->async_send_request(
    req,
    [this](rclcpp::Client<stonefish_ros2::srv::AuthorityNodeSignIn>::SharedFuture fut) {
      auto res = fut.get();
      if (res->success) {
        authority_id_ = res->id;
        signed_in_    = true;
        RCLCPP_INFO(get_logger(),
          "[cube_hw_sim] Signed in as STEPPER (ID %ld), granting %.6f s per exchange",
          authority_id_, advance_seconds_);
      } else {
        RCLCPP_ERROR(get_logger(), "[cube_hw_sim] Sign-in failed; lock-step disabled");
        lock_step_ = false;
      }
    });
}

double CubeHwSim::normalisePwm(uint16_t raw, const ChannelConfig & cfg) const
{
  const int delta = static_cast<int>(raw) - cfg.pwm_center;

  if (std::abs(delta) <= pwm_deadband_)
    return 0.0;

  if (delta > 0) {
    const int upper = cfg.pwm_max - cfg.pwm_center;
    if (upper <= 0) return 0.0;          // one-way reverse channel, ignore +
    return std::clamp(static_cast<double>(delta) / upper, 0.0, 1.0);
  } else {
    const int lower = cfg.pwm_center - cfg.pwm_min;
    if (lower <= 0) return 0.0;          // one-way forward channel, ignore −
    return std::clamp(static_cast<double>(delta) / lower, -1.0, 0.0);
  }
}

}  // namespace stonefish_ros2

// ============================================================================
// Standalone entry point
// ============================================================================

#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<stonefish_ros2::CubeHwSim>();

  // The exchange runs on its own thread, so the executor only has to service
  // /clock plus three sensor topics. A multi-threaded executor keeps a slow
  // sensor callback from delaying the clock tick that gates the whole sim.
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}