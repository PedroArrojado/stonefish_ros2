#include "stonefish_ros2/CUBEHardwareSim.h"

#include <arpa/inet.h>
#include <fcntl.h>
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

// ============================================================================
// Construction / destruction
// ============================================================================

CubeHwSim::CubeHwSim(const rclcpp::NodeOptions & options)
: Node("cube_hw_sim", options)
{
  // ── Declare & read parameters ─────────────────────────────────────────── //
  sitl_port_         = declare_parameter<int>        ("sitl_port",                9002);
  sitl_host_         = declare_parameter<std::string>("sitl_host",                "");

  pwm_deadband_      = declare_parameter<int>        ("pwm_deadband",             10);
  velocity_deadband_ = declare_parameter<double>     ("velocity_deadband",        0.05);

  imu_topic_ = declare_parameter<std::string>("imu_topic", "imu");
  gps_topic_ = declare_parameter<std::string>("gps_topic", "gps");
  odom_topic_ = declare_parameter<std::string>("odom_topic", "odometry");

  imu_frame_   = declare_parameter<std::string>("imu_frame",   "FLU");
  odom_frame_   = declare_parameter<std::string>("odometry_frame",   "FLU");
  world_frame_ = declare_parameter<std::string>("world_frame", "NED");

  sim_namespace_ = declare_parameter<std::string>("sim_namespace", "");
  lock_step_     = declare_parameter<bool>("lock_step", true);

  LOOP_RATE_HZ = declare_parameter<double>("loop_rate", LOOP_RATE_HZ);
  advance_seconds_ = 1/LOOP_RATE_HZ;
  if (advance_seconds_ <= 0.0)
    throw std::invalid_argument("[cube_hw_sim] sim_advance_seconds must be > 0");
  advance_ns_ = std::llround(advance_seconds_ * 1e9);

  // Validate at startup so misconfiguration is caught early
  const std::set<std::string> valid_imu = {"FLU", "FRD"};
  const std::set<std::string> valid_odometry = {"FLU", "FRD"};
  const std::set<std::string> valid_world = {"ENU", "NWU", "NED"};

  if (!valid_imu.count(imu_frame_))
      throw std::invalid_argument("[cube_hw_sim] imu_frame must be FLU or FRD, got: " + imu_frame_);
  if (!valid_odometry.count(odom_frame_))
      throw std::invalid_argument("[cube_hw_sim] imu_frame must be FLU or FRD, got: " + odom_frame_);
  if (!valid_world.count(world_frame_))
      throw std::invalid_argument("[cube_hw_sim] world_frame must be ENU, NWU or NED, got: " + world_frame_);

  RCLCPP_INFO(get_logger(), "[cube_hw_sim] imu_frame=%s  world_frame=%s",
      imu_frame_.c_str(), world_frame_.c_str());

  debug_ = declare_parameter<bool>("debug", false);

  // ── Per-channel output configuration + grouped publishers ─────────────── //
  // Reads the per-channel parameters, builds channels_, groups them by
  // (topic, type) and creates one publisher per group. thruster groups publish
  // std_msgs/Float64MultiArray; position/velocity groups publish
  // sensor_msgs/JointState.
  configureChannels();

  // ── Subscribers ───────────────────────────────────────────────────────── //
  sub_imu_  = create_subscription<sensor_msgs::msg::Imu>(
    imu_topic_, rclcpp::QoS(1),
    [this](sensor_msgs::msg::Imu::SharedPtr msg) { cbImu(msg); });

  sub_gps_  = create_subscription<sensor_msgs::msg::NavSatFix>(
    gps_topic_, rclcpp::QoS(1),
    [this](sensor_msgs::msg::NavSatFix::SharedPtr msg) { cbGps(msg); });

  sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::QoS(1),
    [this](nav_msgs::msg::Odometry::SharedPtr msg) { cbOdom(msg); });

  if (lock_step_) {
    signin_client_  = create_client<stonefish_ros2::srv::AuthorityNodeSignIn>(
                        makeServiceName("authority_node_signin"));
    signout_client_ = create_client<stonefish_ros2::srv::AuthorityNodeSignOut>(
                        makeServiceName("authority_node_signout"));
    vote_pub_       = create_publisher<stonefish_ros2::msg::AuthorityNodeVote>(
                        makeServiceName("authority_node_vote"), rclcpp::QoS(1)); //<-- unused
    step_pub_ = create_publisher<stonefish_ros2::msg::AuthorityNodeStep>(
                  makeServiceName("authority_node_step"), rclcpp::QoS(10));

    // Deferred sign-in: kick it off after spin starts so the executor can
    // process the async callback. One-shot timer.
    signin_timer_ = create_wall_timer(
      std::chrono::milliseconds(200),
      [this]() {
        signin_timer_->cancel();
        signIn();
      });

    // Sign out gracefully on rclcpp shutdown
    rclcpp::on_shutdown([this]() {
      if (signed_in_) {
        signOutSync();
      }
    });

    RCLCPP_INFO(get_logger(),
      "[cube_hw_sim] Lock-step enabled. Sim services at namespace '%s'",
      sim_namespace_.empty() ? "<node-local>" : sim_namespace_.c_str());
  }

  // ── UDP socket ────────────────────────────────────────────────────────── //
  sock_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock_fd_ < 0)
    throw std::runtime_error("[cube_hw_sim] Failed to create UDP socket");

  // Non-blocking so the timer callback never stalls the executor
  int flags = fcntl(sock_fd_, F_GETFL, 0);
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

  // ── Timer ─────────────────────────────────────────────────────────────── //
  const auto period = std::chrono::duration<double>(1.0 / LOOP_RATE_HZ);
  timer_ = create_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    [this]() { loop(); });
}

CubeHwSim::~CubeHwSim()
{
  if (sock_fd_ >= 0)
    close(sock_fd_);
}

// ============================================================================
// Callbacks
// ============================================================================

void CubeHwSim::cbImu(sensor_msgs::msg::Imu::SharedPtr msg)  { imu_  = msg; }
void CubeHwSim::cbGps(sensor_msgs::msg::NavSatFix::SharedPtr msg) { gps_  = msg; }
void CubeHwSim::cbOdom(nav_msgs::msg::Odometry::SharedPtr msg) { odom_ = msg; }


// ============================================================================
// Main loop (called at LOOP_RATE_HZ)
// ============================================================================

void CubeHwSim::loop()
{
    if (!imu_ || !odom_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "[cube_hw_sim] Waiting for %s and %s...",
            imu_topic_.c_str(), odom_topic_.c_str());
        return;
    }
    
    const rclcpp::Time sim_now = get_clock()->now();   // sim clock

    // Has the granted advance been fully consumed and the sim re-locked?
    if (grant_pending_ && sim_now.nanoseconds() >= grant_target_.nanoseconds())
        grant_pending_ = false;

    // Phase 1: send fresh state, only once settled and no reply outstanding.
    if (sitl_addr_known_ && !awaiting_response_ && !grant_pending_) {
        auto payload = buildSitlJson();
        if (payload) {
            const std::string frame = "\n" + *payload + "\n";
            sendto(sock_fd_, frame.c_str(), frame.size(), 0,
                reinterpret_cast<const sockaddr *>(&sitl_addr_), sizeof(sitl_addr_));
            awaiting_response_ = true;
            if (debug_) RCLCPP_INFO(get_logger(), "[cube_hw_sim] Sent state JSON to SITL");
        }
    }

    // Phase 2: poll for the PWM reply (non-blocking).
    SitlPacket pkt{};
    sockaddr_in sender{};
    socklen_t sender_len = sizeof(sender);
    const ssize_t n = recvfrom(sock_fd_, &pkt, sizeof(pkt), 0,
        reinterpret_cast<sockaddr *>(&sender), &sender_len);

    if (n < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK)
            RCLCPP_WARN(get_logger(), "[cube_hw_sim] recvfrom error: %s", strerror(errno));
        return;   // no reply yet: sim stays held (no grant issued)
    }
    if (static_cast<size_t>(n) != sizeof(SitlPacket) || pkt.magic != SITL_MAGIC) {
        RCLCPP_WARN(get_logger(),
            "[cube_hw_sim] Bad packet: size=%zd magic=0x%04X", n, pkt.magic);
        return;
    }

    sitl_addr_         = sender;
    sitl_addr_known_   = true;
    awaiting_response_ = false;
    if (debug_)
        RCLCPP_INFO(get_logger(), "[cube_hw_sim] Received packet from %s:%d",
            inet_ntoa(sender.sin_addr), ntohs(sender.sin_port));

    // Phase 3: apply setpoints, then grant exactly one exchange forward.
    publishActuators(pkt.pwm);

    if (lock_step_ && signed_in_) {
        grant_sim(advance_ns_);
        grant_pending_ = true;
        grant_target_  = rclcpp::Time(sim_now.nanoseconds() + advance_ns_, RCL_ROS_TIME);
    }

    if (debug_) {
        const double current_sim_time = sim_now.nanoseconds() * 1e-9;
        const auto current_wall_time  = std::chrono::steady_clock::now();
        const double dt_sim  = current_sim_time - time_of_last_iteration_sim_time_;
        const double dt_wall = std::chrono::duration<double>(
            current_wall_time - time_of_last_iteration_wall_time_).count();
        RCLCPP_INFO(get_logger(), "sim clock delta t: %f s", dt_sim);
        RCLCPP_INFO(get_logger(), "wall time delta t: %f s", dt_wall);
        time_of_last_iteration_sim_time_  = current_sim_time;
        time_of_last_iteration_wall_time_ = current_wall_time;
    }
}

// ============================================================================
// Helpers
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

void CubeHwSim::grant_sim(int64_t advance_ns)
{
  if (!signed_in_) return;
  stonefish_ros2::msg::AuthorityNodeStep step;
  step.id              = authority_id_;
  step.advance.sec     = static_cast<int32_t>(advance_ns / 1000000000LL);
  step.advance.nanosec = static_cast<uint32_t>(advance_ns % 1000000000LL);
  step_pub_->publish(step);
}

void CubeHwSim::allow_sim(bool permission_to_step)
{
  if (!signed_in_) return;

  auto vote = stonefish_ros2::msg::AuthorityNodeVote();
    vote.id   = authority_id_;
    vote.vote = permission_to_step;

  // Fire-and-forget. The manager serializes votes; ordering is preserved.
  vote_pub_->publish(vote);
}

void CubeHwSim::signOutSync()
{
  // Called from rclcpp::on_shutdown — communication is still live but the
  // executor is winding down. We async-send and give the message a brief
  // moment to leave the network stack. Manager's per-authority timeout
  // is the safety net if the message never lands.
  auto req = std::make_shared<stonefish_ros2::srv::AuthorityNodeSignOut::Request>();
  req->id = authority_id_;
  signout_client_->async_send_request(req);

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  RCLCPP_INFO(get_logger(),
    "[cube_hw_sim] Sign-out request sent for ID %ld", authority_id_);
  signed_in_ = false;
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

std::optional<std::string> CubeHwSim::buildSitlJson() const
{
    if (!imu_ || !odom_)
        return std::nullopt;

    // ── IMU → FRD ────────────────────────────────────────────────────────── //
    double ax, ay, az, p, q, r;

    if (imu_frame_ == "FLU") {
        // FLU → FRD: flip Y and Z
        ax =  imu_->linear_acceleration.x;
        ay = -imu_->linear_acceleration.y;
        az = -imu_->linear_acceleration.z;
        p  =  imu_->angular_velocity.x;
        q  = -imu_->angular_velocity.y;
        r  = -imu_->angular_velocity.z;
    } else {
        // FRD: pass through unchanged
        ax = imu_->linear_acceleration.x;
        ay = imu_->linear_acceleration.y;
        az = imu_->linear_acceleration.z;
        p  = imu_->angular_velocity.x;
        q  = imu_->angular_velocity.y;
        r  = imu_->angular_velocity.z;
    }

    // ── Attitude: quaternion → ZYX Euler ─────────────────────────────────── //
    double roll{}, pitch{}, yaw{};
    
    double qw = odom_->pose.pose.orientation.w;
    double qx_q = odom_->pose.pose.orientation.x;
    double qy_q = odom_->pose.pose.orientation.y;
    double qz_q = odom_->pose.pose.orientation.z;

    eulerFromQuaternion(
        qx_q,
        qy_q,
        qz_q,
        qw,
        roll, pitch, yaw);
    
    // ── Position → NED ───────────────────────────────────────────────────── //
    double px, py, pz;
    const auto & pos = odom_->pose.pose.position;
    
    double qw_ned, qx_ned, qy_ned, qz_ned;
    if (world_frame_ == "ENU") {
        px =  pos.y;   // North
        py =  pos.x;   // East
        pz = -pos.z;   // Down

        qw_ned =  qw;   // placeholder - use attitude path for ENU
        qx_ned =  qx_q;
        qy_ned =  qy_q;
        qz_ned =  qz_q;
    } else if (world_frame_ == "NWU") {
        px =  pos.x;   // North
        py = -pos.y;   // East  (West → negate)
        pz = -pos.z;   // Down

        qw_ned =  qw;
        qx_ned =  qx_q;
        qy_ned = -qy_q;
        qz_ned = -qz_q;
    } else {
        // NED: pass through
        px = pos.x;
        py = pos.y;
        pz = pos.z;

        qw_ned = qw; qx_ned = qx_q; qy_ned = qy_q; qz_ned = qz_q;
    }

    // ── Velocity: body frame → NED ───────────────────────────────────────── //
    // Stonefish odometry twist is in body frame.
    // Rotate body → world using attitude, then world → NED.
    const double cy = std::cos(yaw),   sy = std::sin(yaw);
    const double cp = std::cos(pitch), sp = std::sin(pitch);
    const double cr = std::cos(roll),  sr = std::sin(roll);

    const double vxB = odom_->twist.twist.linear.x;
    const double vyB = odom_->twist.twist.linear.y;
    const double vzB = odom_->twist.twist.linear.z;

    // Body → world (rotation matrix, works for any of ENU/NWU/NED
    // because we use the already-corrected yaw above)
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
        // NED
        vn = deadband(vxW, velocity_deadband_);
        ve = deadband(vyW, velocity_deadband_);
        vd = deadband(vzW, velocity_deadband_);
    }

    // ── Serialise ────────────────────────────────────────────────────────── //
    const rclcpp::Time now = get_clock()->now();
    const double ts = now.nanoseconds() * 1e-9;

    std::ostringstream ss;
    ss << std::fixed;
    ss.precision(9);
    ss << "{"
      << "\"timestamp\":"     << ts << ","
      << "\"imu\":{"
        << "\"gyro\":["       << p  << "," << q  << "," << r  << "],"
        << "\"accel_body\":["  << ax << "," << ay << "," << az << "]"
      << "},"
      << "\"position\":["    << px << "," << py << "," << pz << "],"
      << "\"quaternion\":["  << qw_ned << "," << qx_ned << "," 
                              << qy_ned << "," << qz_ned << "],"
      << "\"velocity\":["    << vn << "," << ve << "," << vd << "],"
      << "\"no_time_sync\": false"
      << "}";
    return ss.str();
}

void CubeHwSim::eulerFromQuaternion(
  double qx, double qy, double qz, double qw,
  double & roll, double & pitch, double & yaw)
{
  // Roll (rotation about X)
  const double sinr_cosp = 2.0 * (qw * qx + qy * qz);
  const double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
  roll = std::atan2(sinr_cosp, cosr_cosp);

  // Pitch (rotation about Y)  — clamp to avoid NaN at gimbal lock
  const double sinp = 2.0 * (qw * qy - qz * qx);
  pitch = std::asin(std::clamp(sinp, -1.0, 1.0));

  // Yaw (rotation about Z)
  const double siny_cosp = 2.0 * (qw * qz + qx * qy);
  const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
  yaw = std::atan2(siny_cosp, cosy_cosp);
}

}  // namespace stonefish_ros2

// ============================================================================
// Standalone entry point (optional — can also be used as a component)
// ============================================================================

#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<stonefish_ros2::CubeHwSim>());
  rclcpp::shutdown();
  return 0;
}