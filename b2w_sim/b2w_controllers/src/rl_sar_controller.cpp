// B2-W course adapter for the rl_sar/robot_lab feed-forward policy.
// Observation and action contracts: fan-ziqi/rl_sar (Apache-2.0), pinned in POLICY_SOURCE.json.
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/create_timer.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <onnxruntime_cxx_api.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <vector>

class RLSarController : public rclcpp::Node {
 public:
  RLSarController() : Node("b2w_rl_sar_controller"), env_(ORT_LOGGING_LEVEL_WARNING, "b2w_rl_sar") {
    names_ = declare_parameter<std::vector<std::string>>("joint_names", std::vector<std::string>{});
    ref_ = declare_parameter<std::vector<double>>("default_dof_pos", std::vector<double>{});
    scale_ = declare_parameter<std::vector<double>>("action_scale", std::vector<double>{});
    kp_ = declare_parameter<std::vector<double>>("rl_kp", std::vector<double>{});
    kd_ = declare_parameter<std::vector<double>>("rl_kd", std::vector<double>{});
    stand_kp_ = declare_parameter<std::vector<double>>("fixed_kp", std::vector<double>{});
    stand_kd_ = declare_parameter<std::vector<double>>("fixed_kd", std::vector<double>{});
    limits_ = declare_parameter<std::vector<double>>("torque_limits", std::vector<double>{});
    for (size_t n : {names_.size(), ref_.size(), scale_.size(), kp_.size(), kd_.size(),
                     stand_kp_.size(), stand_kd_.size(), limits_.size()}) {
      if (n != 16) throw std::runtime_error("rl_sar configuration requires 16 joints and gains");
    }
    for (size_t i = 0; i < 16; ++i) {
      if (!std::isfinite(ref_[i]) || !std::isfinite(scale_[i]) || !std::isfinite(kp_[i]) ||
          !std::isfinite(kd_[i]) || !std::isfinite(limits_[i]) || limits_[i] <= 0 || kp_[i] < 0 || kd_[i] < 0)
        throw std::runtime_error("Invalid joint gains or limits");
      pubs_.push_back(create_publisher<std_msgs::msg::Float64>("/model/b2w/joint/" + names_[i] + "/cmd_force", 1));
    }
    timeout_ = declare_parameter<double>("command_timeout_s", .5);
    feedback_timeout_ = declare_parameter<double>("feedback_timeout_s", .15);
    warmup_ = declare_parameter<double>("stand_transition_s", 2.0);
    idle_hold_ = declare_parameter<bool>("idle_hold", false);
    yaw_gain_ = declare_parameter<double>("yaw_command_gain", 1.0);
    if (!std::isfinite(timeout_) || timeout_<=0 || !std::isfinite(feedback_timeout_) || feedback_timeout_<=0 ||
        !std::isfinite(warmup_) || warmup_<=0 || !std::isfinite(yaw_gain_) || yaw_gain_<=0)
      throw std::runtime_error("Invalid controller timeouts or command gain");
    auto policy = declare_parameter<std::string>("policy_path", "");
    if (!std::filesystem::is_regular_file(policy)) throw std::runtime_error("Missing rl_sar policy: " + policy);
    Ort::SessionOptions options;
    options.SetIntraOpNumThreads(1);
    session_ = std::make_unique<Ort::Session>(env_, policy.c_str(), options);
    if (session_->GetInputCount() != 1 || session_->GetOutputCount() != 1 ||
        session_->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape().back() != 57 ||
        session_->GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape().back() != 16)
      throw std::runtime_error("Policy must have one 57D input and one 16D output");
    target_ = ref_;
    // The public /joint_states stream is decimated and relayed by the sensor
    // disturbance process. Low-level PD needs the newest physics feedback.
    auto feedback_qos = rclcpp::SensorDataQoS().keep_last(1);
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>("/sim/raw/joint_states", feedback_qos,
      [this](sensor_msgs::msg::JointState::SharedPtr m) {
        handleClockReset();
        auto stamp=rclcpp::Time(m->header.stamp,get_clock()->get_clock_type());
        double lag=(now()-stamp).seconds();
        if (lag < -.02 || lag > feedback_timeout_) return;
        std::array<double,16> q{}, dq{};
        for (size_t i=0; i<16; ++i) {
          auto it=std::find(m->name.begin(),m->name.end(),names_[i]);
          if (it==m->name.end()) return;
          size_t k=std::distance(m->name.begin(),it);
          if (k>=m->position.size() || k>=m->velocity.size() ||
              !std::isfinite(m->position[k]) || !std::isfinite(m->velocity[k])) return;
          q[i]=m->position[k]; dq[i]=m->velocity[k];
        }
        q_=q; dq_=dq; joint_time_=stamp;
        if (!have_joints_) { initial_=q; start_=now(); have_joints_=true; }
      });
    // imu_link is aligned with base_link. Use the actual body gyro, not the
    // smoothed Euler derivatives published by Fortress OdometryPublisher.
    body_sub_ = create_subscription<sensor_msgs::msg::Imu>("/sim/raw/imu", feedback_qos,
      [this](sensor_msgs::msg::Imu::SharedPtr m) {
        handleClockReset();
        auto stamp=rclcpp::Time(m->header.stamp,get_clock()->get_clock_type());
        double lag=(now()-stamp).seconds();
        if (lag < -.02 || lag > feedback_timeout_) return;
        const auto &q=m->orientation;
        Eigen::Quaterniond rot(q.w,q.x,q.y,q.z);
        const auto &v=m->angular_velocity;
        if (!rot.coeffs().allFinite() || rot.norm()<.5 || !std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z)) return;
        gravity_=rot.normalized().toRotationMatrix().transpose()*Eigen::Vector3d(0,0,-1);
        gyro_={v.x,v.y,v.z}; body_time_=stamp; have_body_=true;
      });
    cmd_sub_=create_subscription<geometry_msgs::msg::Twist>("/path_manager/path_manager_ros/nav_vel",10,
      [this](geometry_msgs::msg::Twist::SharedPtr m) {
        handleClockReset();
        if (!std::isfinite(m->linear.x) || !std::isfinite(m->linear.y) || !std::isfinite(m->angular.z)) return;
        command_={std::clamp(m->linear.x,-1.,1.),std::clamp(m->linear.y,-.5,.5),std::clamp(m->angular.z,-1.,1.)};
        cmd_time_=now(); have_command_=true;
      });
    debug_=create_publisher<std_msgs::msg::Float64MultiArray>("/network_input_debug",10);
    actions_pub_=create_publisher<std_msgs::msg::Float64MultiArray>("/joint_commands",10);
    torques_pub_=create_publisher<std_msgs::msg::Float64MultiArray>("/joint_torques",10);
    rcl_jump_threshold_t threshold{};
    threshold.on_clock_change=false;
    threshold.min_backward.nanoseconds=-1;
    // The time-source thread only signals; state is reset in the executor.
    jump_handler_=get_clock()->create_jump_callback(nullptr,
      [this](const rcl_time_jump_t &jump) { if (jump.delta.nanoseconds<0) reset_pending_.store(true); },threshold);
    timer_=rclcpp::create_timer(this,get_clock(),std::chrono::milliseconds(5),[this]{update();});
    RCLCPP_INFO(get_logger(),"Loaded rl_sar B2-W policy: 57 observations, 16 actions, 50 Hz policy / 200 Hz torque, %s",policy.c_str());
  }
 private:
  void handleClockReset() {
    if (!reset_pending_.exchange(false)) return;
    have_joints_=false; have_body_=false; have_command_=false;
    last_.fill(0); velocity_target_.fill(0); target_=ref_; tick_=0;
    publishTorques(std::array<double,16>{});
    RCLCPP_WARN(get_logger(),"ROS clock reset: cleared feedback, actions and command; waiting to stand again");
  }
  void update() {
    handleClockReset();
    auto time=now();
    if (!have_joints_ || !have_body_ || (time-joint_time_).seconds()>feedback_timeout_ ||
        (time-body_time_).seconds()>feedback_timeout_ ||
        (time-joint_time_).seconds()<-.02 || (time-body_time_).seconds()<-.02) {
      publishTorques(std::array<double,16>{});
      return;
    }
    double age=(time-start_).seconds();
    if (age<0) { start_=time; age=0; initial_=q_; last_.fill(0); tick_=0; }
    bool command_valid=have_command_ && (time-cmd_time_).seconds()>=0 && (time-cmd_time_).seconds()<timeout_;
    bool idle=idle_hold_ && (!command_valid || (std::abs(command_[0])<.01 &&
              std::abs(command_[1])<.01 && std::abs(command_[2])<.01));
    bool standing=age<warmup_;
    if (standing) {
      double f=std::clamp(age/warmup_,0.,1.);
      for (size_t i=0;i<16;++i) target_[i]=i<12?initial_[i]+f*(ref_[i]-initial_[i]):ref_[i];
      velocity_target_.fill(0);
    } else if (tick_%4==0) {
      std::array<float,57> obs{};
      for (size_t i=0;i<3;++i) {
        obs[i]=gyro_[i]*.25;
        obs[3+i]=gravity_[i];
        obs[6+i]=command_valid?command_[i]:0;
      }
      obs[8]=std::clamp(obs[8]*float(yaw_gain_),-1.f,1.f);
      for (size_t i=0;i<16;++i) {
        obs[9+i]=i<12?q_[i]-ref_[i]:0;
        obs[25+i]=dq_[i]*.05;
        obs[41+i]=last_[i];
      }
      for (auto &v:obs) v=std::clamp(v,-100.f,100.f);
      auto mem=Ort::MemoryInfo::CreateCpu(OrtArenaAllocator,OrtMemTypeDefault);
      std::array<int64_t,2> shape{1,57};
      auto tensor=Ort::Value::CreateTensor<float>(mem,obs.data(),obs.size(),shape.data(),2);
      const char *inputs[]{"observations"}, *outputs[]{"actions"};
      try {
        auto result=session_->Run(Ort::RunOptions{nullptr},inputs,&tensor,1,outputs,1);
        const float *a=result[0].GetTensorData<float>();
        for (size_t i=0;i<16;++i) if (!std::isfinite(a[i])) throw std::runtime_error("Non-finite action");
        for (size_t i=0;i<16;++i) {
          last_[i]=std::clamp(double(a[i]),-100.,100.);
          // Retain learned leg balance while braking the wheels. Replacing
          // learned leg targets with the nominal pose pitched the body.
          if (idle && i>=12) last_[i]=0;
          target_[i]=ref_[i]+(i<12?scale_[i]*last_[i]:0);
          velocity_target_[i]=i>=12?scale_[i]*last_[i]:0;
        }
      } catch (const std::exception &e) {
        RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),2000,"Policy inference failed: %s",e.what());
        publishTorques(std::array<double,16>{}); return;
      }
      std_msgs::msg::Float64MultiArray m; m.data.assign(obs.begin(),obs.end()); debug_->publish(m);
      m.data=target_; for (size_t i=12;i<16;++i) m.data[i]=velocity_target_[i]; actions_pub_->publish(m);
    }
    ++tick_;
    std::array<double,16> tau{};
    for (size_t i=0;i<16;++i) {
      double kp=standing?stand_kp_[i]:kp_[i], kd=standing?stand_kd_[i]:kd_[i];
      tau[i]=std::clamp(kp*(target_[i]-q_[i])+kd*(velocity_target_[i]-dq_[i]),-limits_[i],limits_[i]);
    }
    publishTorques(tau);
  }
  void publishTorques(const std::array<double,16> &tau) {
    for (size_t i=0;i<16;++i) {std_msgs::msg::Float64 m; m.data=tau[i];pubs_[i]->publish(m);}
    std_msgs::msg::Float64MultiArray m; m.data.assign(tau.begin(),tau.end()); torques_pub_->publish(m);
  }
  Ort::Env env_; std::unique_ptr<Ort::Session> session_;
  std::vector<std::string> names_; std::vector<double> ref_,scale_,kp_,kd_,stand_kp_,stand_kd_,limits_,target_;
  std::array<double,16> q_{},dq_{},initial_{},last_{},velocity_target_{};
  std::array<double,3> gyro_{},command_{}; Eigen::Vector3d gravity_{0,0,-1};
  bool have_joints_=false,have_body_=false,have_command_=false,idle_hold_=false; size_t tick_=0;
  double timeout_,feedback_timeout_,warmup_,yaw_gain_;
  rclcpp::Time start_{0,0,RCL_ROS_TIME},joint_time_{0,0,RCL_ROS_TIME},body_time_{0,0,RCL_ROS_TIME},cmd_time_{0,0,RCL_ROS_TIME};
  std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> pubs_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_,actions_pub_,torques_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr body_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::JumpHandler::SharedPtr jump_handler_;
  std::atomic<bool> reset_pending_{false};
};
int main(int argc,char **argv) {rclcpp::init(argc,argv);try {rclcpp::spin(std::make_shared<RLSarController>());}catch(const std::exception &e){fprintf(stderr,"B2-W rl_sar startup failed: %s\n",e.what());rclcpp::shutdown();return 1;}rclcpp::shutdown();return 0;}
