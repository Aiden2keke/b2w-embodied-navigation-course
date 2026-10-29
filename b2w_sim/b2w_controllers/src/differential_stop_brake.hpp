#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

// A stop-position spring in wheel differential coordinates. No velocity target.
// The caller supplies command/feedback freshness and preserves all torque clamps.
class DifferentialStopBrake {
 public:
  using Wheels = std::array<double,4>;  // FR, FL, RR, RL
  DifferentialStopBrake(double stiffness=4., double cap=.75, double slew=5.,
                        bool body_heading=false,double heading_k=8.,double heading_d=.5)
      : stiffness_(stiffness), cap_(cap), slew_(slew), body_heading_(body_heading),
        heading_k_(heading_k), heading_d_(heading_d) {
    if (!std::isfinite(stiffness) || stiffness<=0 || !std::isfinite(cap) || cap<=0 ||
        !std::isfinite(slew) || slew<=0 || !std::isfinite(heading_k) || heading_k<=0 ||
        !std::isfinite(heading_d) || heading_d<0) throw std::invalid_argument("Invalid stop brake gains");
  }
  void release() { armed_=holding_=false; scalar_=0; output_time_=NAN; }
  void invalidate() { release(); have_sample_=have_heading_=false; }
  bool sample_valid() const { return have_sample_ && (!body_heading_ || have_heading_); }
  double heading() const { return heading_; }
  bool sample_heading(double yaw,double gyro_z,double stamp) {
    if (!std::isfinite(yaw) || !std::isfinite(gyro_z) || !std::isfinite(stamp)) {
      invalidate(); return false;
    }
    if(have_heading_) {
      double dt=stamp-heading_time_;
      if(dt==0 && yaw==raw_heading_ && gyro_z==heading_rate_) return true;
      double delta=std::remainder(yaw-raw_heading_,2.*3.14159265358979323846);
      if(dt<=0 || dt>.15 || std::max(std::abs(gyro_z),std::abs(heading_rate_))*dt>=3. ||
          std::abs(delta)>.03+std::max(std::abs(gyro_z),std::abs(heading_rate_))*dt) {
        invalidate();return false;
      }
      heading_+=delta;
    } else {heading_=yaw;have_heading_=true;}
    raw_heading_=yaw;heading_rate_=gyro_z;heading_time_=stamp;
    return true;
  }
  bool holding() const { return holding_; }
  double reference() const { return reference_; }
  double differential() const { return (angle_[0]+angle_[2]-angle_[1]-angle_[3])/2.; }
  bool sample(const Wheels &q,const Wheels &dq,double stamp) {
    if (!std::isfinite(stamp)) { invalidate(); return false; }
    for (int i=0;i<4;++i) if (!std::isfinite(q[i]) || !std::isfinite(dq[i])) {
      invalidate(); return false;
    }
    if (have_sample_) {
      double dt=stamp-sample_time_;
      if (dt==0 && q==raw_ && dq==velocity_) return true;
      if (dt<=0 || dt>.15) { invalidate(); return false; }
      Wheels next=angle_;
      for (int i=0;i<4;++i) {
        // Source continuous angles and +/-pi wrapped angles use the same small
        // increments. Discontinuous resets/inconsistent motion release the hold.
        double delta=std::remainder(q[i]-raw_[i],2.*3.14159265358979323846);
        if (std::max(std::abs(dq[i]),std::abs(velocity_[i]))*dt>=3.04159265358979323846 ||
            std::abs(delta)>.1+std::max(std::abs(dq[i]),std::abs(velocity_[i]))*dt+1e-9 ||
            std::abs(delta)>=3.14159265358979323846) { invalidate(); return false; }
        next[i]+=delta;
      }
      angle_=next;
    } else { angle_=q; have_sample_=true; }
    raw_=q; velocity_=dq; sample_time_=stamp;
    return true;
  }
  void command(bool active,bool exact_zero,bool qualified) {
    if (!qualified || !sample_valid()) { release(); return; }
    if (active) { release(); armed_=true; return; }
    if (!exact_zero) { release(); return; }
    if (armed_ && !holding_) {
      reference_=body_heading_?heading_:differential(); holding_=true; armed_=false;
      scalar_=0; output_time_=NAN;
    }
  }
  double output(double now) {
    if (!holding_) return 0.;
    if (!std::isfinite(now)) { release(); return 0.; }
    if (!std::isfinite(output_time_)) { output_time_=now; return 0.; }
    double dt=now-output_time_;
    if (dt<0 || dt>.15) { release(); return 0.; }
    // No integral accumulator: no hidden windup at the cap or after release.
    double error=body_heading_?std::remainder(reference_-heading_,2.*3.14159265358979323846):reference_-differential();
    double target=std::clamp(body_heading_?heading_k_*error-heading_d_*heading_rate_:
        .5*stiffness_*error,-cap_,cap_);
    scalar_+=std::clamp(target-scalar_,-slew_*dt,slew_*dt);
    output_time_=now;
    return scalar_;  // add [+s,-s,+s,-s], then clamp TOTAL wheel torque
  }
 private:
  double stiffness_,cap_,slew_,reference_=0,scalar_=0,sample_time_=0,output_time_=NAN;
  bool body_heading_=false,have_heading_=false;
  double heading_k_=8.,heading_d_=.5,heading_=0,raw_heading_=0,heading_rate_=0,heading_time_=0;
  Wheels raw_{},angle_{},velocity_{};
  bool have_sample_=false,armed_=false,holding_=false;
};
