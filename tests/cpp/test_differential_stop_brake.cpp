#include "differential_stop_brake.hpp"
#include <cassert>
#include <cmath>
#include <limits>
using W=std::array<double,4>;
bool near(double a,double b){return std::abs(a-b)<1e-9;}
int main(){
  DifferentialStopBrake b(4.,.75,5.);
  assert(b.sample(W{1,1,1,1},W{},1.));
  b.command(false,true,true); assert(!b.holding()); // initial zero never arms
  b.command(true,false,true);b.command(false,true,true);assert(b.holding());
  assert(near(b.output(1.),0));
  assert(b.sample(W{.9,1.1,.9,1.1},W{},1.01));
  assert(near(b.output(1.005),.025));assert(near(b.output(1.01),.05)); // torque slew
  assert(near(b.output(1.1),.4));
  b.command(false,true,true);assert(near(b.reference(),0)); // repeated zero never recaptures
  b.command(true,false,true);assert(!b.holding());
  b.command(false,true,true);assert(near(b.reference(),-.2));assert(near(b.output(1.1),0));
  b.release();b.command(false,true,true);assert(!b.holding()); // timeout/invalid state cannot rearm
  b.invalidate();assert(!b.sample_valid());
  assert(b.sample(W{3.13,3.13,3.13,3.13},W{},2.));b.command(true,false,true);b.command(false,true,true);
  assert(b.sample(W{-3.13,3.13,-3.13,3.13},W{},2.01));assert(near(b.differential(),2*3.141592653589793-6.26)); // wrap without reference jump
  b.output(2.);assert(b.output(2.01)<0);
  assert(!b.sample(W{0,0,0,0},W{},2.02));assert(!b.holding()); // inconsistent position reset
  assert(b.sample(W{},W{},3.));b.command(true,false,true);b.command(false,true,true);b.output(3.);
  assert(b.sample(W{.1,-.1,.1,-.1},W{},3.01));assert(near(b.output(3.01),-.05));
  b.release();assert(near(b.output(3.02),0));
  assert(!b.sample(W{NAN,0,0,0},W{},3.03));
  DifferentialStopBrake capped(1000.,.75,1000.);capped.sample(W{},W{},4.);capped.command(true,false,true);capped.command(false,true,true);capped.output(4.);
  capped.sample(W{.1,-.1,.1,-.1},W{},4.01);assert(near(capped.output(4.01),-.75));
  DifferentialStopBrake ambiguous;ambiguous.sample(W{},W{400,400,400,400},5.);
  assert(!ambiguous.sample(W{4,4,4,4},W{400,400,400,400},5.01));
  capped.output(3.);assert(!capped.holding()); // backward clock

  // Body heading must react even when wheel differential stays exactly fixed.
  DifferentialStopBrake h(4.,.75,5.,true,8.,.5);
  h.sample(W{},W{},10.); h.sample_heading(3.13,0.,10.);
  h.command(false,true,true);assert(!h.holding());
  h.command(true,false,true);h.command(false,true,true);h.output(10.);
  h.sample(W{},W{},10.01);h.sample_heading(-3.13,.1,10.01);
  assert(near(h.heading(),3.13+2*3.141592653589793-6.26));
  assert(h.output(10.01)<0); // positive measured yaw needs negative restoring torque
  h.sample(W{},W{},10.1);h.sample_heading(3.08,-.9,10.1);
  h.sample_heading(3.08,-.1,10.11);
  assert(near(h.output(10.11),.45)); // .05rad*8 - .5*(-.1), no wheel travel
  h.command(false,true,true);assert(near(h.reference(),3.13));
  // No integral windup: saturation clears immediately when heading returns.
  DifferentialStopBrake fast(4.,.75,1000.,true,8.,.5);
  fast.sample(W{},W{},20.);fast.sample_heading(0.,0.,20.);
  fast.command(true,false,true);fast.command(false,true,true);fast.output(20.);
  for(int i=1;i<=20;++i){double t=20.+i*.01;fast.sample(W{},W{},t);fast.sample_heading(-.01*i,-1.,t);fast.output(t);}
  assert(near(fast.output(20.2),.75));
  for(int i=1;i<=20;++i){double t=20.2+i*.01;fast.sample(W{},W{},t);fast.sample_heading(-.2+.01*i,1.,t);fast.output(t);}
  fast.sample_heading(0.,0.,20.41);assert(near(fast.output(20.41),0));
  fast.invalidate();fast.sample(W{},W{},21.);fast.sample_heading(0.,0.,21.);
  fast.command(false,true,true);assert(!fast.holding());
  fast.command(true,false,true);fast.command(false,true,true);
  assert(!fast.sample_heading(1.,0.,21.01));assert(!fast.holding()); // discontinuity
  fast.sample_heading(0.,0.,22.);fast.sample(W{},W{},22.);
  fast.command(true,false,true);fast.command(false,true,true);
  assert(!fast.sample_heading(NAN,0.,22.01));assert(!fast.holding());
  fast.sample_heading(0.,0.,23.);fast.sample(W{},W{},23.);
  fast.command(true,false,true);fast.command(false,true,true);
  assert(!fast.sample_heading(0.,0.,23.2));assert(!fast.holding()); // feedback gap
}
