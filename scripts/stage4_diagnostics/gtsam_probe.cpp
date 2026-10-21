#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/inference/Symbol.h>
#include <iostream>
#include <string>
using namespace gtsam;
int main(int argc,char**argv){
 std::string phase="initial",mode=argc>1?argv[1]:"CHOLESKY";double dt=argc>2?std::stod(argv[2]):.005;
 try {
 ISAM2Params params;params.relinearizeThreshold=.1;params.relinearizeSkip=1;params.setFactorization(mode);ISAM2 optimizer(params);
 Symbol x0('x',0),v0('v',0),b0('b',0),x1('x',1),v1('v',1),b1('b',1);
 Pose3 pose;Vector3 velocity=Vector3::Zero();imuBias::ConstantBias bias;
 NonlinearFactorGraph graph;Values values;
 graph.add(PriorFactor<Pose3>(x0,pose,noiseModel::Diagonal::Sigmas(Vector6::Constant(.01))));
 graph.add(PriorFactor<Vector3>(v0,velocity,noiseModel::Isotropic::Sigma(3,1e4)));
 graph.add(PriorFactor<imuBias::ConstantBias>(b0,bias,noiseModel::Isotropic::Sigma(6,1e-3)));
 values.insert(x0,pose);values.insert(v0,velocity);values.insert(b0,bias);
 optimizer.update(graph,values);std::cout<<"initial_priors=PASS ";
 phase="imu_step";graph.resize(0);values.clear();auto p=PreintegrationParams::MakeSharedU(9.81);
 p->accelerometerCovariance=Matrix33::Identity()*1e-4;p->gyroscopeCovariance=Matrix33::Identity()*1e-6;p->integrationCovariance=Matrix33::Identity()*1e-8;
 PreintegratedImuMeasurements pre(p,bias);if(dt>0)pre.integrateMeasurement(Vector3(0,0,9.81),Vector3::Zero(),dt);
 graph.add(ImuFactor(x0,v0,x1,v1,b0,pre));
 Vector6 noise;noise<<6.435665935353257e-5,6.435665935353257e-5,6.435665935353257e-5,3.564031869636761e-5,3.564031869636761e-5,3.564031869636761e-5;
 graph.add(BetweenFactor<imuBias::ConstantBias>(b0,b1,bias,noiseModel::Diagonal::Sigmas(std::sqrt(pre.deltaTij())*noise)));
 graph.add(PriorFactor<Pose3>(x1,pose,noiseModel::Diagonal::Sigmas((Vector6()<<.05,.05,.05,.1,.1,.1).finished())));
 NavState predicted=pre.predict(NavState(pose,velocity),bias);values.insert(x1,predicted.pose());values.insert(v1,predicted.v());values.insert(b1,bias);
 optimizer.update(graph,values);optimizer.update();std::cout<<"imu_step=PASS dt="<<pre.deltaTij()<<" factorization="<<mode<<"\n";return 0;
 }catch(const std::exception&e){std::cout<<"FAIL phase="<<phase<<" factorization="<<mode<<" dt="<<dt<<" what="<<e.what()<<"\n";return 3;}
}
