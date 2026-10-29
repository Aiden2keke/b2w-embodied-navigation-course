#!/usr/bin/python3
"""Bounded CPU-only compiled-controller boundary probe. Never starts Gazebo.

Reordered named feedback, all force topics remapped to /probe, leased DDS domain.
"""
import argparse,json,math,os,signal,subprocess,time
from pathlib import Path
import numpy as np
import yaml
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState,Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from course_sim.session import SessionLease

def main():
    root=Path(__file__).resolve().parents[1]
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--binary',type=Path,required=True);ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--enabled',action='store_true');ap.add_argument('--body-heading',action='store_true');args=ap.parse_args()
    args.output.parent.mkdir(parents=True,exist_ok=True)
    lease=SessionLease(int(os.environ['ROS_DOMAIN_ID']))
    rclpy.init();node=rclpy.create_node('stop_brake_cpu_probe');proc=None
    result=dict(success=False,enabled=args.enabled,simulator_started=False)
    obs=[];targets=[];torques=[];sim=1.;wheel_q=[0.]*4
    try:
        until=time.monotonic()+1.
        while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.01)
        assert node.get_node_names()==['stop_brake_cpu_probe'],'Domain occupied'
        cfgpath=root/'course_sim/config/rl_sar.yaml'
        cfg=yaml.safe_load(cfgpath.read_text())['b2w_rl_sar_controller']['ros__parameters']
        log=args.output.with_suffix('.log').open('w')
        argv=[str(args.binary),'--ros-args','--params-file',str(cfgpath),'-p',
              'policy_path:='+str(root/'third_party/b2w_rl_sar/policy/b2w/robot_lab/policy.onnx'),
              '-p','stand_transition_s:=0.05','-p','differential_stop_brake:='+str(args.enabled).lower(),
              '-p','stop_brake_body_heading:='+str(args.body_heading).lower()]
        for source,dest in [('/sim/raw/joint_states','/probe/joints'),('/sim/raw/imu','/probe/imu'),
                            ('/path_manager/path_manager_ros/nav_vel','/probe/cmd'),
                            ('/network_input_debug','/probe/obs'),('/joint_commands','/probe/targets'),('/joint_torques','/probe/torques')]:
            argv+=['-r',source+':='+dest]
        for name in cfg['joint_names']:argv+=['-r','/model/b2w/joint/'+name+'/cmd_force:=/probe/force/'+name]
        proc=subprocess.Popen(argv,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        pubs={k:node.create_publisher(t,n,10) for k,t,n in [('clock',Clock,'/clock'),('joint',JointState,'/probe/joints'),('imu',Imu,'/probe/imu'),('cmd',Twist,'/probe/cmd')]}
        for name,rows in [('obs',obs),('targets',targets),('torques',torques)]:
            node.create_subscription(Float64MultiArray,'/probe/'+name,lambda msg,rows=rows:rows.append(list(msg.data)),qos_profile_sensor_data)
        until=time.monotonic()+1.2
        while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.01)
        def tick(yaw=0.,feedback=True,commands=True,invalid=False,clock=None,wheel_velocity=0.,body_yaw=0.,body_gyro=0.,invalid_imu=False):
            nonlocal sim
            sim=sim+.005 if clock is None else clock
            c=Clock();c.clock.sec=int(sim);c.clock.nanosec=round((sim%1)*1e9);pubs['clock'].publish(c)
            if feedback:
                q=list(cfg['default_dof_pos']);q[12:]=wheel_q
                if invalid:q[12]=float('nan')
                j=JointState();j.header.stamp=c.clock;j.name=cfg['joint_names'][::-1];j.position=q[::-1];j.velocity=([0.]*12+[wheel_velocity]*4)[::-1];pubs['joint'].publish(j)
                im=Imu();im.header.stamp=c.clock;im.orientation.w=math.cos(body_yaw/2);im.orientation.z=math.sin(body_yaw/2);im.angular_velocity.z=body_gyro
                if invalid_imu:im.orientation.w=float('nan')
                pubs['imu'].publish(im)
            if commands:
                cmd=Twist();cmd.angular.z=yaw;pubs['cmd'].publish(cmd)
            until=time.monotonic()+.009
            while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.001)
        def run(n,**kw):
            for _ in range(n):tick(**kw)
            assert proc.poll() is None,'Controller exited'
        def wheel():return torques[-1][12:]
        def zero():assert np.array_equal(wheel(),[0.]*4),wheel()
        run(80);zero()
        if args.body_heading:
            # Wheel positions remain fixed: heading error alone must restore yaw.
            run(32,yaw=.25);run(12)
            for i in range(1,11):run(1,body_yaw=-i*.005,body_gyro=-1.)
            run(24,body_yaw=-.05)
            expected=[.4,-.4,.4,-.4] if args.enabled else [0.]*4
            assert np.allclose(wheel(),expected,atol=1e-8),(wheel(),expected)
            result['body_error_without_wheel_travel']=True
            run(24,body_yaw=-.05,body_gyro=.2)
            expected=[.3,-.3,.3,-.3] if args.enabled else [0.]*4
            assert np.allclose(wheel(),expected,atol=1e-8),(wheel(),expected)
            result['gyro_damping_sign']=True
            run(8,body_yaw=-.05,wheel_velocity=25.)
            assert np.array_equal(wheel(),[-20.]*4),wheel()
            run(12,invalid_imu=True);run(12);zero()
            run(24,yaw=.25);run(12);run(20,body_yaw=-.02)
            run(152,commands=False);zero();run(12);zero()
            run(24,yaw=.25);run(12);run(20,body_yaw=-.02)
            run(40,feedback=False);assert np.array_equal(torques[-1],[0.]*16)
            run(16);zero()
            run(24,yaw=.25,body_yaw=3.13);run(12,body_yaw=3.13)
            run(20,body_yaw=-3.13,body_gyro=.1)
            expected=[-8*(2*math.pi-6.26)-.05,8*(2*math.pi-6.26)+.05]*2 if args.enabled else [0.]*4
            assert np.allclose(wheel(),expected,atol=1e-8),(wheel(),expected)
            run(12);zero() # discontinuous heading reset
            run(24,yaw=.25);run(12);run(24,body_yaw=-.02)
            tick(clock=.5);run(40);zero()
            result.update(heading_wrap=True,heading_reset_release=True,invalid_imu_release=True)
        else:
            # Startup zero commands must not create a reference as wheel positions vary.
            wheel_q[:]=[.05,-.05,.05,-.05];run(24);zero()
            run(32,yaw=.25);run(12)  # qualified fresh active->exact-zero, first d=.1
            for _ in range(20):
                wheel_q[:]=[wheel_q[0]-.005,wheel_q[1]+.005,wheel_q[2]-.005,wheel_q[3]+.005];tick()
            run(24)
            expected=[.4,-.4,.4,-.4] if args.enabled else [0.]*4
            assert np.allclose(wheel(),expected,atol=1e-8),(wheel(),expected)
            result['held_torque']=wheel();result['qualified_capture_and_signed_torque']=True
            run(8,wheel_velocity=25.)
            assert np.array_equal(wheel(),[-20.]*4),wheel()
            result['actual_total_torque_clamp']=True
            run(12)
            assert all(not any(v[12:]) for v in targets[-6:]),'Nonzero stop velocity target'
            # Invalid new feedback releases immediately, without waiting for old cache age.
            run(1,invalid=True);run(12);zero()
            run(24,yaw=.25);run(12)
            wheel_q[:]=[v*.8 for v in wheel_q];run(20)
            run(152,commands=False);zero();run(12);zero() # command timeout and no automatic rearm
            run(24,yaw=.25);run(12);wheel_q[:]=[v*.7 for v in wheel_q];run(20)
            run(40,feedback=False);assert np.array_equal(torques[-1],[0.]*16),torques[-1]
            run(16);zero()  # feedback reactivation does not resurrect the reference
            # Wrapped wheel angles keep the shortest continuous increment; reset is fail-closed.
            wheel_q[:]=[3.13]*4;run(24,yaw=.25);run(12)
            wheel_q[:]=[-3.13,3.13,-3.13,3.13];run(20)
            expected=[-2*(2*math.pi-6.26),2*(2*math.pi-6.26)]*2 if args.enabled else [0.]*4
            assert np.allclose(wheel(),expected,atol=1e-8),(wheel(),expected)
            wheel_q[:]=[0.]*4;run(12);zero()
            run(24,yaw=.25);run(12);wheel_q[:]=[.05,-.05,.05,-.05];run(24)
            tick(clock=.5);run(40);zero()
        if args.enabled:
            from rcl_interfaces.srv import SetParameters
            from rclpy.parameter import Parameter
            client=node.create_client(SetParameters,'/b2w_rl_sar_controller/set_parameters')
            def set_brake(value):
                f=client.call_async(SetParameters.Request(parameters=[Parameter('differential_stop_brake',value=value).to_parameter_msg()]))
                until=time.monotonic()+2.
                while not f.done() and time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.01)
                assert f.done()
                return f.result().results[0].successful
            run(24,yaw=.25);run(12);wheel_q[:]=[.1,-.1,.1,-.1];run(24,body_yaw=-.02 if args.body_heading else 0.)
            assert set_brake(False)
            run(8);zero()
            assert not set_brake(True),'Reactivation without restart must reject'
            wheel_q[:]=[.05,-.05,.05,-.05];run(8);zero()
            result['runtime_disable_releases_and_reenable_rejects']=True
        # Same source PD formula for disabled stop, all legs and all finite torque limits.
        assert np.isfinite(torques).all()
        assert np.all(np.abs(torques)<=np.array(cfg['torque_limits'])+1e-9)
        assert np.array_equal(targets[-1][12:],[0.]*4)
        np.savez_compressed(args.output.with_suffix('.npz'),observations=obs,targets=targets,torques=torques)
        result.update(success=True,observations=len(obs),torques=len(torques),exact_zero_targets=True,
            invalid_feedback_release=not args.body_heading,stale_feedback_zero_torque=True,command_timeout_release=True,
            no_rearm_on_zero=True,wrapped_angles=not args.body_heading,position_reset_release=not args.body_heading,clock_reset_release=True,
            torque_saturation_respected=True,active_motion_policy_exercised=True)
    except BaseException as error:
        result['error']=repr(error)
    finally:
        if proc is not None:
            proc.send_signal(signal.SIGINT)
            try:proc.wait(timeout=3)
            except subprocess.TimeoutExpired:proc.terminate();proc.wait(timeout=2)
            result['controller_exit']=proc.returncode;result['remaining']=[] if proc.poll() is not None else [proc.pid]
        node.destroy_node();rclpy.shutdown();lease.close()
        args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2));return 0 if result['success'] else 1
if __name__=='__main__':raise SystemExit(main())
