"""Offline contracts: a wrong inverse, implicit fallback or changed default must fail."""
import ast
import math
import random
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'course_navigation'))


def candidate():
    from course_navigation import m2_inverse
    return m2_inverse


def test_candidate_exists():
    assert (ROOT / 'course_navigation/course_navigation/m2_inverse.py').is_file()


def test_exact_inverse_endpoints_monotonic_and_input_support():
    m = candidate()
    values = [m.V_MIN + (m.V_MAX-m.V_MIN)*i/1000 for i in range(1001)]
    outputs = [m.inverse(v) for v in values]
    assert outputs == sorted(outputs)
    assert all(m.U_MIN <= u <= m.U_MAX for u in outputs)
    assert outputs[0] == m.U_MIN
    assert outputs[-1] == (m.V_MAX-m.B)/m.A
    for v,u in zip(values, outputs):
        assert u == pytest.approx((v-.0659258536489146)/1.0245445280935277, abs=1e-17)
        assert m.A*u+m.B == pytest.approx(v, abs=1e-16)


def test_zero_is_separate_and_all_unsupported_values_rejected():
    m = candidate()
    assert m.inverse(0.) == 0.
    for v in [math.nextafter(0.,1.), .025, .10, math.nextafter(m.V_MIN,0.),
              math.nextafter(m.V_MAX,math.inf), .4, -.1, math.nan, math.inf]:
        with pytest.raises(m.UnsupportedVelocity, match='unsupported'):
            m.inverse(v)


def test_disabled_returns_identical_tuple_and_enabled_preserves_other_axes():
    m = candidate()
    rng = random.Random(29)
    for _ in range(1000):
        command = tuple(rng.uniform(-2,2) for _ in range(3))
        assert m.transform(command) is command
    command = (.15, -.123, .456)
    assert m.transform(command, enabled=True) == (m.inverse(.15), command[1],command[2])
    assert m.transform((0.,.1,-.2), enabled=True) == (0.,.1,-.2)


def test_adapter_default_path_and_legacy_gate_equivalence():
    # Execute only the real ROS-free gate AST from old and working sources.
    old=subprocess.check_output(['git','show','HEAD:course_navigation/course_navigation/velocity_adapter.py'],cwd=ROOT,text=True)
    new=(ROOT/'course_navigation/course_navigation/velocity_adapter.py').read_text()
    def gate(source):
        tree=ast.parse(source)
        node=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='VelocityGate')
        ns={'math':math}; exec(compile(ast.Module(body=[node],type_ignores=[]),'gate','exec'),ns)
        return ns['VelocityGate']()
    a,b=gate(old),gate(new)
    for now,cmd in enumerate([(0.,0.,0.),(.8,.6,1.),(-.2,.1,-1.),(math.nan,0.,0.)]):
        for g in (a,b):g.state_time=now;g.set_command(cmd,now)
        for dt in [-.1,0.,.49,.51]:
            assert candidate().transform(b.output(now+dt)) == a.output(now+dt)
    assert "declare_parameter('m2_inverse_enabled', False)" in new
    assert 'm2_inverse.transform' in new


def test_identity_and_open_space_supported_points():
    m=candidate(); ident=m.identity()
    assert ident['version']=='m2-strict-v1'
    assert ident['frozen_b_sha256']=='15b371aa80afe5cef8461d077fae0bc8bb0a180ae92ea5ff6c669c9b04d352bc'
    assert len(ident['sha256'])==64
    from scripts.m2_open_space import plan
    p=plan()
    assert p['desired_vx_mps']==[.12,.15,.20,.25]
    assert p['candidate']==ident
    assert all(m.U_MIN <= u <= m.U_MAX for u in p['policy_u'])
    assert p['prospective_gates']['truth_max_abs_error_mps']==.010
    assert p['prospective_gates']['truth_rmse_mps']==.007
    assert p['dynamic_nav2_authorized'] is False


def test_real_adapter_callback_rejects_before_publication_and_preserves_default():
    from types import SimpleNamespace as NS
    m=candidate()
    source=(ROOT/'course_navigation/course_navigation/velocity_adapter.py').read_text()
    tree=ast.parse(source)
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='VelocityAdapter')
    published=[];diagnostics=[]
    class Twist:
        def __init__(self):self.linear=NS(x=0.,y=0.);self.angular=NS(z=0.)
    class TransformException(Exception):pass
    ns={'Node':object,'m2_inverse':m,'Twist':Twist,'TransformException':TransformException,
        'Time':lambda:None,'time':NS(monotonic=lambda:1.),'json':__import__('json')}
    exec(compile(ast.Module(body=[cls],type_ignores=[]),'adapter','exec'),ns)
    adapter=object.__new__(ns['VelocityAdapter']);adapter.m2_enabled=False;adapter.require_tf=False
    adapter.gate=NS(output=lambda now:(.05,-.2,.3),set_command=lambda cmd,t:diagnostics.append(('gate',cmd)))
    adapter.pub=NS(publish=lambda msg:published.append((msg.linear.x,msg.linear.y,msg.angular.z)))
    adapter.report_m2=lambda *args:diagnostics.append(args)
    adapter.get_logger=lambda:NS(fatal=lambda msg:diagnostics.append(('fatal',msg)))
    adapter.publish();assert published==[(.05,-.2,.3)]
    adapter.m2_enabled=True
    with pytest.raises(m.UnsupportedVelocity):adapter.publish()
    assert published==[(.05,-.2,.3)]
    assert diagnostics[-2][2]=='unsupported'
    msg=Twist();msg.linear.x=.4
    with pytest.raises(m.UnsupportedVelocity):adapter.command(msg)
    assert not any(d[0]=='gate' for d in diagnostics)
    adapter.gate.output=lambda now:(.15,-.2,.3)
    adapter.publish();assert published[-1]==(m.inverse(.15),-.2,.3)


def test_offline_cli_has_no_ros_import_or_live_side_effect(tmp_path):
    code="""import sys,runpy
class RejectROS:
 def find_spec(self,fullname,*args):
  if fullname.split('.')[0] in {'rclpy','geometry_msgs','nav_msgs','tf2_ros','launch','launch_ros'}:raise AssertionError(fullname)
sys.meta_path.insert(0,RejectROS())
sys.argv=['scripts/m2_open_space.py']
runpy.run_path('scripts/m2_open_space.py',run_name='__main__')
"""
    r=subprocess.run([sys.executable,'-c',code],cwd=ROOT,capture_output=True,text=True)
    assert r.returncode==0,r.stderr
    assert __import__('json').loads(r.stdout)['dynamic_nav2_authorized'] is False
