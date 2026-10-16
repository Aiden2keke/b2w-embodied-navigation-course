"""Pure offline forward models and wheel semantics. No ROS or process imports."""
from __future__ import annotations

import bisect
import math
from statistics import mean, stdev

WHEELS = ('FR_foot_joint','FL_foot_joint','RR_foot_joint','RL_foot_joint')


def axis_forward_sign(axis):
    """At a bottom contact, positive +y spin needs +x hub motion: v=-ω×(-r z)."""
    x,y,z=map(float,axis)
    if abs(y)<.9 or abs(x)>.1 or abs(z)>.1:
        raise ValueError('wheel axis is not authenticated as transverse')
    return 1. if y>0 else -1.


def forward_equivalent(names, values, signs):
    """Reorder by joint NAME, then multiply authenticated forward signs."""
    if len(names)!=len(set(names)) or len(names)!=len(values): return None
    by_name=dict(zip(names,values))
    try:
        result=[float(by_name[w])*float(signs[w]) for w in WHEELS]
    except (KeyError,TypeError,ValueError): return None
    return result if all(math.isfinite(v) for v in result) else None


def _moving(rows):
    return [r for r in rows if r.get('u') is not None and r.get('v') is not None
            and math.isfinite(r['u']) and math.isfinite(r['v']) and r['u']>0]


def repeatability(rows):
    result=[]
    for u in sorted({r['u'] for r in _moving(rows)}):
        group=[r for r in _moving(rows) if r['u']==u]
        by_run={run:mean(r['v'] for r in group if r['run_id']==run) for run in {r['run_id'] for r in group}}
        values=list(by_run.values())
        result.append({'u':u,'n_runs':len(values),'n_observations':len(group),
            'mean':mean(values),'std':stdev(values) if len(values)>1 else None,
            'min':min(values),'max':max(values),'range':max(values)-min(values),
            'std_reason':None if len(values)>1 else 'one independent run; std not identifiable'})
    return result


def fit_model(kind, rows):
    rows=_moving(rows); unique=sorted({r['u'] for r in rows})
    model={'kind':kind,'valid':False,'reason':None,'parameters':{},'knots':[],
        'zero_state':{'input':0.,'output':0.,'source':'explicit physical state; not fitted or joined to moving branch'},
        'input_domain':{'zero_state':[0.,0.],'moving':[min(unique),max(unique)] if unique else None},
        'output_domain':{'zero_state':[0.,0.],'moving':None},'extrapolation':'forbidden',
        'monotonic':None,'independent_runs':len({r['run_id'] for r in rows}),'distinct_speeds':len(unique)}
    if not rows or (kind in ('M2','M3') and len(unique)<2):
        model['reason']='insufficient distinct nonzero speeds';return model
    xs=[r['u'] for r in rows];ys=[r['v'] for r in rows]
    if kind=='M0':model['parameters']={'k':1.};slope=1.
    elif kind=='M1':
        slope=sum(x*y for x,y in zip(xs,ys))/sum(x*x for x in xs);model['parameters']={'k':slope}
    elif kind=='M2':
        mx,my=mean(xs),mean(ys)
        slope=sum((x-mx)*(y-my) for x,y in zip(xs,ys))/sum((x-mx)**2 for x in xs)
        model['parameters']={'a':slope,'b':my-slope*mx}
    elif kind=='M3':
        model['knots']=[[r['u'],r['mean']] for r in repeatability(rows)]
        slope=min(b[1]-a[1] for a,b in zip(model['knots'],model['knots'][1:]))
    else:raise ValueError(f'unknown model: {kind}')
    model['monotonic']=slope>=0
    model['valid']=True
    if kind=='M3' and not model['monotonic']:
        model.update(valid=False,reason='measured knot means are nonmonotonic; no isotonic smoothing applied');return model
    values=[predict(model,u) for u in unique]
    model['predicted_output_range']=[min(values),max(values)]
    model['measured_output_range']=[min(ys),max(ys)]
    lo,hi=max(min(values),min(ys)),min(max(values),max(ys))
    model['output_domain']['moving']=[lo,hi] if lo<=hi else None
    model['output_domain_note']='inverse supported only in intersection of fitted and measured output ranges'
    return model


def _interpolate(knots, value):
    if not knots or not knots[0][0]<=value<=knots[-1][0]: return None
    if value==knots[-1][0]:return knots[-1][1]
    i=bisect.bisect_right([p[0] for p in knots],value)-1
    a,b=knots[i:i+2]
    return a[1]+(value-a[0])*(b[1]-a[1])/(b[0]-a[0])


def predict(model, u):
    if not math.isfinite(u):return None
    if u==0.:return 0.
    domain=model['input_domain']['moving']
    if not model['valid'] or domain is None or not domain[0]<=u<=domain[1]:return None
    if model['kind']=='M3':return _interpolate(model['knots'],u)
    p=model['parameters']
    return p['k']*u if 'k' in p else p['a']*u+p['b']


def inverse(model, v):
    """Diagnostic only; no zero-to-first-knot bridge and no extrapolation."""
    if not math.isfinite(v):return None
    if v==0.:return 0.
    domain=model['output_domain']['moving']
    if not model['valid'] or not model['monotonic'] or domain is None or not domain[0]<=v<=domain[1]:return None
    if model['kind']=='M3':
        knots=model['knots']
        if any(b[1]<=a[1] for a,b in zip(knots,knots[1:])):return None
        u=_interpolate([[y,x] for x,y in knots],v)
    else:
        p=model['parameters'];slope=p.get('k',p.get('a'))
        if slope<=0:return None
        u=(v-p.get('b',0.))/slope
    lo,hi=model['input_domain']['moving']
    return min(hi,max(lo,u)) if lo-1e-12<=u<=hi+1e-12 else None # floating roundoff only


def errors(residuals, ys):
    if not residuals:return {'n':0,'mae':None,'rmse':None,'max_abs_error':None,'r2':None}
    sse=sum(r*r for r in residuals);den=sum((y-mean(ys))**2 for y in ys)
    return {'n':len(residuals),'mae':mean(abs(r) for r in residuals),
        'rmse':math.sqrt(sse/len(residuals)),'max_abs_error':max(abs(r) for r in residuals),
        'r2':1-sse/den if len(ys)>1 and den>1e-15 else None}


def compare_models(rows):
    rows=_moving(rows); result={'independent_runs':len({r['run_id'] for r in rows}),
        'distinct_speeds':len({r['u'] for r in rows}),'repeatability':repeatability(rows),
        'observed_monotonic':None,'models':{},'production_selection':None,
        'selection_reason':'fresh independent user-run validation required; in-sample R² does not select production model',
        'weighting':'one run-window per observation; knots and repeat statistics use per-run speed means'}
    knots=result['repeatability']
    if len(knots)>1:result['observed_monotonic']=all(b['mean']>=a['mean'] for a,b in zip(knots,knots[1:]))
    for kind in ('M0','M1','M2','M3'):
        m=fit_model(kind,rows);valid=[r for r in rows if predict(m,r['u']) is not None]
        residuals=[r['v']-predict(m,r['u']) for r in valid] # observed minus prediction
        m['metrics']=errors(residuals,[r['v'] for r in valid]);m['residual_sign']='observed minus predicted'
        m['residuals_by_speed']=[{'u':u,'signed_residuals':[e for r,e in zip(valid,residuals) if r['u']==u],
            'mean_signed_residual':mean(e for r,e in zip(valid,residuals) if r['u']==u)} for u in sorted({r['u'] for r in valid})]
        loso=[];unsupported=[]
        for u in sorted({r['u'] for r in rows}):
            held=[r for r in rows if r['u']==u];f=fit_model(kind,[r for r in rows if r['u']!=u])
            pred=predict(f,u)
            for r in held:
                item={'run_id':r['run_id'],'u':u,'observed':r['v'],'predicted':pred,'residual':r['v']-pred if pred is not None else None}
                (loso if pred is not None else unsupported).append(item)
        m['loso']={**errors([r['residual'] for r in loso],[r['observed'] for r in loso]),
            'predicted_count':len(loso),'unsupported_count':len(unsupported),'predictions':loso,'unsupported':unsupported,
            'reason':'held endpoints outside training support remain null; no extrapolation'}
        m['diagnostic_inverse']=[{'desired_v':v,'policy_u':inverse(m,v)} for v in [0.]+[r['mean'] for r in knots]]
        result['models'][kind]=m
    return result
