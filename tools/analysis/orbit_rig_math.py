"""Independent OrbitRig constraint math; consumes graph-stage model transforms.

History-source equations are evidence, not a claim of current retail execution.
No graph parser, history integration, capture reads or mutations live here.
"""
import numpy as np

def vector(v):return np.array([v[k] for k in 'XYZ'],dtype=float)
def transform_name(v):
    value=v['name'];return value['$value'] if isinstance(value,dict) else value
def normalized(v):
    length=np.linalg.norm(v)
    if not np.isfinite(length) or length<1e-8:raise ValueError('Degenerate OrbitRig aim/up')
    return v/length
def frame(forward,up):
    f=normalized(forward);u=normalized(up-f*np.dot(f,up));return np.column_stack((f,u,np.cross(f,u)))
def object_up_rotation(origin,target,up_position,local_forward,local_up):
    # Weight1 ObjectUp: equivalent to historical two shortest rotations after
    # cached local Gram-Schmidt and world aim/up Gram-Schmidt (source490-540).
    return frame(target-origin,up_position-origin)@frame(local_forward,local_up).T
def bounce_value(input_x,node):
    multiplier=node['negativeMultiplier'] if input_x<0 else node['multiplier']
    value=abs(input_x)-node['delay'];out=0.0
    if value>0:
        smooth=node['smoothStep']
        out=(value*value/smooth if smooth!=0 and abs(value)<smooth else value)*multiplier
    return out+node['offset']
def weighted_position(ms,node):
    return sum(weight*ms[transform_name(entry['transform'])][:3,3]
               for weight,entry in zip(node['preprocessedWeights'],node['inputWeightedTransforms']))
def backpoint(ms,nodes,side):
    """Compute OrbitMS and back-muscle point from pre-Orbit stage inputs.

    Both sides use graph data (including actual sign/reference offsets), not
    guessed symmetry. Returned matrices do not mutate the provided model pose.
    Caller must provide stage-correct logical midpoint transforms.
    """
    if side not in ('l','r'):raise ValueError('side')
    aim,average,up,setter,bounce_y,bounce_z=(363,364,365,330,329,328) if side=='l' else (350,351,352,320,319,318)
    def node(h):return nodes[str(h)] if str(h) in nodes else nodes[h]
    an=node(aim);orbit=ms[transform_name(an['transformIndex'])].copy()
    target=weighted_position(ms,node(average));up_position=weighted_position(ms,node(up))
    orbit[:3,:3]=object_up_rotation(orbit[:3,3],target,up_position,vector(an['forwardAxisLS']),vector(an['upAxisLS']))
    entry=node(setter)['entries'][0];offset=entry['offset'];point=vector(offset['Translation'])
    # For this branch both multipliers are exactly zero on both bounce nodes,
    # so driver input cannot affect Y/Z. Reject graph drift instead of silently
    # treating a future nonzero multiplier as the same constant expression.
    for handle in (bounce_y,bounce_z):
        bn=node(handle)
        if bn['multiplier']!=0 or bn['negativeMultiplier']!=0:raise ValueError('Orbit constant branch changed')
        for output in bn['transformOutputs']:
            for channel in output['channelEntries']:
                point['XYZ'.index(channel['transformChannel'][-1])]=channel['multiplier']*bounce_value(0,bn)
    world=orbit[:3,3]+orbit[:3,:3]@point
    return dict(orbit=orbit,point=world,relative_point=point,target=target,up=up_position,
                handles=dict(aim=aim,target=average,up=up,setter=setter,bounce_y=bounce_y,bounce_z=bounce_z))
