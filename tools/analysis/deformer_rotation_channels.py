"""Standard column-vector Rz(yaw) Rx(pitch) Ry(roll) channel replacement.

The channel convention and the degree units are confirmed by comparison with poses captured from the game.
This mathematical adapter does not establish current retail evaluator equivalence.
"""
import numpy as np

def replace_rotation_channels(local, channels):
    out=local.copy()
    scale=np.linalg.norm(out[:3,:3],axis=0)
    if min(scale)<1e-9:raise ValueError('Singular rotation channel input')
    r=out[:3,:3]/scale
    if np.max(np.abs(r.T@r-np.eye(3)))>1e-5 or np.linalg.det(r)<.99999:
        raise ValueError('Nonrigid rotation channel input')
    pitch=np.arcsin(np.clip(r[2,1],-1,1))
    if abs(np.cos(pitch))<1e-7:raise ValueError('Unvalidated Euler gimbal boundary')
    yaw=np.arctan2(-r[0,1],r[1,1]);roll=np.arctan2(-r[2,0],r[2,2])
    values={'RotX':pitch,'RotY':roll,'RotZ':yaw}
    for channel,value in channels.items():
        if channel not in values:raise ValueError('Unsupported rotation channel')
        values[channel]=np.deg2rad(value)
    p,b,y=values['RotX'],values['RotY'],values['RotZ']
    cp,sp,cb,sb,cy,sy=np.cos(p),np.sin(p),np.cos(b),np.sin(b),np.cos(y),np.sin(y)
    rx=np.array([[1,0,0],[0,cp,-sp],[0,sp,cp]])
    ry=np.array([[cb,0,sb],[0,1,0],[-sb,0,cb]])
    rz=np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]])
    out[:3,:3]=(rz@rx@ry)*scale
    return out
