"""Focused Blender mathutils tests for ankle-offset yaw, no assets or writes."""
import sys,math
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent));import retarget_batch as r
from mathutils import Quaternion
passed=0
def check(q,expected):
 global passed
 d=(r.ankle_offset_yaw(q)-expected+math.pi)%(2*math.pi)-math.pi
 assert abs(d)<2e-6,(d,q);passed+=1
for deg in (-179,-90,0,45,179):check(Quaternion((0,0,1),math.radians(deg)),math.radians(deg))
for deg in (80,89,90,91,100):check(Quaternion((0,0,1),.4)@Quaternion((1,0,0),math.radians(deg)),.4)
check(Quaternion((1,0,0),math.pi),0)
check(Quaternion((0,0,0,0)),0)
q=Quaternion((0,0,1),.7);check(-q,.7)
print('ANKLE_OFFSET_YAW_PASS',passed)
