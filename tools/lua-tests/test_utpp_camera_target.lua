-- Real camera module/math with a mocked native sink; no gameplay or camera writes.
UTPP_CAMERA_MATH_PATH = UTPP_CAMERA_MATH_PATH or "Source/plugin/UTPP/cet/utpp_camera/camera_math.lua"
local calls,oldAimCalls,logs={},0,{}
Vector4={new=function(x,y,z,w)return{x=x,y=y,z=z,w=w}end}
Game={UTPPSetCameraTarget=function(p,d,on)calls[#calls+1]={p=p,d=d,on=on}end,
 UTPPSetAimTarget=function()oldAimCalls=oldAimCalls+1 end,
 UTPPSetPoseWriteTpp=function()return true end}
local M=dofile(UTPP_CAMERA_PATH or "Source/plugin/UTPP/cet/utpp_camera/init.lua")
M.Log=function(x)logs[#logs+1]=x end
local function near(a,b)return math.abs(a-b)<1e-8 end
local res={world={x=1.25,y=-3.5,z=1.8},yaw=37,pitch=-12,fov=83}
local oldFov=res.fov
local beforeAim=M.Aim and M.Aim() or nil
assert(M.FeedCameraTarget(res),"hipfire heartbeat accepted")
local expected=M.CM.forward(res.yaw,res.pitch)
assert(#calls==1 and calls[1].on==true and calls[1].p.w==1 and calls[1].d.w==0)
assert(near(calls[1].p.x,res.world.x) and near(calls[1].d.x,expected.x) and near(calls[1].d.y,expected.y) and near(calls[1].d.z,expected.z),"solved camera ray exact")
-- ADS does not change this ray's route or call the legacy ADS-only native.
M.aimTargetSent=true
assert(M.FeedCameraTarget(res) and #calls==2 and calls[2].on,"ADS also refreshes separate camera target")
assert(oldAimCalls==0 and M.aimTargetSent==true and res.fov==oldFov,"no legacy ADS or FOV mutation")
M.tppSender=function()return true,"mock"end
M.SignalTpp(false,"FPP",0)
assert(#calls==3 and calls[3].on==false,"FPP central veto disables camera target")
M.SignalTpp(false,"still FPP",1)
assert(#calls==3,"disable only once even repeated veto")
assert(M.FeedCameraTarget(res) and #calls==4,"TPP resumes heartbeat")
M.RequestFppHandoff(true,"test")
assert(calls[#calls].on==false,"explicit handoff disables ray")
Game.UTPPSetCameraTarget=nil
assert(not M.FeedCameraTarget(res),"older DLL is graceful")
assert(not M.FeedCameraTarget(res) and #logs>=1)
local warnings=0;for _,x in ipairs(logs)do if string.find(x,"UTPPSetCameraTarget unavailable",1,true)then warnings=warnings+1 end end
assert(warnings==1,"missing native warns once")
print("PASS camera target hipfire/ADS ray, FPP/handback disable, no FOV/legacy mutation, old DLL")
