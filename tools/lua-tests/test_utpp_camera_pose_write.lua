-- Targeted strict TPP write permission contract; no simulated gameplay suite.
UTPP_CAMERA_MATH_PATH = UTPP_CAMERA_DIR .. "/camera_math.lua"
local calls = {}
Game = { UTPPSetPoseWriteTpp = function(on) calls[#calls + 1] = on; return true end }
local M = dofile(UTPP_CAMERA_PATH)
assert(#calls == 0, "load must not grant permission")
assert(M.PoseWriteTpp(true) and calls[1] == true)
M.tppSender = function() return true, "stub" end
M.SignalTpp(false, "fpp", 0)
assert(calls[#calls] == false)
local count = #calls
M.SignalTpp(false, "already false", 1)
assert(#calls == count + 1 and calls[#calls] == false, "cached script signal must not suppress native veto")
M.RequestFppHandoff(true, "zoom")
assert(calls[#calls] == false)
local T = {state = "tpp", want = true}
assert(M.PoseWriteTppWanted(T, false, true, {}, false, {}))
for _, command in ipairs({"fpp_on", "cam_off", "rep_off"}) do
  local allowed = M.PoseWriteTppWanted(T, false, true, {}, false, {command})
  assert(not allowed, "command veto must win even while state remains tpp: " .. command)
  M.PoseWriteTpp(allowed and true) -- simulate a successful camera write after execute
  assert(calls[#calls] == false, "successful write must not reopen a command-vetoed frame")
end
assert(not M.PoseWriteTppWanted(T, false, true, nil, false, {}), "missing pose veto")
assert(not M.PoseWriteTppWanted(nil, false, true, {}, false, {}), "missing transition veto")
assert(not M.PoseWriteTppWanted(T, true, true, {}, false, {}), "menu/photo veto")
assert(not M.PoseWriteTppWanted(T, false, true, {}, true, {}), "vehicle hold veto")
-- exit: after the unmount the hold must not veto the MM write (the live vanilla exit pose = squat, hands at the face)
assert(not M.PoseWriteTppWanted(T, false, true, {}, true, {}, false), "mounted hold still vetoes")
assert(M.PoseWriteTppWanted(T, false, true, {}, true, {}, true), "unmounted hold does not veto")
assert(M.PoseWriteTppWanted(T, false, true, {}, false, {}, true), "onFoot without a hold: unchanged")
assert(not M.PoseWriteTppWanted(T, false, true, {}, true, {"fpp_on"}, true), "command veto still wins on foot")
assert(not M.PoseWriteTppWanted(T, false, true, nil, true, {}, true), "missing pose veto still wins on foot")
assert(not M.PoseWriteTppWanted(T, true, true, {}, true, {}, true), "menu/photo veto still wins on foot")
assert(not M.PoseWriteTppWanted(T, false, false, {}, true, {}, true), "camera unavailable veto still wins on foot")
assert(not M.PoseWriteTppWanted({state="leaving",want=false}, false, true, {}, true, {}, true), "state veto still wins on foot")
-- diagnostic: which clause vetoed
assert(M.PoseWriteDeniedReason(T, false, true, {}, false, true) == nil and M.PoseWriteDeniedReason(T, false, true, {}, true, true) == nil)
assert(M.PoseWriteDeniedReason(T, false, true, {}, true, false) == "held(mounted)")
assert(M.PoseWriteDeniedReason(T, true, true, {}, true, true) == "freeze")
assert(M.PoseWriteDeniedReason(T, false, false, {}, false) == "camNotOK")
assert(M.PoseWriteDeniedReason(T, false, true, nil, false) == "res=nil")
assert(M.PoseWriteDeniedReason({state="fpp",want=true}, false, true, {}, false) == "state=fpp")
assert(M.PoseWriteDeniedReason({state="tpp",want=false}, false, true, {}, false) == "want=false")
assert(M.PoseWriteDeniedReason(nil, false, true, {}, false) == "noT")
-- VehicleOnFoot API: only while held; cleared by a new hold / the release
assert(M.vh.onFoot == false)
local okF, whyF = M.VehicleOnFoot(true, "test")
assert(okF == true and whyF == "not held" and M.vh.onFoot == false, "VehicleOnFoot without a hold is a no-op")
M.vh.held = true
assert(M.VehicleOnFoot(true, "test") and M.vh.onFoot == true, "VehicleOnFoot sets the flag while held")
assert(M.HoldForVehicle(true, "remount") and M.vh.onFoot == false, "a new mount clears the flag (hold still running)")
assert(M.VehicleOnFoot(true, "test") and M.vh.onFoot == true)
assert(M.HoldForVehicle(false, "test") and M.vh.held == false and M.vh.onFoot == false, "release clears the flag")
assert(not M.PoseWriteTppWanted(T, false, false, {}, false, {}), "camera unavailable veto")
assert(not M.PoseWriteTppWanted({state="leaving",want=false}, false, true, {}, false, {}))
Game.UTPPSetPoseWriteTpp = function() error("native unavailable") end
assert(not M.PoseWriteTpp(true), "missing native fails closed")
print("strict TPP camera targeted stub PASS")
