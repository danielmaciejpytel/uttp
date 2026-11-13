-- Local verified-bundle test opt-in. Package only with its matching assets.
-- Does not change the gate's default OFF or issue gameplay commands.
local M={attempted=false,activated=false,lastError=nil}
registerForEvent('onUpdate',function()
  if M.attempted then return end
  local ok,gate=pcall(GetMod,'utpp_reload_gate')
  if not ok or not gate or gate.ready~=true then return end
  M.attempted=true
  local enabled,result=pcall(function()
    gate.SetBundleReady(true)
    gate.SetNormalBundleReady(true)
    gate.SetTwoHandBundleReady(true)
    return gate.SetEnabled(true)
  end)
  M.activated=enabled and result==true
  if M.activated then
    print('[UTPP local test profile] Verified EMPTY/NORMAL 1H/2H reload bundle enabled.')
  else
    M.lastError=enabled and 'gate rejected activation' or tostring(result)
    print('[UTPP local test profile] Activation failed: '..M.lastError)
  end
end)
return M
