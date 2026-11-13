-- UTPP test command channel: CET helper for automated test runs (test installs only, never part of a release).
--
-- About 20 times per second the module reads cmd.txt in this mod folder. Each line is "<sequence> <lua chunk>". A line whose
-- sequence is greater than the last executed one runs once (compiled with loadstring / load, executed in pcall), then one line
-- "[testcmd] ACK seq=<n> ok=<true|false> <result or error>" is written to the UTPP log through Game.UTPPLogLine.
-- Lines already in cmd.txt when the module loads are never executed (a command left over from an earlier session cannot run
-- at the next start). The last sequence is kept in state.txt, so it survives "Reload all mods".
-- On load: "[testcmd] READY seq=<last> loader=<loadstring|load|none>" (once at init and once on the first update).
-- No keyboard, no overlay window.

local M = {
  CMD_FILE = "cmd.txt",
  STATE_FILE = "state.txt",
  PERIOD = 0.05,          -- seconds between reads of cmd.txt
  RESULT_MAX = 300,       -- characters of the result / error text in the ACK line
  last = 0,
  acc = 0.0,
  updates = 0,
  loader = nil,
  loaderName = "none",
}

local function fmtSeq(n)
  return string.format("%.0f", n)
end

local function logLine(msg)
  local ok = pcall(function() Game.UTPPLogLine(msg) end)
  if not ok then print(msg) end
  return ok
end

local function readAll(path)
  local f = io.open(path, "r")
  if not f then return nil end
  local txt = f:read("*a")
  f:close()
  return txt
end

-- {{seq, chunk}, ...} in file order; malformed lines are ignored
function M.ParseLines(txt)
  local out = {}
  if type(txt) ~= "string" then return out end
  for line in txt:gmatch("[^\r\n]+") do
    local s, chunk = line:match("^%s*(%d+)%s+(.-)%s*$")
    if s and chunk and chunk ~= "" then
      out[#out + 1] = { tonumber(s), chunk }
    end
  end
  return out
end

local function maxSeq(entries)
  local m = 0
  for _, e in ipairs(entries) do
    if e[1] > m then m = e[1] end
  end
  return m
end

local function readState()
  local txt = readAll(M.STATE_FILE)
  local s = txt and txt:match("(%d+)")
  return s and tonumber(s) or 0
end

local function saveState()
  local f = io.open(M.STATE_FILE, "w")
  if f then
    f:write(fmtSeq(M.last), "\n")
    f:close()
  end
end

local function oneLine(v)
  local s = tostring(v)
  s = s:gsub("[%c]", " ")
  if #s > M.RESULT_MAX then s = s:sub(1, M.RESULT_MAX) end
  return s
end

local function compile(chunk)
  if M.loader == nil then return nil, "no Lua loader available" end
  local fn = M.loader("return " .. chunk, "testcmd")   -- expressions report their value
  if fn then return fn end
  return M.loader(chunk, "testcmd")
end

function M.Execute(seq, chunk)
  local fn, err = compile(chunk)
  local ok, res
  if not fn then
    ok, res = false, err
  else
    local r = { pcall(fn) }
    ok, res = r[1], r[2]
    if ok and res == nil then res = "" end
  end
  logLine("[testcmd] ACK seq=" .. fmtSeq(seq) .. " ok=" .. tostring(ok) .. " " .. oneLine(res))
  return ok, res
end

function M.Poll()
  local entries = M.ParseLines(readAll(M.CMD_FILE))
  for _, e in ipairs(entries) do
    local seq, chunk = e[1], e[2]
    if seq > M.last then
      M.last = seq
      saveState()           -- before running: a chunk that ends the session never runs twice
      M.Execute(seq, chunk)
    end
  end
end

function M.Ready()
  return logLine("[testcmd] READY seq=" .. fmtSeq(M.last) .. " loader=" .. M.loaderName)
end

function M.Init()
  if loadstring ~= nil then
    M.loader, M.loaderName = loadstring, "loadstring"
  elseif load ~= nil then
    M.loader, M.loaderName = load, "load"
  end
  local fromFile = maxSeq(M.ParseLines(readAll(M.CMD_FILE)))
  M.last = math.max(readState(), fromFile)
  saveState()
  M.Ready()
end

function M.Update(dt)
  M.updates = M.updates + 1
  if M.updates == 1 then M.Ready() end
  M.acc = M.acc + (tonumber(dt) or 0)
  if M.acc < M.PERIOD then return end
  M.acc = 0.0
  local ok, err = pcall(M.Poll)
  if not ok then
    M.pollErrors = (M.pollErrors or 0) + 1
    if M.pollErrors <= 5 then logLine("[testcmd] poll error " .. oneLine(err)) end
  end
end

registerForEvent("onInit", function() M.Init() end)
registerForEvent("onUpdate", function(dt) M.Update(dt) end)

return M
