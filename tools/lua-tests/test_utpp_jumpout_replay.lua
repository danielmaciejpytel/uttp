-- UTPP utpp_camera: jump-out pivot replay (2026-11-04 r13). The two jump-outs of the r6a recordings (game log utpp-2026-11-04-12-57-36.log, camTrace
-- lines at 10 Hz: pivotZ(+offset) = V's height, speed = V's horizontal speed, jo=ground/brake/recovery/base speed) are replayed through CM.pivotStep:
--   * stand: car at 1.7 m/s, launch 13:00:17.795, V pushed ~1.7 m sideways (direction from the [night] POS lines) right after a false ground contact at
--     13:00:19.144 (speed 0.9 -> 6.6 m/s within 0.1 s), landing 19.243;
--   * move: car at 8 m/s, launch 13:02:20.688, V keeps 3.8 m/s and stops dead in about 0.1 s at the touchdown (22.2).
-- V's height is a cubic Hermite curve through the logged samples; her xy is the logged speed integrated along the logged direction. The 10 Hz samples
-- make the push look slower than the video (full speed within 1 - 2 frames): the "sharp" variants squeeze the same speed change into 2 frames.
-- Measured: the pivot's acceleration / jerk (finite differences over the real frame times), lag to V, the offset to V at the end of the jump-out mode
-- (continuity), the vertical brake (no undershoot below V's stance). The cfg switch jumpOutXY = false is the old behaviour (pivot xy = V's xy): the
-- numbers of the old code are asserted too (the checks fail on it).
-- Globals from the runner: UTPP_CAMERA_MATH_PATH.
local results = { pass = 0, fail = 0 }
local function check(name, ok, detail)
  if ok then results.pass = results.pass + 1 else results.fail = results.fail + 1 end
  print(string.format("[%s] %s%s", ok and "PASS" or "FAIL", name, (not ok and detail ~= nil) and (" -- " .. tostring(detail)) or ""))
end
local CM = dofile(UTPP_CAMERA_MATH_PATH)
local V = CM.v3
local VERBOSE = false

-- ---------------------------------------------------------------- fixtures (log seconds, metres)
local STAND = {
  t0 = 17.775, tOff = 19.425, final = 182.03, dir = { 0.793, 0.609 },
  z = { { 17.775, 183.87 }, { 17.875, 184.93 }, { 17.975, 185.68 }, { 18.075, 186.28 }, { 18.175, 186.71 }, { 18.276, 186.99 }, { 18.375, 187.10 },
        { 18.476, 187.06 }, { 18.575, 186.84 }, { 18.675, 186.48 }, { 18.775, 185.96 }, { 18.875, 185.27 }, { 18.975, 184.43 }, { 19.075, 183.42 },
        { 19.175, 182.43 }, { 19.275, 182.04 }, { 19.375, 182.03 }, { 20.5, 182.03 } },
  speed = { { 17.775, 2.57 }, { 17.875, 1.26 }, { 17.975, 1.22 }, { 18.075, 1.20 }, { 18.175, 1.16 }, { 18.276, 1.13 }, { 18.375, 1.11 },
            { 18.476, 1.07 }, { 18.575, 1.04 }, { 18.675, 1.02 }, { 18.775, 0.98 }, { 18.875, 0.95 }, { 18.975, 0.92 }, { 19.075, 0.89 },
            { 19.175, 6.63 }, { 19.275, 5.49 }, { 19.375, 3.22 }, { 19.475, 0.88 }, { 19.575, 0 }, { 20.5, 0 } },
  -- the same push in 2 frames: 0.9 -> 6.6 m/s at 19.115 - 19.148 (the false contact), then the logged decay
  sharp = { { 17.775, 2.57 }, { 17.875, 1.26 }, { 18.975, 0.92 }, { 19.115, 0.89 }, { 19.148, 6.63 }, { 19.275, 5.49 }, { 19.375, 3.22 }, { 19.475, 0.88 },
            { 19.575, 0 }, { 20.5, 0 } },
}
local MOVE = {
  t0 = 20.483, tOff = 22.40, final = 180.07, dir = { 0.43, -0.90 },
  z = { { 20.483, 182.65 }, { 20.583, 182.62 }, { 20.683, 182.88 }, { 20.783, 183.74 }, { 20.881, 184.44 }, { 20.981, 185.00 }, { 21.081, 185.40 },
        { 21.180, 185.64 }, { 21.281, 185.74 }, { 21.380, 185.65 }, { 21.481, 185.42 }, { 21.581, 185.02 }, { 21.681, 184.46 }, { 21.780, 183.75 },
        { 21.881, 182.88 }, { 21.980, 181.83 }, { 22.081, 180.66 }, { 22.180, 180.08 }, { 22.281, 180.07 }, { 23.0, 180.07 } },
  speed = { { 20.483, 7.35 }, { 20.583, 7.19 }, { 20.683, 4.17 }, { 20.783, 4.15 }, { 20.881, 4.11 }, { 20.981, 4.08 }, { 21.081, 4.05 }, { 21.180, 4.02 },
            { 21.281, 4.00 }, { 21.380, 3.96 }, { 21.481, 3.93 }, { 21.581, 3.90 }, { 21.681, 3.88 }, { 21.780, 3.85 }, { 21.881, 3.82 },
            { 21.980, 3.78 }, { 22.081, 3.75 }, { 22.180, 2.78 }, { 22.281, 0 }, { 23.0, 0 } },
  -- the stop as the position log has it (V stands still from 22.215): 3.75 m/s at 22.10 -> 0 at 22.20
  sharp = { { 20.483, 7.35 }, { 20.583, 7.19 }, { 20.683, 4.17 }, { 22.10, 3.75 }, { 22.16, 2.78 }, { 22.20, 0 }, { 23.0, 0 } },
}

-- 2026-11-04 r13a recordings (game build r13a, jo diag lines at 20 Hz of the jump-out harness: V's position in the vehicle frame, the car speed,
-- camTrace pivotZ at 10 Hz). px / py / z = V's world position (x lateral, y along the car; the car origin integrated from the logged car speed; z = the
-- logged vehicle frame height shifted to the world ground), cam = the logged (t, pivotZ, V's horizontal speed). Cubic Hermite through the samples.
local R13A_STAND = {
  t0 = 132.832, tOff = 134.458, final = 182.03,
  px = {
    { 132.532, -1.160 }, { 132.682, -1.160 }, { 132.832, -1.160 }, { 132.882, -1.160 }, { 132.932, -1.160 }, { 132.982, -1.150 },
    { 133.048, -1.150 }, { 133.115, -1.150 }, { 133.165, -1.150 }, { 133.215, -1.150 }, { 133.265, -1.150 }, { 133.332, -1.160 },
    { 133.398, -1.160 }, { 133.465, -1.150 }, { 133.532, -1.150 }, { 133.582, -1.150 }, { 133.642, -1.150 }, { 133.692, -1.150 },
    { 133.758, -1.150 }, { 133.808, -1.150 }, { 133.858, -1.150 }, { 133.908, -1.150 }, { 133.975, -1.150 }, { 134.042, -1.150 },
    { 134.092, -1.150 }, { 134.158, -1.240 }, { 134.208, -1.560 }, { 134.258, -1.910 }, { 134.308, -2.200 }, { 134.375, -2.540 },
    { 134.425, -2.720 }, { 134.492, -2.870 }, { 134.558, -2.920 }, { 134.625, -2.920 }, { 134.675, -2.920 }, { 134.725, -2.920 },
    { 134.775, -2.920 }, { 134.842, -2.920 }, { 134.922, -2.920 }, { 134.972, -2.920 }, { 135.022, -2.920 }, { 135.072, -2.920 },
    { 135.122, -2.920 }, { 135.189, -2.920 }, { 135.255, -2.920 }, { 135.305, -2.920 }, { 135.355, -2.920 }, { 135.405, -2.920 },
    { 135.455, -2.920 }, { 135.505, -2.920 }, { 135.555, -2.920 }, { 135.605, -2.920 }, { 135.655, -2.920 }, { 135.705, -2.920 },
    { 135.755, -2.920 }, { 135.805, -2.920 }, { 135.855, -2.920 },
  },
  py = {
    { 132.532, 0.309 }, { 132.682, 0.274 }, { 132.832, 0.240 }, { 132.882, 0.229 }, { 132.932, 0.184 }, { 132.982, 0.127 }, { 133.048, 0.055 },
    { 133.115, -0.019 }, { 133.165, -0.076 }, { 133.215, -0.123 }, { 133.265, -0.180 }, { 133.332, -0.246 }, { 133.398, -0.314 },
    { 133.465, -0.383 }, { 133.532, -0.451 }, { 133.582, -0.499 }, { 133.642, -0.557 }, { 133.692, -0.607 }, { 133.758, -0.666 },
    { 133.808, -0.715 }, { 133.858, -0.764 }, { 133.908, -0.803 }, { 133.975, -0.862 }, { 134.042, -0.922 }, { 134.092, -0.962 },
    { 134.158, -1.021 }, { 134.208, -1.061 }, { 134.258, -1.111 }, { 134.308, -1.151 }, { 134.375, -1.200 }, { 134.425, -1.220 },
    { 134.492, -1.239 }, { 134.558, -1.249 }, { 134.625, -1.249 }, { 134.675, -1.249 }, { 134.725, -1.249 }, { 134.775, -1.249 },
    { 134.842, -1.248 }, { 134.922, -1.248 }, { 134.972, -1.248 }, { 135.022, -1.248 }, { 135.072, -1.248 }, { 135.122, -1.247 },
    { 135.189, -1.247 }, { 135.255, -1.247 }, { 135.305, -1.247 }, { 135.355, -1.247 }, { 135.405, -1.247 }, { 135.455, -1.247 },
    { 135.505, -1.247 }, { 135.555, -1.247 }, { 135.605, -1.247 }, { 135.655, -1.247 }, { 135.705, -1.247 }, { 135.755, -1.247 },
    { 135.805, -1.247 }, { 135.855, -1.247 },
  },
  z = {
    { 132.532, 184.350 }, { 132.682, 184.350 }, { 132.832, 184.350 }, { 132.882, 184.800 }, { 132.932, 185.220 }, { 132.982, 185.590 },
    { 133.048, 186.010 }, { 133.115, 186.370 }, { 133.165, 186.600 }, { 133.215, 186.770 }, { 133.265, 186.930 }, { 133.332, 187.050 },
    { 133.398, 187.110 }, { 133.465, 187.090 }, { 133.532, 187.010 }, { 133.582, 186.900 }, { 133.642, 186.700 }, { 133.692, 186.510 },
    { 133.758, 186.180 }, { 133.808, 185.890 }, { 133.858, 185.560 }, { 133.908, 185.190 }, { 133.975, 184.630 }, { 134.042, 183.980 },
    { 134.092, 183.490 }, { 134.158, 182.970 }, { 134.208, 182.560 }, { 134.258, 182.070 }, { 134.308, 182.030 }, { 134.375, 182.030 },
    { 134.425, 182.030 }, { 134.492, 182.030 }, { 134.558, 182.030 }, { 134.625, 182.030 }, { 134.675, 182.030 }, { 134.725, 182.030 },
    { 134.775, 182.030 }, { 134.842, 182.030 }, { 134.922, 182.030 }, { 134.972, 182.030 }, { 135.022, 182.030 }, { 135.072, 182.030 },
    { 135.122, 182.030 }, { 135.189, 182.030 }, { 135.255, 182.030 }, { 135.305, 182.030 }, { 135.355, 182.030 }, { 135.405, 182.030 },
    { 135.455, 182.030 }, { 135.505, 182.030 }, { 135.555, 182.030 }, { 135.605, 182.030 }, { 135.655, 182.030 }, { 135.705, 182.030 },
    { 135.755, 182.030 }, { 135.805, 182.030 }, { 135.855, 182.030 },
  },
  -- the logged camTrace samples (t, pivotZ, V horizontal speed)
  cam = {
    { 132.87, 186.01, 1.22 }, { 132.97, 187.11, 1.18 }, { 133.07, 187.80, 1.15 }, { 133.17, 188.24, 1.12 }, { 133.27, 188.56, 1.09 },
    { 133.36, 188.70, 1.06 }, { 133.47, 188.68, 1.03 }, { 133.56, 188.50, 1.00 }, { 133.67, 188.10, 0.99 }, { 133.77, 187.60, 0.96 },
    { 133.87, 186.91, 0.92 }, { 133.97, 186.07, 0.89 }, { 134.07, 185.11, 0.86 }, { 134.17, 184.31, 6.52 }, { 134.27, 183.80, 6.34 },
    { 134.37, 183.60, 4.38 }, { 134.47, 183.60, 2.08 }, { 134.57, 183.58, 0.00 }, { 134.67, 183.58, 0.00 }, { 134.77, 183.58, 0.00 },
    { 134.87, 183.58, 0.00 }, { 134.97, 183.58, 0.00 }, { 135.07, 183.58, 0.00 }, { 135.17, 183.58, 0.00 }, { 135.27, 183.58, 0.00 },
    { 135.37, 183.58, 0.00 },
  },
}
local R13A_MOVE = {
  t0 = 246.934, tOff = 248.600, final = 180.01,
  px = {
    { 246.634, -1.170 }, { 246.784, -1.170 }, { 246.934, -1.170 }, { 246.984, -1.170 }, { 247.034, -1.170 }, { 247.084, -1.170 },
    { 247.151, -1.180 }, { 247.201, -1.180 }, { 247.267, -1.180 }, { 247.317, -1.180 }, { 247.367, -1.180 }, { 247.417, -1.180 },
    { 247.467, -1.180 }, { 247.517, -1.180 }, { 247.568, -1.180 }, { 247.634, -1.190 }, { 247.684, -1.190 }, { 247.751, -1.190 },
    { 247.801, -1.190 }, { 247.868, -1.190 }, { 247.934, -1.190 }, { 247.984, -1.190 }, { 248.051, -1.190 }, { 248.118, -1.190 },
    { 248.168, -1.190 }, { 248.234, -1.200 }, { 248.301, -1.200 }, { 248.351, -1.210 }, { 248.417, -1.210 }, { 248.467, -1.210 },
    { 248.517, -1.200 }, { 248.567, -1.190 }, { 248.617, -1.190 }, { 248.667, -1.180 }, { 248.718, -1.170 }, { 248.767, -1.170 },
    { 248.818, -1.170 }, { 248.868, -1.170 }, { 248.935, -1.170 }, { 249.001, -1.170 }, { 249.051, -1.180 }, { 249.101, -1.180 },
    { 249.151, -1.180 }, { 249.201, -1.170 }, { 249.251, -1.160 }, { 249.301, -1.150 }, { 249.368, -1.120 }, { 249.418, -1.090 },
    { 249.484, -1.070 }, { 249.550, -1.060 }, { 249.600, -1.050 }, { 249.666, -1.040 }, { 249.716, -1.030 }, { 249.767, -1.030 },
    { 249.833, -1.030 }, { 249.900, -1.040 }, { 249.950, -1.060 },
  },
  py = {
    { 246.634, -0.973 }, { 246.784, -0.342 }, { 246.934, 0.290 }, { 246.984, 0.501 }, { 247.034, 0.709 }, { 247.084, 0.917 }, { 247.151, 1.201 },
    { 247.201, 1.405 }, { 247.267, 1.685 }, { 247.317, 1.890 }, { 247.367, 2.093 }, { 247.417, 2.295 }, { 247.467, 2.501 }, { 247.517, 2.702 },
    { 247.568, 2.909 }, { 247.634, 3.170 }, { 247.684, 3.368 }, { 247.751, 3.639 }, { 247.801, 3.832 }, { 247.868, 4.096 }, { 247.934, 4.362 },
    { 247.984, 4.557 }, { 248.051, 4.817 }, { 248.118, 5.082 }, { 248.168, 5.271 }, { 248.234, 5.537 }, { 248.301, 5.778 }, { 248.351, 5.966 },
    { 248.417, 6.215 }, { 248.467, 6.329 }, { 248.517, 6.371 }, { 248.567, 6.378 }, { 248.617, 6.374 }, { 248.667, 6.375 }, { 248.718, 6.383 },
    { 248.767, 6.374 }, { 248.818, 6.382 }, { 248.868, 6.388 }, { 248.935, 6.384 }, { 249.001, 6.389 }, { 249.051, 6.388 }, { 249.101, 6.395 },
    { 249.151, 6.398 }, { 249.201, 6.397 }, { 249.251, 6.393 }, { 249.301, 6.391 }, { 249.368, 6.401 }, { 249.418, 6.399 }, { 249.484, 6.401 },
    { 249.550, 6.402 }, { 249.600, 6.405 }, { 249.666, 6.404 }, { 249.716, 6.402 }, { 249.767, 6.411 }, { 249.833, 6.410 }, { 249.900, 6.418 },
    { 249.950, 6.428 },
  },
  z = {
    { 246.634, 182.709 }, { 246.784, 182.709 }, { 246.934, 182.709 }, { 246.984, 183.291 }, { 247.034, 183.710 }, { 247.084, 184.100 },
    { 247.151, 184.542 }, { 247.201, 184.832 }, { 247.267, 185.138 }, { 247.317, 185.331 }, { 247.367, 185.493 }, { 247.417, 185.596 },
    { 247.467, 185.678 }, { 247.517, 185.715 }, { 247.568, 185.712 }, { 247.634, 185.649 }, { 247.684, 185.542 }, { 247.751, 185.352 },
    { 247.801, 185.162 }, { 247.868, 184.842 }, { 247.934, 184.451 }, { 247.984, 184.122 }, { 248.051, 183.591 }, { 248.118, 183.005 },
    { 248.168, 182.501 }, { 248.234, 181.836 }, { 248.301, 181.013 }, { 248.351, 180.295 }, { 248.417, 180.010 }, { 248.467, 180.010 },
    { 248.517, 180.010 }, { 248.567, 180.010 }, { 248.617, 180.010 }, { 248.667, 180.010 }, { 248.718, 180.010 }, { 248.767, 180.010 },
    { 248.818, 180.010 }, { 248.868, 180.010 }, { 248.935, 180.010 }, { 249.001, 180.010 }, { 249.051, 180.010 }, { 249.101, 180.010 },
    { 249.151, 180.010 }, { 249.201, 180.010 }, { 249.251, 180.010 }, { 249.301, 180.010 }, { 249.368, 180.010 }, { 249.418, 180.010 },
    { 249.484, 180.010 }, { 249.550, 180.010 }, { 249.600, 180.010 }, { 249.666, 180.010 }, { 249.716, 180.010 }, { 249.767, 180.010 },
    { 249.833, 180.010 }, { 249.900, 180.010 }, { 249.950, 180.010 },
  },
  -- the logged camTrace samples (t, pivotZ, V horizontal speed)
  cam = {
    { 246.97, 184.09, 4.18 }, { 247.07, 184.68, 4.16 }, { 247.17, 185.72, 4.12 }, { 247.27, 186.56, 4.09 }, { 247.37, 187.10, 4.06 },
    { 247.47, 187.28, 4.03 }, { 247.57, 187.30, 4.00 }, { 247.67, 187.14, 3.97 }, { 247.77, 186.82, 3.94 }, { 247.87, 186.34, 3.91 },
    { 247.97, 185.70, 3.88 }, { 248.07, 184.90, 3.85 }, { 248.17, 183.94, 3.82 }, { 248.27, 182.90, 3.80 }, { 248.37, 182.13, 3.77 },
    { 248.47, 181.70, 1.84 }, { 248.57, 181.59, 0.00 }, { 248.67, 181.59, 0.00 }, { 248.77, 181.59, 0.00 }, { 248.87, 181.59, 0.00 },
    { 248.97, 181.59, 0.00 }, { 249.07, 181.59, 0.00 }, { 249.17, 181.59, 0.00 }, { 249.27, 181.59, 0.00 }, { 249.37, 181.59, 0.00 },
    { 249.47, 181.59, 0.00 }, { 249.57, 181.59, 0.00 },
  },
}

local function lerpTab(tab, t)
  if t <= tab[1][1] then return tab[1][2] end
  for i = 2, #tab do
    if t <= tab[i][1] then
      local a, b = tab[i - 1], tab[i]
      return a[2] + (b[2] - a[2]) * (t - a[1]) / (b[1] - a[1])
    end
  end
  return tab[#tab][2]
end
-- cubic Hermite through the samples (Catmull-Rom slopes)
local function hermite(tab, t)
  if t <= tab[1][1] then return tab[1][2] end
  if t >= tab[#tab][1] then return tab[#tab][2] end
  for i = 2, #tab do
    if t <= tab[i][1] then
      local p0, p1 = tab[math.max(1, i - 2)], tab[i - 1]
      local p2, p3 = tab[i], tab[math.min(#tab, i + 1)]
      local function slope(a, b, c) return (c[2] - a[2]) / (c[1] - a[1]) end
      local m1 = slope(p0, p1, p2)
      if i - 1 == 1 then m1 = (p2[2] - p1[2]) / (p2[1] - p1[1]) end
      local m2 = slope(p1, p2, p3)
      if i == #tab then m2 = (p2[2] - p1[2]) / (p2[1] - p1[1]) end
      local h = p2[1] - p1[1]
      local s = (t - p1[1]) / h
      local s2, s3 = s * s, s * s * s
      return (2 * s3 - 3 * s2 + 1) * p1[2] + (s3 - 2 * s2 + s) * h * m1 + (-2 * s3 + 3 * s2) * p2[2] + (s3 - s2) * h * m2
    end
  end
end

-- V's trajectory: xy = the speed profile integrated along the logged direction (1 ms grid), z = Hermite
local function trajectory(fix, sharp)
  if fix.px then return function(t) return 1000 + hermite(fix.px, t), 2000 + hermite(fix.py, t), hermite(fix.z, t) end end -- (logged positions)
  local sp = sharp and fix.sharp or fix.speed
  local grid, s, dtg = {}, 0, 0.001
  local tstart = fix.t0 - 0.3
  for i = 0, math.floor((fix.z[#fix.z][1] - tstart) / dtg) do
    local t = tstart + i * dtg
    grid[i] = s
    s = s + lerpTab(sp, t) * dtg
  end
  return function(t)
    local f = (t - tstart) / dtg
    local i = math.max(0, math.min(#grid - 1, math.floor(f)))
    local sI = grid[i] + (grid[i + 1] - grid[i]) * (f - i)
    return 1000 + fix.dir[1] * sI, 2000 + fix.dir[2] * sI, hermite(fix.z, t)
  end
end

-- ---------------------------------------------------------------- replay
-- o: fix, sharp, fps, over (cfg overrides), jit (frame time jitter), noJo (the jump-out input never given), tEnd
local function replay(o)
  local cfg = o.over and CM.config(o.over) or CM.config()
  local fix = o.fix
  local pos = trajectory(fix, o.sharp)
  local pv = CM.newPivot()
  local G = fix.final
  local tNow = 0
  local function ray(from, to)
    local g = G
    if o.hi and tNow < o.hi[1] then g = G + o.hi[2] end -- (the first hit is higher than the ground: the fender / roof of the car V just left)
    if from.z < g or to.z > g then return nil end
    return from.z - g
  end
  local rec = {}
  local seed = 7
  local function rnd() seed = (seed * 16807) % 2147483647; return seed / 2147483647 end
  local t = fix.t0 - 0.2
  local tEnd = o.tEnd or (fix.tOff + 2.2)
  local joEnd
  local nb = 0
  -- warm start: the pivot sits on V
  local x, y, z = pos(t)
  for _ = 1, 8 do CM.pivotStep(pv, cfg, 1 / 60, V(x, y, z), nil, false, nil, nil, nil, nil, nil, nil) end
  while t < tEnd do
    local dt = (1 / o.fps) * (1 + (o.jit or 0) * (2 * rnd() - 1))
    t = t + dt
    tNow = t
    x, y, z = pos(t)
    local on = t < fix.tOff
    local jo = (not o.noJo) and { on = on, ray = ray } or nil
    local wasOn = pv.joOn == true
    pv.snapEv = nil
    local p = CM.pivotStep(pv, cfg, dt, V(x, y, z), nil, false, nil, nil, nil, nil, nil, jo)
    local isOn = pv.joOn == true
    if wasOn and not isOn and joEnd == nil then joEnd = #rec + 1 end
    rec[#rec + 1] = { t = t, dt = dt, px = p.x, py = p.y, pz = p.z, vx = x, vy = y, vz = z, on = isOn, xy = pv.xyJ ~= nil, ev = pv.snapEv ~= nil, rc = pv.joRec == true }
  end
  return rec, joEnd, pv
end

-- metrics of the pivot xy path: peak speed / acceleration / jerk (finite differences over the real frame times), lag to V, offset steps
local function metrics(rec, fix, t0, t1)
  local m = { acc = 0, jerk = 0, lag = 0, offStep = 0, accZ = 0, under = 0, lagTD = nil, endStep = nil, events = 0, maxStep = 0 }
  local pvx, pvy, pa_x, pa_y, pvz
  local prev
  local final = fix.final + 1.55
  for i, r in ipairs(rec) do
    if r.ev then m.events = m.events + 1 end
    if prev then
      local vx, vy = (r.px - prev.px) / r.dt, (r.py - prev.py) / r.dt
      m.maxStep = math.max(m.maxStep, math.sqrt((r.px - prev.px) ^ 2 + (r.py - prev.py) ^ 2))
      if pvx and r.t >= t0 and r.t <= t1 then
        local dtm = (r.dt + prev.dt) / 2
        local ax, ay = (vx - pvx) / dtm, (vy - pvy) / dtm
        m.acc = math.max(m.acc, math.sqrt(ax * ax + ay * ay))
        if pa_x then
          local jx, jy = (ax - pa_x) / dtm, (ay - pa_y) / dtm
          m.jerk = math.max(m.jerk, math.sqrt(jx * jx + jy * jy))
        end
        pa_x, pa_y = ax, ay
      else
        pa_x, pa_y = nil, nil
      end
      pvx, pvy = vx, vy
      local vz = (r.pz - prev.pz) / r.dt
      if pvz and r.t >= t0 and r.t <= t1 then m.accZ = math.max(m.accZ, math.abs(vz - pvz) / ((r.dt + prev.dt) / 2)) end
      pvz = vz
      -- offset to V: how fast it changes per frame
      local ox0, oy0 = prev.px - prev.vx, prev.py - prev.vy
      local ox1, oy1 = r.px - r.vx, r.py - r.vy
      m.offStep = math.max(m.offStep, math.sqrt((ox1 - ox0) ^ 2 + (oy1 - oy0) ^ 2))
    end
    local lag = math.sqrt((r.px - r.vx) ^ 2 + (r.py - r.vy) ^ 2)
    m.lag = math.max(m.lag, lag)
    if r.t >= fix.tOff - 0.2 then m.under = math.max(m.under, final - r.pz) end
    prev = r
  end
  return m
end

local function fmt(m)
  return string.format("acc %.0f m/s2, jerk %.0f m/s3, lag %.2f m, offset step %.4f m/frame, events %d", m.acc, m.jerk, m.lag, m.offStep, m.events)
end

-- ---------------------------------------------------------------- the config
do
  local c = CM.config()
  check("r13 jump-out xy cfg: defaults", c.jumpOutXY == true and c.jumpOutXYTime == 0.25 and c.jumpOutXYVel == 0.05 and c.jumpOutXYLag == 0.35 and c.jumpOutXYAmax == 40 and c.jumpOutXYLagHard == 1.0,
        string.format("%s %s %s %s", tostring(c.jumpOutXY), tostring(c.jumpOutXYTime), tostring(c.jumpOutXYVel), tostring(c.jumpOutXYLag)))
end

-- ---------------------------------------------------------------- replays
local function run(fix, sharp, fps, over, jit)
  local rec, joEnd, pv = replay({ fix = fix, sharp = sharp, fps = fps, over = over, jit = jit })
  local m = metrics(rec, fix, fix.tOff - 0.55, fix.tOff + 0.9)
  return m, rec, joEnd, pv
end
local OLD = { jumpOutXY = false }
local CASES = {
  -- name, fixture, sharp, new limits { acc, jerk }, old floors { acc, jerk } (the old code must exceed them: the checks fail on it)
  { "stand 10 Hz", STAND, false, { 50, 650 }, { 50, 1000 } },
  { "stand sharp (push over 2 frames)", STAND, true, { 60, 2000 }, { 100, 3000 } },
  { "move 10 Hz", MOVE, false, { 30, 260 }, { 20, 400 } },
  { "move sharp (stop in 4 frames)", MOVE, true, { 36, 600 }, { 55, 1400 } },
}
local cfgL = CM.config().jumpOutXYLag
local cfgLH = CM.config().jumpOutXYLagHard
local function offLen(r) return math.sqrt((r.px - r.vx) ^ 2 + (r.py - r.vy) ^ 2) end
for _, fps in ipairs({ 30, 60, 144 }) do
  for _, c in ipairs(CASES) do
    local name, fix, sharp, lim, floor = c[1], c[2], c[3], c[4], c[5]
    local mNew, rNew, jNew, pvNew = run(fix, sharp, fps)
    local mOld, rOld = run(fix, sharp, fps, OLD)
    print(string.format("%s %d fps: new %s | old %s", name, fps, fmt(mNew), fmt(mOld)))
    check(string.format("%s %d fps: the pivot's xy acceleration <= %d m/s2 and jerk <= %d m/s3 (%s)", name, fps, lim[1], lim[2], fmt(mNew)),
          mNew.acc <= lim[1] and mNew.jerk <= lim[2], fmt(mNew))
    check(string.format("%s %d fps: the old code (jumpOutXY = false) is at or above jerk %d (%s) - the limits above bite", name, fps, floor[2], fmt(mOld)),
          mOld.jerk >= floor[2] and (not sharp or mOld.acc >= floor[1]), fmt(mOld))
    check(string.format("%s %d fps: the offset to V stays at about jumpOutXYLag (%.2f <= %.2f + 0.06), no snap / clamp event", name, fps, mNew.lag, cfgL),
          mNew.lag <= cfgL + 0.06 and mNew.events == 0, fmt(mNew))
    -- the vertical path is not touched: the same pivot z, and no undershoot below V's stance
    local sameZ = #rNew == #rOld
    for k = 1, #rNew do if rNew[k].pz ~= rOld[k].pz then sameZ = false end end
    check(string.format("%s %d fps: the pivot height is bit for bit the old one, no undershoot below V's stance (%.3f m)", name, fps, mNew.under), sameZ and mNew.under <= 0.01)
    -- continuity at the end of the jump-out mode: the follower goes on, no kink; once it sat on V the pivot is V (0.1 mm)
    local flip = jNew
    local flipOk = flip ~= nil and flip > 5
    local stepAt, stepRef = 0, 0
    if flipOk then
      local function dOff(k)
        local x0, y0 = rNew[k - 1].px - rNew[k - 1].vx, rNew[k - 1].py - rNew[k - 1].vy
        local x1, y1 = rNew[k].px - rNew[k].vx, rNew[k].py - rNew[k].vy
        return math.sqrt((x1 - x0) ^ 2 + (y1 - y0) ^ 2)
      end
      stepAt = dOff(flip)
      for k = flip - 4, flip - 1 do stepRef = math.max(stepRef, dOff(k)) end
    end
    local endLag = offLen(rNew[#rNew])
    check(string.format("%s %d fps: the end of the jump-out mode (hold off + settle) is no kink: the offset step in that frame %.4f m <= the 4 frames before (%.4f) + 3 mm; the pivot is V's xy at the end (%.5f m)",
          name, fps, stepAt, stepRef, endLag), flipOk and stepAt <= stepRef + 0.003 and endLag < 1e-3 and pvNew.xyJ == nil)
  end
end
-- frame time jitter (+-20 %)
do
  local m = run(STAND, true, 60, nil, 0.2)
  check(string.format("stand sharp 60 fps with +-20 %% frame time jitter: acceleration <= 90 m/s2, offset about jumpOutXYLag (%s)", fmt(m)), m.acc <= 90 and m.lag <= cfgL + 0.06, fmt(m))
  local o = run(STAND, true, 60, OLD, 0.2)
  check(string.format("stand sharp 60 fps jitter, old code: acceleration above 100 m/s2 (%.0f)", o.acc), o.acc >= 100)
end

-- ---------------------------------------------------------------- the switches restore the old behaviour
do
  local function sameAsV(o)
    local rec, _, pv = replay(o)
    for _, r in ipairs(rec) do if r.px ~= r.vx or r.py ~= r.vy then return false end end
    return true, pv
  end
  check("jumpOutXY = false: the pivot xy is V's xy in every frame (bit for bit)", sameAsV({ fix = STAND, sharp = true, fps = 60, over = OLD }))
  local ok2, pv2 = sameAsV({ fix = STAND, sharp = true, fps = 60, noJo = true })
  check("no jump-out input at all (a normal exit / walking): the pivot xy is V's xy bit for bit, no follower state", ok2 and pv2.xyJ == nil)
  check("jumpOutGov = false (the mode itself off): the pivot xy is V's xy bit for bit", sameAsV({ fix = STAND, sharp = true, fps = 60, over = { jumpOutGov = false } }))
  check("pivotFast = false (the old snap rule): the pivot xy is V's xy bit for bit", sameAsV({ fix = STAND, sharp = true, fps = 60, over = { pivotFast = false } }))
end

-- ---------------------------------------------------------------- single frame behaviours
do
  local cfg = CM.config()
  local pv = CM.newPivot()
  local ray = function() return nil end
  local x, y, z = 10, 20, 100
  for i = 1, 20 do CM.pivotStep(pv, cfg, 1 / 60, V(x + 0.1 * i, y, z), nil, false, nil, nil, nil, nil, nil, { on = true, ray = ray }) end
  check("the follower exists while the mode runs", pv.xyJ ~= nil)
  -- a repeated frame (dt = 0): the pivot does not change
  local p1 = CM.pivotStep(pv, cfg, 1 / 60, V(x + 2.1, y, z), nil, false, nil, nil, nil, nil, nil, { on = true, ray = ray })
  local p0 = CM.pivotStep(pv, cfg, 0, V(x + 2.1, y, z), nil, false, nil, nil, nil, nil, nil, { on = true, ray = ray })
  check("a repeated frame (dt = 0) leaves the pivot bit for bit as it was", p0.x == p1.x and p0.y == p1.y and p0.z == p1.z)
  local p2 = CM.pivotStep(pv, cfg, 1 / 60, V(x + 2.2, y, z), nil, false, nil, nil, nil, nil, nil, { on = true, ray = ray })
  check("... and the next frame goes on from the same state (the repeated frame did not eat a step)", p2.x > p1.x and p2.x < x + 2.2 + 1e-3)
  -- a teleport (10 m in one frame): the pivot is V at once, the follower is gone
  local pt = CM.pivotStep(pv, cfg, 1 / 60, V(x + 12.2, y, z), nil, false, nil, nil, nil, nil, nil, { on = true, ray = ray })
  check("a teleport of V (10 m in one frame): the pivot xy is V's in that frame, the follower state is dropped", pt.x == x + 12.2 and pt.y == y and pv.xyJ == nil)
  -- pin (ragdoll)
  for i = 1, 10 do CM.pivotStep(pv, cfg, 1 / 60, V(x + 12.2 + 0.1 * i, y, z), nil, false, nil, nil, nil, nil, nil, { on = true, ray = ray }) end
  local pp = CM.pivotStep(pv, cfg, 1 / 60, V(x + 14, y, z), nil, false, nil, nil, nil, nil, true, { on = true, ray = ray })
  check("pin (a ragdoll episode): the pivot xy is V's, the follower state is dropped", pp.x == x + 14 and pv.xyJ == nil)
  -- the mode starting while V moves at 12 m/s: no step, no kick (velocity continuous)
  local pv2 = CM.newPivot()
  local px, maxStep, prevStep = nil, 0, nil
  local jerkMax = 0
  for i = 1, 40 do
    local p = CM.pivotStep(pv2, cfg, 1 / 60, V(5 + 0.2 * i, 5, 50), nil, false, nil, nil, nil, nil, nil, { on = i > 10, ray = ray })
    if px then
      local st = p.x - px
      if prevStep then jerkMax = math.max(jerkMax, math.abs(st - prevStep) * 3600) end -- m/s2
      prevStep = st
    end
    px = p.x
  end
  check(string.format("the mode starting while V moves at 12 m/s: the pivot's acceleration stays below 30 m/s2 (%.1f), no kick", jerkMax), jerkMax < 30)
end

-- ---------------------------------------------------------------- a hard stop from 8 m/s (beyond the lag floor): bounded lag, bounded overshoot
do
  local HARD = { t0 = 0, tOff = 1.3, final = 100, dir = { 1, 0 }, z = { { 0, 100 }, { 5, 100 } }, speed = { { 0, 8 }, { 1.0, 8 }, { 1.06, 0 }, { 5, 0 } },
                 sharp = { { 0, 8 }, { 1.0, 8 }, { 1.06, 0 }, { 5, 0 } } }
  for _, fps in ipairs({ 30, 60, 144 }) do
    local rec = replay({ fix = HARD, fps = fps, tEnd = 3.0 })
    local oldRec = replay({ fix = HARD, fps = fps, over = OLD, tEnd = 3.0 })
    local mx, ahead = 0, 0
    local mOld = metrics(oldRec, HARD, 0.9, 2.0)
    local m = metrics(rec, HARD, 0.9, 2.0)
    for _, r in ipairs(rec) do
      mx = math.max(mx, offLen(r))
      ahead = math.max(ahead, r.px - r.vx) -- the pivot past V along the travel direction
    end
    local lastLag = offLen(rec[#rec])
    print(string.format("hard stop 8 m/s %d fps: new acc %.0f jerk %.0f lag %.2f ahead %.2f | old acc %.0f jerk %.0f", fps, m.acc, m.jerk, mx, ahead, mOld.acc, mOld.jerk))
    check(string.format("hard stop from 8 m/s in 0.06 s, %d fps: the offset <= %.2f m (%.2f), the pivot runs past V by <= 0.9 m (%.2f), acceleration %.0f < 0.6 x the old %.0f, sat on V 3 s later (%.5f m)",
          fps, cfgLH, mx, ahead, m.acc, mOld.acc, lastLag), mx <= cfgLH + 1e-6 and ahead <= 0.9 and m.acc <= 0.6 * mOld.acc and lastLag < 1e-3)
  end
end

-- ---------------------------------------------------------------- r13a recordings (game build r13a, 20 Hz jo diag): the landing of both jump-outs
-- R13A_STAND: car standing, V comes down beside the rear fender and is thrown 1.77 m sideways at up to 7 m/s about 0.2 s before the touchdown (the game's
-- AIR feed reports onGround with vz -11.7 m/s 0.94 m above the ground and takes it back 15 ms later: a contact with the car body, not the camera).
-- R13A_MOVE: car at 5.7 m/s, V keeps 3.8 m/s and stops dead within about 0.1 s at the touchdown (heavy landing).
-- The pivot (CM.pivotStep, jump-out mode) is replayed at 30 / 60 / 144 fps over the real V trajectory. Measured on the replay: the pivot's xy speed / acceleration
-- / jerk, its height speed / acceleration around the touchdown, the offset to V, no snap or clamp event, no undershoot below V's stance, the end of the mode.
-- The logged pivot height (camTrace) is the reference for the replay itself (z of the standing car is the vehicle frame height, the moving one the logged V height).
local R13A = {
  { name = "r13a stand", fix = R13A_STAND, vxy = 7.3, vz = 10.0, lagMax = 0.56,
    -- limits per fps: { xy acceleration m/s2, xy jerk m/s3 }; the old code (pivot xy = V's xy) must be above the jerk `floor` (m/s3)
    lim = { [30] = { 58, 850 }, [60] = { 63, 1250 }, [144] = { 64, 1900 } }, floor = { 3000 } },
  { name = "r13a move", fix = R13A_MOVE, vxy = 4.3, vz = 13.0, lagMax = 0.34,
    lim = { [30] = { 30, 360 }, [60] = { 31, 380 }, [144] = { 31, 400 } }, floor = { 700 } },
}
local function r13aStats(rec, fix, t0, t1)
  local s = { zSpeed = 0, zAcc = 0, xySpeed = 0, xyStepMax = 0 }
  local pz, pvz
  for i = 2, #rec do
    local r, p = rec[i], rec[i - 1]
    if r.t >= t0 and r.t <= t1 then
      local vz = (r.pz - p.pz) / r.dt
      s.zSpeed = math.max(s.zSpeed, math.abs(vz))
      if pvz then s.zAcc = math.max(s.zAcc, math.abs(vz - pvz) / ((r.dt + p.dt) / 2)) end
      pvz = vz
      local st = math.sqrt((r.px - p.px) ^ 2 + (r.py - p.py) ^ 2)
      s.xyStepMax = math.max(s.xyStepMax, st)
      s.xySpeed = math.max(s.xySpeed, st / r.dt)
    else
      pvz = nil
    end
  end
  return s
end
for _, fps in ipairs({ 30, 60, 144 }) do
  for _, c in ipairs(R13A) do
    local fix, lim = c.fix, c.lim[fps]
    local rec, joEnd, pv = replay({ fix = fix, fps = fps, tEnd = fix.tOff + 2.5 })
    local recOld = replay({ fix = fix, fps = fps, over = OLD, tEnd = fix.tOff + 2.5 })
    local w0, w1 = fix.tOff - 0.8, fix.tOff + 0.9
    local m = metrics(rec, fix, w0, w1)
    local mOld = metrics(recOld, fix, w0, w1)
    local st = r13aStats(rec, fix, w0, w1)
    local stOld = r13aStats(recOld, fix, w0, w1)
    print(string.format("%s %d fps: new %s | zSpeed %.1f zAcc %.1f xySpeed %.1f step %.3f | old acc %.0f jerk %.0f", c.name, fps, fmt(m), st.zSpeed, st.zAcc, st.xySpeed, st.xyStepMax, mOld.acc, mOld.jerk))
    check(string.format("%s %d fps: the pivot's xy acceleration <= %d m/s2 and jerk <= %d m/s3 over the landing (%s)", c.name, fps, lim[1], lim[2], fmt(m)),
          m.acc <= lim[1] and m.jerk <= lim[2], fmt(m))
    check(string.format("%s %d fps: the pivot never moves faster than V's own peak (%.1f <= %.1f m/s xy, per frame %.3f m) and its height speed stays under V's fall (%.1f <= %.1f m/s)",
          c.name, fps, st.xySpeed, c.vxy, st.xyStepMax, st.zSpeed, c.vz), st.xySpeed <= c.vxy and st.zSpeed <= c.vz)
    check(string.format("%s %d fps: the vertical brake is jerk limited around the touchdown: height acceleration %.0f <= 40 m/s2", c.name, fps, st.zAcc), st.zAcc <= 40)
    check(string.format("%s %d fps: the old code (pivot xy = V's xy) is above jerk %d m/s3 (%.0f, acceleration %.0f): the limits bite", c.name, fps, c.floor[1], mOld.jerk, mOld.acc), mOld.jerk >= c.floor[1])
    check(string.format("%s %d fps: offset to V <= %.2f m (%.2f), far inside jumpOutXYLagHard %.1f; no snap / clamp event", c.name, fps, c.lagMax, m.lag, cfgLH), m.lag <= c.lagMax and m.events == 0)
    local sameZ = #rec == #recOld
    for k = 1, #rec do if rec[k].pz ~= recOld[k].pz then sameZ = false end end
    check(string.format("%s %d fps: no undershoot below V's stance (%.3f m), the pivot height is the old one bit for bit", c.name, fps, m.under), sameZ and m.under <= 0.01)
    -- the end of the mode: the follower goes on without a kink and sits on V at the end
    local flipOk = joEnd ~= nil and joEnd > 5
    local stepAt, stepRef = 0, 0
    if flipOk then
      local function dOff(k)
        local x0, y0 = rec[k - 1].px - rec[k - 1].vx, rec[k - 1].py - rec[k - 1].vy
        local x1, y1 = rec[k].px - rec[k].vx, rec[k].py - rec[k].vy
        return math.sqrt((x1 - x0) ^ 2 + (y1 - y0) ^ 2)
      end
      stepAt = dOff(joEnd)
      for k = joEnd - 4, joEnd - 1 do stepRef = math.max(stepRef, dOff(k)) end
    end
    check(string.format("%s %d fps: end of the jump-out mode: offset step %.4f m <= the 4 frames before (%.4f) + 3 mm, the pivot is V's xy at the end (%.5f m)", c.name, fps, stepAt, stepRef, offLen(rec[#rec])),
          flipOk and stepAt <= stepRef + 0.003 and offLen(rec[#rec]) < 1e-3 and pv.xyJ == nil)
  end
end
-- the first ground hit under V is the car she just left (0.6 m above the ground until 247.85, the log's estimate fell by 0.58 m at 247.87 with no brake latched and the whole
-- landing ran on the recovery brake, measured 27 - 34 m/s2 at 10 Hz): a fall of the estimate with no landing brake latched only replaces it (jumpOutDropLatched),
-- the nominal brake plan follows. jumpOutDropLatched = false is the previous behaviour (recovery brake from the drop on).
for _, fps in ipairs({ 30, 60, 144 }) do
  local fix = R13A_MOVE
  local function run2(over)
    local rec = replay({ fix = fix, fps = fps, over = over, hi = { 247.85, 0.6 }, tEnd = fix.tOff + 1.0 })
    local rc = false
    for _, r in ipairs(rec) do if r.rc then rc = true end end
    local s = r13aStats(rec, fix, fix.tOff - 0.8, fix.tOff + 0.9)
    local m = metrics(rec, fix, fix.tOff - 0.8, fix.tOff + 0.9)
    return s, m, rc, rec
  end
  local sN, mN, rcN = run2(nil)
  local sO, mO, rcO = run2({ jumpOutDropLatched = false })
  print(string.format("r13a move %d fps, false first hit: new height acceleration %.1f (recovery %s) | previous %.1f (recovery %s)", fps, sN.zAcc, tostring(rcN), sO.zAcc, tostring(rcO)))
  check(string.format("r13a move %d fps, first hit 0.6 m above the ground: no recovery brake, height acceleration %.1f <= 30 m/s2 and no undershoot (%.3f)", fps, sN.zAcc, mN.under),
        not rcN and sN.zAcc <= 30 and mN.under <= 0.01 and mN.events == 0)
  check(string.format("r13a move %d fps, first hit 0.6 m above the ground: the previous behaviour (jumpOutDropLatched = false) enters the recovery brake and brakes harder (%.1f >= %.1f + 1 m/s2)", fps, sO.zAcc, sN.zAcc),
        rcO and sO.zAcc >= sN.zAcc + 1)
end

-- the per-frame diagnostics of the jump-out step (trace line "jo frame" of utpp_camera): V's height / vertical speed / frame time of the last step
do
  local rec, _, pv = replay({ fix = R13A_STAND, fps = 60, tEnd = R13A_STAND.t0 + 0.5 })
  check("CM.jumpOutStep leaves V's height / vertical speed / frame time of its last step for the per-frame trace line", pv.joOn == true and pv.joDdt ~= nil and pv.joDdt > 0 and pv.joDz ~= nil and pv.joDv ~= nil)
end
-- the replay against the log: the pivot height of the replay vs the logged camTrace pivotZ (10 Hz) at 60 fps
for _, c in ipairs(R13A) do
  local fix = c.fix
  local rec = replay({ fix = fix, fps = 60, tEnd = fix.tOff + 1.0 })
  local worst, worstT = 0, 0
  for _, s in ipairs(fix.cam) do
    if s[1] >= fix.t0 + 0.2 and s[1] <= fix.tOff + 0.4 then
      local best, bd = nil, 1
      for _, r in ipairs(rec) do if math.abs(r.t - s[1]) < bd then bd, best = math.abs(r.t - s[1]), r.pz end end
      if best and math.abs(best - s[2]) > worst then worst, worstT = math.abs(best - s[2]), s[1] end
    end
  end
  print(string.format("%s: replay pivot height vs the logged camTrace pivotZ: worst %.2f m at %.2f", c.name, worst, worstT))
  -- the real base brakes harder than the replay (logged base speed -9.3 -> -0.6 m/s in 0.3 s = 27 - 31 m/s2, the move run with the recovery brake: 27 - 34; the replay
  -- 18 - 23): the replay pivot is up to 0.4 m higher at the touchdown. The replay is a model of the path, not of the brake: see the per-frame "jo frame" trace line.
  check(string.format("%s: the replayed pivot height follows the logged one within 0.45 m (%.2f m at t=%.2f; 10 Hz log, Hermite V path, nominal brake in the replay)", c.name, worst, worstT), worst <= 0.45)
end

-- ---------------------------------------------------------------- 2026-11-05 r13g / r13f landing framing (cfg.jumpOutFrame, CM.jumpOutFrameStep)
-- Game logs r13g-jumpout / r13f-jumpout (jumpout_stand-lines.txt, the per-frame "jo frame" lines at about 60 Hz): V's position of every frame and the logged pivot
-- base. The landing brake stopped the base on the ground 0.70 m (r13g) / 0.43 m (r13f) above V at her touchdown with the look pitch unchanged: V's chest
-- (V + 1.3 m) dropped up to 13.6 / 9.2 deg below its place in the frame and came back as the base sank (video jumpout_stand-tail-3 9.10 - 9.45 s).
-- Replayed through CM.pivotStep (jump-out mode on until the logged hold OFF, the ground at V's landed height) and CM.jumpOutFrameStep at 30 / 60 / 144 fps.
-- The camera is the pivot plus the boom measured after the landing (r13g: 2.96 m behind V, 1.89 m over her feet; yaw and look pitch constant through the
-- landing in both logs). The framing error of a frame = the elevation of V's chest seen from the camera minus the elevation of the same point over the pivot base
-- (its place when the base sits on V, the on-foot framing at the same look pitch), minus the shown tilt. Measured from V's apex to 0.6 s after the touchdown.
-- The 60 Hz samples are interpolated linearly; outside them V holds the first / last sample.
local function frameFix(name, tOff, final, f) return { name = name, tOff = tOff, final = final, f = f } end
local R13G = frameFix("r13g", 545.922, 181.980, {
    { 544.339, 0.000, 0.000, 184.496, 184.234 }, { 544.356, -0.001, 0.024, 184.635, 184.434 }, { 544.373, 0.000, 0.048, 184.775, 184.634 },
    { 544.389, 0.000, 0.074, 184.916, 184.828 }, { 544.406, 0.000, 0.099, 185.051, 185.019 }, { 544.422, 0.000, 0.125, 185.183, 185.198 },
    { 544.439, -0.000, 0.153, 185.321, 185.369 }, { 544.456, -0.000, 0.175, 185.432, 185.527 }, { 544.472, -0.000, 0.200, 185.551, 185.673 },
    { 544.489, 0.000, 0.225, 185.664, 185.802 }, { 544.506, -0.001, 0.249, 185.773, 185.920 }, { 544.522, -0.001, 0.275, 185.884, 186.025 },
    { 544.539, -0.001, 0.299, 185.980, 186.122 }, { 544.556, -0.002, 0.326, 186.084, 186.212 }, { 544.572, -0.002, 0.349, 186.171, 186.297 },
    { 544.590, -0.002, 0.372, 186.254, 186.378 }, { 544.606, -0.002, 0.398, 186.340, 186.452 }, { 544.622, -0.003, 0.420, 186.415, 186.526 },
    { 544.639, -0.003, 0.445, 186.488, 186.596 }, { 544.656, -0.004, 0.469, 186.558, 186.664 }, { 544.672, -0.004, 0.496, 186.630, 186.728 },
    { 544.689, -0.005, 0.516, 186.683, 186.789 }, { 544.706, -0.005, 0.540, 186.738, 186.843 }, { 544.722, -0.005, 0.564, 186.790, 186.892 },
    { 544.739, -0.005, 0.588, 186.838, 186.937 }, { 544.756, -0.006, 0.611, 186.880, 186.975 }, { 544.772, -0.006, 0.634, 186.918, 187.009 },
    { 544.789, -0.007, 0.664, 186.959, 187.039 }, { 544.806, -0.007, 0.681, 186.981, 187.064 }, { 544.822, -0.008, 0.705, 187.006, 187.084 },
    { 544.839, -0.008, 0.729, 187.027, 187.099 }, { 544.856, -0.008, 0.750, 187.043, 187.110 }, { 544.872, -0.009, 0.773, 187.054, 187.116 },
    { 544.889, -0.009, 0.796, 187.061, 187.118 }, { 544.906, -0.009, 0.821, 187.064, 187.116 }, { 544.922, -0.010, 0.841, 187.062, 187.110 },
    { 544.939, -0.010, 0.864, 187.056, 187.100 }, { 544.956, -0.010, 0.886, 187.045, 187.085 }, { 544.972, -0.011, 0.909, 187.030, 187.066 },
    { 544.989, -0.011, 0.931, 187.010, 187.042 }, { 545.006, -0.012, 0.953, 186.987, 187.014 }, { 545.022, -0.012, 0.977, 186.955, 186.981 },
    { 545.039, -0.012, 0.997, 186.925, 186.943 }, { 545.056, -0.013, 1.019, 186.886, 186.901 }, { 545.072, -0.013, 1.040, 186.847, 186.854 },
    { 545.089, -0.013, 1.062, 186.800, 186.804 }, { 545.106, -0.013, 1.084, 186.750, 186.749 }, { 545.122, -0.014, 1.106, 186.694, 186.690 },
    { 545.139, -0.014, 1.129, 186.629, 186.625 }, { 545.156, -0.014, 1.148, 186.571, 186.557 }, { 545.172, -0.014, 1.170, 186.502, 186.484 },
    { 545.189, -0.014, 1.191, 186.430, 186.407 }, { 545.206, -0.014, 1.213, 186.350, 186.325 }, { 545.222, -0.014, 1.233, 186.269, 186.238 },
    { 545.239, -0.016, 1.256, 186.174, 186.147 }, { 545.256, -0.016, 1.275, 186.094, 186.051 }, { 545.272, -0.016, 1.296, 185.997, 185.951 },
    { 545.289, -0.016, 1.317, 185.898, 185.849 }, { 545.306, -0.016, 1.338, 185.794, 185.739 }, { 545.322, -0.016, 1.360, 185.678, 185.627 },
    { 545.339, -0.017, 1.379, 185.573, 185.509 }, { 545.356, -0.017, 1.402, 185.444, 185.386 }, { 545.372, -0.017, 1.420, 185.335, 185.259 },
    { 545.389, -0.017, 1.440, 185.207, 185.130 }, { 545.406, -0.018, 1.461, 185.080, 184.995 }, { 545.422, -0.018, 1.481, 184.942, 184.858 },
    { 545.439, -0.019, 1.501, 184.800, 184.718 }, { 545.456, -0.019, 1.523, 184.643, 184.576 }, { 545.472, -0.019, 1.541, 184.509, 184.433 },
    { 545.489, -0.019, 1.561, 184.355, 184.286 }, { 545.506, -0.019, 1.580, 184.199, 184.145 }, { 545.522, -0.019, 1.600, 184.039, 184.004 },
    { 545.539, -0.020, 1.621, 183.870, 183.867 }, { 545.556, -0.020, 1.639, 183.702, 183.734 }, { 545.572, -0.020, 1.662, 183.507, 183.607 },
    { 545.589, -0.020, 1.679, 183.336, 183.486 }, { 545.606, -0.020, 1.698, 183.156, 183.366 }, { 545.622, -0.020, 1.717, 182.972, 183.254 },
    { 545.639, -0.021, 1.736, 182.777, 183.146 }, { 545.656, -0.021, 1.755, 182.585, 183.043 }, { 545.673, -0.021, 1.774, 182.383, 182.944 },
    { 545.689, -0.021, 1.795, 182.150, 182.851 }, { 545.706, -0.021, 1.812, 181.981, 182.761 }, { 545.722, -0.022, 1.831, 181.980, 182.678 },
    { 545.739, -0.022, 1.844, 181.980, 182.599 }, { 545.756, -0.022, 1.850, 181.980, 182.524 }, { 545.772, -0.023, 1.849, 181.980, 182.454 },
    { 545.789, -0.023, 1.849, 181.980, 182.390 }, { 545.806, -0.022, 1.848, 181.980, 182.329 }, { 545.822, -0.022, 1.848, 181.980, 182.274 },
    { 545.839, -0.022, 1.847, 181.980, 182.224 }, { 545.856, -0.023, 1.846, 181.980, 182.178 }, { 545.872, -0.023, 1.846, 181.980, 182.137 },
    { 545.889, -0.023, 1.846, 181.980, 182.100 }, { 545.906, -0.023, 1.845, 181.980, 182.070 }, { 545.922, -0.023, 1.845, 181.980, 182.043 },
    { 545.939, -0.023, 1.844, 181.980, 182.022 }, { 545.956, -0.023, 1.844, 181.980, 182.005 }, { 545.972, -0.023, 1.844, 181.980, 181.993 },
    { 545.989, -0.023, 1.845, 181.980, 181.985 },
})
local R13F = frameFix("r13f", 1502.165, 182.029, {
    { 1500.583, 0.000, 0.000, 184.511, 184.291 }, { 1500.599, 0.001, 0.021, 184.654, 184.491 }, { 1500.615, 0.002, 0.044, 184.795, 184.690 },
    { 1500.632, 0.003, 0.066, 184.931, 184.885 }, { 1500.649, 0.004, 0.089, 185.072, 185.067 }, { 1500.666, 0.005, 0.112, 185.203, 185.240 },
    { 1500.682, 0.006, 0.134, 185.334, 185.401 }, { 1500.699, 0.008, 0.158, 185.463, 185.551 }, { 1500.716, 0.009, 0.178, 185.573, 185.688 },
    { 1500.732, 0.009, 0.201, 185.685, 185.813 }, { 1500.749, 0.010, 0.223, 185.799, 185.927 }, { 1500.766, 0.011, 0.245, 185.903, 186.030 },
    { 1500.782, 0.012, 0.268, 186.007, 186.129 }, { 1500.799, 0.012, 0.288, 186.099, 186.222 }, { 1500.816, 0.014, 0.312, 186.199, 186.312 },
    { 1500.832, 0.014, 0.331, 186.278, 186.396 }, { 1500.849, 0.015, 0.352, 186.361, 186.477 }, { 1500.866, 0.015, 0.374, 186.440, 186.553 },
    { 1500.883, 0.016, 0.395, 186.515, 186.627 }, { 1500.899, 0.017, 0.417, 186.588, 186.694 }, { 1500.916, 0.018, 0.438, 186.651, 186.758 },
    { 1500.932, 0.018, 0.461, 186.717, 186.819 }, { 1500.949, 0.019, 0.479, 186.768, 186.874 }, { 1500.966, 0.019, 0.500, 186.820, 186.924 },
    { 1500.982, 0.021, 0.521, 186.868, 186.968 }, { 1500.999, 0.022, 0.542, 186.911, 187.006 }, { 1501.016, 0.022, 0.563, 186.950, 187.040 },
    { 1501.032, 0.022, 0.586, 186.988, 187.070 }, { 1501.049, 0.023, 0.603, 187.014, 187.095 }, { 1501.066, 0.024, 0.623, 187.039, 187.116 },
    { 1501.082, 0.025, 0.644, 187.061, 187.132 }, { 1501.099, 0.025, 0.664, 187.077, 187.144 }, { 1501.116, 0.027, 0.685, 187.089, 187.151 },
    { 1501.132, 0.027, 0.705, 187.097, 187.154 }, { 1501.149, 0.027, 0.726, 187.100, 187.153 }, { 1501.166, 0.029, 0.745, 187.099, 187.148 },
    { 1501.182, 0.029, 0.764, 187.093, 187.138 }, { 1501.199, 0.030, 0.784, 187.083, 187.124 }, { 1501.216, 0.030, 0.803, 187.069, 187.106 },
    { 1501.232, 0.031, 0.823, 187.049, 187.082 }, { 1501.249, 0.032, 0.844, 187.024, 187.054 }, { 1501.266, 0.033, 0.862, 186.998, 187.020 },
    { 1501.282, 0.034, 0.884, 186.962, 186.984 }, { 1501.299, 0.034, 0.900, 186.930, 186.943 }, { 1501.316, 0.035, 0.919, 186.889, 186.897 },
    { 1501.332, 0.035, 0.938, 186.843, 186.847 }, { 1501.349, 0.036, 0.960, 186.784, 186.792 }, { 1501.366, 0.036, 0.976, 186.739, 186.733 },
    { 1501.382, 0.037, 0.995, 186.679, 186.669 }, { 1501.399, 0.038, 1.014, 186.615, 186.602 }, { 1501.416, 0.039, 1.032, 186.548, 186.530 },
    { 1501.432, 0.039, 1.050, 186.476, 186.454 }, { 1501.449, 0.040, 1.070, 186.390, 186.372 }, { 1501.466, 0.041, 1.087, 186.317, 186.286 },
    { 1501.482, 0.042, 1.105, 186.232, 186.196 }, { 1501.499, 0.043, 1.125, 186.136, 186.099 }, { 1501.516, 0.043, 1.142, 186.046, 186.001 },
    { 1501.532, 0.043, 1.160, 185.948, 185.897 }, { 1501.549, 0.045, 1.181, 185.823, 185.787 }, { 1501.566, 0.045, 1.195, 185.738, 185.673 },
    { 1501.582, 0.045, 1.213, 185.624, 185.557 }, { 1501.599, 0.046, 1.231, 185.507, 185.437 }, { 1501.616, 0.047, 1.249, 185.386, 185.314 },
    { 1501.632, 0.047, 1.266, 185.261, 185.186 }, { 1501.649, 0.048, 1.284, 185.131, 185.054 }, { 1501.666, 0.049, 1.303, 184.985, 184.914 },
    { 1501.682, 0.049, 1.318, 184.857, 184.770 }, { 1501.699, 0.050, 1.335, 184.712, 184.620 }, { 1501.716, 0.051, 1.353, 184.564, 184.467 },
    { 1501.732, 0.051, 1.370, 184.414, 184.309 }, { 1501.749, 0.052, 1.387, 184.247, 184.150 }, { 1501.766, 0.053, 1.406, 184.076, 183.990 },
    { 1501.782, 0.053, 1.420, 183.930, 183.831 }, { 1501.799, 0.054, 1.437, 183.756, 183.674 }, { 1501.816, 0.055, 1.455, 183.577, 183.521 },
    { 1501.832, 0.055, 1.471, 183.401, 183.373 }, { 1501.849, 0.056, 1.487, 183.222, 183.231 }, { 1501.866, 0.057, 1.505, 183.010, 183.097 },
    { 1501.882, 0.057, 1.520, 182.843, 182.971 }, { 1501.899, 0.058, 1.536, 182.645, 182.853 }, { 1501.916, 0.058, 1.552, 182.443, 182.738 },
    { 1501.932, 0.059, 1.569, 182.228, 182.640 }, { 1501.949, 0.059, 1.584, 182.029, 182.546 }, { 1501.965, 0.060, 1.602, 182.029, 182.461 },
    { 1501.982, 0.060, 1.611, 182.029, 182.382 }, { 1501.999, 0.060, 1.614, 182.029, 182.312 }, { 1502.016, 0.060, 1.614, 182.029, 182.250 },
    { 1502.032, 0.060, 1.613, 182.029, 182.196 }, { 1502.049, 0.060, 1.612, 182.029, 182.150 }, { 1502.066, 0.060, 1.612, 182.029, 182.112 },
    { 1502.082, 0.060, 1.611, 182.029, 182.082 }, { 1502.099, 0.060, 1.611, 182.029, 182.060 }, { 1502.116, 0.060, 1.611, 182.029, 182.046 },
    { 1502.133, 0.060, 1.610, 182.029, 182.038 }, { 1502.149, 0.060, 1.610, 182.029, 182.036 }, { 1502.165, 0.060, 1.609, 182.029, 182.038 },
    { 1502.182, 0.060, 1.610, 182.029, 182.042 }, { 1502.199, 0.060, 1.610, 182.029, 182.045 }, { 1502.216, 0.060, 1.610, 182.029, 182.047 },
    { 1502.232, 0.060, 1.609, 182.029, 182.047 }, { 1502.249, 0.060, 1.610, 182.029, 182.045 }, { 1502.266, 0.060, 1.609, 182.029, 182.042 },
    { 1502.282, 0.059, 1.609, 182.029, 182.039 }, { 1502.299, 0.059, 1.609, 182.029, 182.036 }, { 1502.316, 0.060, 1.609, 182.029, 182.032 },
    { 1502.332, 0.060, 1.609, 182.029, 182.030 },
})
local BOOM = V(-2.094, 2.094, 0.338) -- the camera from the pivot (pivot = base + 1.55 m): r13g after the landing
local function frameReplay(fix, fps, over)
  local cfg = over and CM.config(over) or CM.config()
  local F = fix.f
  local function at(t)
    if t <= F[1][1] then return F[1][2], F[1][3], F[1][4], F[1][5] end
    for i = 2, #F do
      if t <= F[i][1] then
        local a, b = F[i - 1], F[i]
        local s = (t - a[1]) / (b[1] - a[1])
        return a[2] + (b[2] - a[2]) * s, a[3] + (b[3] - a[3]) * s, a[4] + (b[4] - a[4]) * s, a[5] + (b[5] - a[5]) * s
      end
    end
    local l = F[#F]
    return l[2], l[3], l[4], l[5]
  end
  local G = fix.final
  local function ray(from, to)
    if from.z < G or to.z > G then return nil end
    return from.z - G
  end
  local pv, st = CM.newPivot(), {}
  local t = F[1][1]
  local x, y, z = at(t)
  for _ = 1, 8 do CM.pivotStep(pv, cfg, 1 / 60, V(1000 + x, 2000 + y, z), nil, false, nil, nil, nil, nil, nil, nil) end -- (warm start: the pivot sits on V)
  local rec, contact, apex, apexZ = {}, nil, nil, -1e9
  while t < F[#F][1] + 0.8 - 1e-9 do
    local dt = 1 / fps
    t = t + dt
    local vx, vy, vz, lb = at(t)
    local P = V(1000 + vx, 2000 + vy, vz)
    local p = CM.pivotStep(pv, cfg, dt, P, nil, false, nil, nil, nil, nil, nil, { on = t < fix.tOff, ray = ray })
    local cam = V(p.x + BOOM.x, p.y + BOOM.y, p.z + BOOM.z)
    local tilt = CM.jumpOutFrameStep(st, pv, cfg, dt, cam, P, 1)
    local hd = math.sqrt((P.x - cam.x) ^ 2 + (P.y - cam.y) ^ 2)
    local e = math.deg(math.atan2(vz + 1.3 - cam.z, hd) - math.atan2(pv.base + 1.3 - cam.z, hd))
    if contact == nil and vz <= G + 1e-3 then contact = t end
    if vz > apexZ then apexZ, apex = vz, t end
    rec[#rec + 1] = { t = t, dt = dt, cam = cam, tilt = tilt, e = e, dev = e - tilt, gap = pv.base - vz, lb = lb, base = pv.base, on = pv.joOn == true }
  end
  return rec, contact, apex, st
end
local function frameStats(rec, contact, apex)
  local s = { dev = 0, devAt = nil, rate = 0, rateL = 0, acc = 0, accAt = nil, gapTD = nil, settle = 0, tiltMax = 0, devE = 0 }
  for i, r in ipairs(rec) do
    if r.t >= apex and r.t <= contact + 0.6 then
      if math.abs(r.dev) > s.dev then s.dev, s.devAt = math.abs(r.dev), r.t end
      s.devE = math.max(s.devE, math.abs(r.e))
      s.tiltMax = math.max(s.tiltMax, math.abs(r.tilt))
      if i > 2 then
        local p, q = rec[i - 1], rec[i - 2]
        s.rate = math.max(s.rate, math.abs(r.tilt - p.tilt) / r.dt)
        if r.t >= contact - 1e-6 then s.rateL = math.max(s.rateL, math.abs(r.tilt - p.tilt) / r.dt) end -- (from the touchdown on)
        local v1 = CM.scale(CM.sub(r.cam, p.cam), 1 / r.dt)
        local v0 = CM.scale(CM.sub(p.cam, q.cam), 1 / p.dt)
        local a = CM.len(CM.sub(v1, v0)) / ((r.dt + p.dt) / 2)
        if a > s.acc then s.acc, s.accAt = a, r.t end
      end
    end
    if s.gapTD == nil and r.t >= contact then s.gapTD = r.gap end
    if r.t >= contact and (math.abs(r.dev) > 1 or math.abs(r.gap) > 0.02 or math.abs(r.tilt) > 0.5) then s.settle = r.t - contact end
  end
  return s
end
local function fmtF(s) return string.format("framing error %.2f deg at %.3f (without the tilt %.2f), tilt peak %.2f deg, tilt rate %.0f deg/s (%.2f deg per 60 fps frame; from the touchdown %.0f deg/s), camera acceleration %.1f m/s2 (%.4f m step change per 60 fps frame), gap at the touchdown %.2f m, settled %.2f s after it",
  s.dev, s.devAt or -1, s.devE, s.tiltMax, s.rate, s.rate / 60, s.rateL, s.acc, s.acc / 3600, s.gapTD or -1, s.settle) end
for _, fix in ipairs({ R13G, R13F }) do
  for _, fps in ipairs({ 30, 60, 144 }) do
    local rec, contact, apex, st = frameReplay(fix, fps)
    local s = frameStats(rec, contact, apex)
    local recO, contactO, apexO = frameReplay(fix, fps, { jumpOutFrame = false })
    local o = frameStats(recO, contactO, apexO)
    print(string.format("%s landing framing %d fps: %s | jumpOutFrame = false: %s", fix.name, fps, fmtF(s), fmtF(o)))
    check(string.format("%s landing %d fps: V's chest stays within 6 deg of its on-foot place in the frame from the apex to 0.6 s after the touchdown (%.2f deg; without the tilt %.2f)",
          fix.name, fps, s.dev, o.dev), contact ~= nil and s.dev <= 6)
    check(string.format("%s landing %d fps: the tilt turns at <= 96 deg/s (1.6 deg per 60 fps frame: %.2f), from the touchdown on <= 60 deg/s (r13e: 1.0 deg per 60 fps frame; %.0f), the camera position has no velocity step (acceleration %.1f <= 72 m/s2 = 0.02 m step change at 60 fps)",
          fix.name, fps, s.rate / 60, s.rateL, s.acc), s.rate <= 96 and s.rateL <= 60 and s.acc <= 72)
    check(string.format("%s landing %d fps: settled %.2f s after the touchdown (<= 0.4 s: framing error <= 1 deg, base within 2 cm of V, tilt <= 0.5 deg); the tilt is exactly 0 at the end",
          fix.name, fps, s.settle), s.settle <= 0.4 and rec[#rec].tilt == 0 and (st.b or 0) == 0)
    check(string.format("%s landing %d fps: the switch jumpOutFrame = false is the reported framing error (%.2f > 6 deg, no tilt) and the same pivot path bit for bit", fix.name, fps, o.dev),
          o.dev > 6 and o.tiltMax == 0 and (function()
            if #rec ~= #recO then return false end
            for k = 1, #rec do if rec[k].base ~= recO[k].base or rec[k].cam.x ~= recO[k].cam.x or rec[k].cam.y ~= recO[k].cam.y then return false end end
            return true
          end)())
  end
  -- the replay against the log: the replayed base vs the logged one (60 fps, per frame)
  local rec = frameReplay(fix, 60)
  local worst, worstT = 0, nil
  for _, r in ipairs(rec) do
    if r.t >= fix.f[1][1] + 0.3 and r.t <= fix.f[#fix.f][1] and math.abs(r.base - r.lb) > worst then worst, worstT = math.abs(r.base - r.lb), r.t end
  end
  print(string.format("%s: replayed pivot base vs the logged base (60 fps): worst %.3f m at %.3f", fix.name, worst, worstT or -1))
  check(string.format("%s: the replayed pivot base follows the logged one within 0.10 m from 0.3 s after the launch (%.3f m at %.3f)", fix.name, worst, worstT or -1), worst <= 0.10)
end
-- the tilt only acts in the jump-out mode's descent: no mode = no state, exactly 0
do
  local cfg = CM.config()
  local pv, st = CM.newPivot(), {}
  local maxT = 0
  for i = 1, 120 do
    local z = 100 + 3 * math.sin(i / 20) -- (V moves up and down at up to 9 m/s without the jump-out input)
    CM.pivotStep(pv, cfg, 1 / 60, V(10, 20, z), nil, false, nil, nil, nil, nil, nil, nil)
    maxT = math.max(maxT, math.abs(CM.jumpOutFrameStep(st, pv, cfg, 1 / 60, V(8, 22, z + 1.9), V(10, 20, z), 1)))
  end
  check("without the jump-out mode the landing framing tilt is exactly 0 (no arming)", maxT == 0 and st.armed == nil)
end

print(string.format("utpp jump-out replay test: %d passed, %d failed", results.pass, results.fail))
assert(results.fail == 0, "utpp jump-out replay test failed")
