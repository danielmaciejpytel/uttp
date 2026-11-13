param(
  [string]$Game = '',
  [switch]$ListOnly,
  [switch]$SkipList,
  [switch]$SkipUnbundle
)
# Dual wield TPP from NPC animations (2026-11-09): list every vanilla NPC gameplay animset of the installed game (content + ep1;
# archive\pc\mod is NOT read, so the data are vanilla), then extract the dual-pistol NPC sets (man_average / man_big handgundual,
# Jackie), the single-handgun / revolver / SMG action sets of woman_average and man_average (one-hand donors, mirrored per hand) and
# the NPC base rigs; serialize them to json (events, durations) and export them to .anims.glb (root motion included).
# Modelled on Source\tools\melee\extract_npc_melee.ps1.  Writes only under Extracted\game\npc-dualwield\ (raw, json, glb, logs,
# listing_*.txt, provenance.txt).  The game folder is only read.
$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot '..\utpp_paths.ps1')
if (-not $Game) { $Game = Get-UtppGameDir }
$root = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $PSScriptRoot))
$wkit = Join-Path $root 'Tools\WolvenKit.CLI\WolvenKit.CLI.exe'
$out = Join-Path $root 'Extracted\game\npc-dualwield'
$raw = Join-Path $out 'raw'; $json = Join-Path $out 'json'; $glb = Join-Path $out 'glb'; $logs = Join-Path $out 'logs'
foreach ($d in @($raw, $json, $glb, $logs)) { New-Item -ItemType Directory -Force $d | Out-Null }
$pc = Join-Path $Game 'archive\pc'

# ---- 1. listing: every *.anims / *.rig of content + ep1 (no lang / audio / video / gi / terrain archives; never archive\pc\mod)
if (!$SkipList) {
  $archives = foreach ($sub in 'content', 'ep1') {
    Get-ChildItem (Join-Path $pc $sub) -Filter '*.archive' | Where-Object { $_.Name -notmatch '^(lang_|audio_)|_gi\.archive$|_terrain\.archive$|_video\.archive$' }
  }
  $all = New-Object System.Collections.Generic.List[string]
  foreach ($a in $archives) {
    $tag = (Split-Path -Leaf (Split-Path -Parent $a.FullName)) + '\' + $a.BaseName
    foreach ($w in '*.anims', '*.rig') {
      $lines = & $wkit archive $a.FullName -l -w $w 2>&1
      if ($LASTEXITCODE -ne 0) { throw "archive -l $tag $w failed $LASTEXITCODE" }
      foreach ($l in $lines) {
        $s = "$l".Trim()
        if ($s -match '\.(anims|rig)$') { $all.Add($tag + '|' + $s) }
      }
    }
    "listed $tag"
  }
  $all | Sort-Object | Set-Content (Join-Path $out 'listing_all_anims_rigs.txt') -Encoding ascii
  # NPC gameplay animsets (every body type and family) and the ranged / dual candidates
  $npc = $all | Where-Object { $_ -match '\|(base|ep1)\\animations\\npc\\gameplay\\' } | Sort-Object
  $npc | Set-Content (Join-Path $out 'listing_npc_gameplay.txt') -Encoding ascii
  $npc | Where-Object { $_ -match '(?i)dual|akimbo|handgun|revolver|smg|pistol' } |
    Set-Content (Join-Path $out 'listing_npc_ranged_candidates.txt') -Encoding ascii
  "listing: $($all.Count) anims/rigs, npc gameplay $($npc.Count)"
}
if ($ListOnly) { return }

# ---- 2. extraction: dual-pistol NPC sets, one-hand pistol / revolver / SMG action sets of the average bodies, rigs
# (listing 2026-11-09: dual pistols exist only for man_average (handgundual action) and man_big (handgundual action + locomotion,
#  Jackie); no woman / dual-SMG NPC set.  One-hand pistol NPC sets = handgunleft_wounded_right_arm (gun in the LEFT hand) and
#  handgunright_wounded_left_arm (gun in the RIGHT hand); handgunboth = two hands on one pistol (stance reference).)
$re = '(animations\\npc\\gameplay\\[a-z_]+\\[a-z_]+\\handgundual\\[^\\]+\.anims$)' +
      '|(animations\\npc\\gameplay\\(woman_average|man_average)\\gang\\handgun(left|right)\\(wa|ma)_gang_handgun(left_wounded_right_arm|right_wounded_left_arm)_actions\.anims$)' +
      '|(animations\\npc\\gameplay\\woman_average\\gang\\handgunboth\\wa_gang_handgunboth_action_combat\.anims$)' +
      '|(woman_base\\woman_base\.rig$)|(man_base\\man_base\.rig$)|(man_big\\man_big\.rig$)' +
      # Female V player dual-wield sets of the pilot (vanilla, read-only reference for the 'Game Animations' side and the time maps):
      # the generic default set (prio 200), the dual-wield pistol base set (100, var dual_wield) and the Militech Lexington pair (64)
      '|(animations\\weapon\\firearms\\dual_wield\\pwa_w_default_weapon_dual_wield\.anims$)' +
      '|(animations\\weapon\\firearms\\dual_wield\\handgun\\base\\pwa_w_handgun_dual_wield_base\.anims$)' +
      '|(animations\\weapon\\firearms\\dual_wield\\handgun\\militech_lexington\\pwa_w_handgun_dual_wield_(left|right)__militech_lexington\.anims$)'
if (!$SkipUnbundle) {
  $hit = @(Get-Content (Join-Path $out 'listing_all_anims_rigs.txt') | Where-Object { ($_ -split '\|')[1] -match $re })
  # the NPC base rigs are listed in basegame_1_engine AND basegame_4_appearance; read them from basegame_1_engine (as the melee extraction)
  $arch = @($hit | ForEach-Object { ($_ -split '\|')[0] } | Where-Object { $_ -ne 'content\basegame_4_appearance' } | Sort-Object -Unique)
  "matching files: $($hit.Count) in $($arch -join ', ')"
  foreach ($a in $arch) {
    $p = Join-Path $pc ($a + '.archive')
    "unbundle $a"
    & $wkit unbundle $p -o $raw -r $re | Out-File (Join-Path $logs ('unbundle_' + ($a -replace '\\', '_') + '.log')) -Encoding utf8
    if ($LASTEXITCODE -ne 0) { throw "unbundle $a failed $LASTEXITCODE" }
  }
}
$files = Get-ChildItem $raw -Recurse -File
$files | ForEach-Object { '{0}  {1}  {2}' -f (Get-FileHash $_.FullName -Algorithm SHA256).Hash, $_.Length, $_.FullName.Substring($raw.Length + 1) } |
  Set-Content (Join-Path $out 'provenance.txt') -Encoding ascii
"raw files: $($files.Count)"
$env:AnimationExportArgs__incRootMotion = 'true'
$anims = $files | Where-Object { $_.Extension -eq '.anims' }
foreach ($f in $anims) {
  & $wkit convert serialize $f.FullName -o $json | Out-File (Join-Path $logs ('json_' + $f.Name + '.log')) -Encoding utf8
}
# glb export: at most 4 WolvenKit processes at a time
$queue = [System.Collections.Queue]::new(@($anims))
$running = @()
while ($queue.Count -gt 0 -or $running.Count -gt 0) {
  while ($queue.Count -gt 0 -and $running.Count -lt 4) {
    $f = $queue.Dequeue()
    if (Test-Path (Join-Path $glb ($f.Name + '.glb'))) { continue }     # already exported (re-runs export only what is missing)
    $a = @('export', ('"' + $f.FullName + '"'), '-o', ('"' + $glb + '"'), '-gp', ('"' + $Game + '"'))
    $running += Start-Process -FilePath $wkit -ArgumentList $a -RedirectStandardOutput (Join-Path $logs ('export_' + $f.Name + '.log')) `
      -RedirectStandardError (Join-Path $logs ('export_' + $f.Name + '.err')) -PassThru -WindowStyle Hidden
  }
  Start-Sleep -Milliseconds 500
  $running = @($running | Where-Object { !$_.HasExited })
}
Get-ChildItem $glb, $json -File | Select-Object Name, Length | Format-Table -AutoSize | Out-String -Width 200
