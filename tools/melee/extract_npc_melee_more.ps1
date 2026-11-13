param(
  [string]$Game = '',
  [switch]$SkipUnbundle,
  [switch]$SkipExport,
  [int]$Parallel = 6,
  [int]$BatchTimeoutSec = 900
)
# Extract MORE vanilla third-person melee sources for the player melee sets not rebuilt yet (2026-11-09), from the installed game
# archives (content + ep1 only; archive\pc\mod is NOT read, so the data are vanilla):
#   * NPC body sets: gang\onehanded (wa / ma / mb: action_combat, locomotion_combat, reaction), android\onehanded (ma, EP1),
#     mantisblades (wa / ma, corpo / gang / oda: action_combat, locomotion_combat, incl. the cw_ blade-part sets), gang\strongarms
#     (wa / ma / mb / mm), gang\hammer (mb / mm: action_combat, locomotion_combat, reaction);
#   * cyberware part sets: cw_monowire*, cw_mantisblade*, and the player-rig synced finisher sets (baton, fists, one-handed blunt, axe,
#     two-handed blunt, hammer, knife, monowire, strong arms, mantis blades) that are not registered on the player entity;
#   * rigs: man_big, man_massive, mantis blade skeleton, monowire whip; NPC base entities (animset registrations, json only).
# Every archive is unbundled into its own folder raw\<archive>\ (no silent overwrite when a depot path exists in two archives); the
# copy used for json / glb is the one of the LAST archive in load order (content, then ep1). provenance.txt: SHA-256, size, archive,
# depot path, 'used' / 'shadowed'. json: WolvenKit 'convert serialize'; glb: 'export -gp' with root motion (AnimationExportArgs__
# incRootMotion=true), run in batches of -Parallel processes, each batch killed after -BatchTimeoutSec.
# Writes only under Extracted\game\npc-melee\more\ (raw, json, glb, logs, provenance.txt). The game folder is only read.
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $PSScriptRoot))
. (Join-Path $PSScriptRoot '..\utpp_paths.ps1')
if (-not $Game) { $Game = Get-UtppGameDir }
$wkit = Join-Path $root 'Tools\WolvenKit.CLI\WolvenKit.CLI.exe'
$out = Join-Path $root 'Extracted\game\npc-melee\more'
$raw = Join-Path $out 'raw'; $json = Join-Path $out 'json'; $glb = Join-Path $out 'glb'; $logs = Join-Path $out 'logs'
foreach ($d in @($raw, $json, $glb, $logs)) { New-Item -ItemType Directory -Force $d | Out-Null }
# load order: content first, ep1 last (ep1 wins for a depot path present in both)
$arch = @('content\basegame_1_engine', 'content\basegame_4_animation', 'content\basegame_4_appearance', 'ep1\ep1_1_nightcity', 'ep1\ep1_2_gamedata')
$g = 'animations\\npc\\gameplay\\'
$re = @(
  ($g + '(woman_average|man_average|man_big)\\gang\\onehanded\\(wa|ma|mb)_gang_onehanded_(action_combat|locomotion_combat|reaction)\.anims$'),
  ($g + 'man_average\\android\\onehanded\\ma_android_onehanded_action_combat\.anims$'),
  ($g + '(woman_average|man_average)\\(corpo|gang|oda)\\mantisblades\\(wa|ma|cw)_(corpo|gang|oda)_mantisblades_(action_combat|locomotion_combat)\.anims$'),
  ($g + '(woman_average|man_average|man_big|man_massive)\\gang\\strongarms\\(wa|ma|mb|mm)_gang_strongarms_action_combat\.anims$'),
  ($g + '(man_big|man_massive)\\gang\\hammer\\(mb|mm)_gang_hammer_(action_combat|locomotion_combat|reaction)\.anims$'),
  'animations\\cyberware\\monowire\\(cw_monowire(_ma|_wa|_finisher)?|p[wm]a_monowire_finisher)\.anims$',
  'animations\\cyberware\\mantisblade\\(cw_mantisblade(_ma|_wa|_finisher)?|p[wm]a_mantisblade_finisher)\.anims$',
  'animations\\cyberware\\strongarms\\(p[wm]a_strongarms_finisher|cw_p[wm]a_strongarms_finisher)\.anims$',
  'animations\\weapon\\(baton\\p[wm]a_baton_finisher|melee_fists\\p[wm]a_fist_finisher|one_handed_blunt\\p[wm]a_(finisher_one_handed_blunt|axe_finisher)|two_handed_blunt\\p[wm]a_two_handed_blunt_finisher|two_handed_hammer\\p[wm]a_two_handed_hammer_finisher|knife\\p[wm]a_knife_finisher)\.anims$',
  'base_entities\\(man_big\\man_big|man_massive\\man_massive)\.rig$',
  '(a0_003_wa__mantisblade_skeleton|a0_002_wa__monowire_whip)\.rig$',
  'characters\\base_entities\\(woman_base\\woman_base|man_base\\man_base|man_big\\man_big|man_massive\\man_massive)(_ep1)?\.ent$'
) -join '|'
if (!$SkipUnbundle) {
  foreach ($a in $arch) {
    $p = Join-Path (Join-Path $Game 'archive\pc') ($a + '.archive')
    if (!(Test-Path $p)) { "skip (missing) $p"; continue }
    $tag = $a -replace '\\', '_'
    $dst = Join-Path $raw $tag
    "unbundle $a"
    & $wkit unbundle $p -o $dst -r $re | Out-File (Join-Path $logs ('unbundle_' + $tag + '.log')) -Encoding utf8
    if ($LASTEXITCODE -ne 0) { throw "unbundle $a failed $LASTEXITCODE" }
  }
}
# provenance + the copy to use per depot path (last archive in load order)
$use = [ordered]@{}
$rows = @()
foreach ($a in $arch) {
  $tag = $a -replace '\\', '_'
  $dst = Join-Path $raw $tag
  if (!(Test-Path $dst)) { continue }
  foreach ($f in (Get-ChildItem $dst -Recurse -File)) {
    $dp = $f.FullName.Substring($dst.Length + 1)
    $rows += [pscustomobject]@{ Hash = (Get-FileHash $f.FullName -Algorithm SHA256).Hash; Size = $f.Length; Arch = $a; Path = $dp; Full = $f.FullName }
    $use[$dp.ToLower()] = $f.FullName
  }
}
$prov = foreach ($r in ($rows | Sort-Object Path, Arch)) {
  $st = if ($use[$r.Path.ToLower()] -eq $r.Full) { 'used' } else { 'shadowed' }
  '{0}  {1}  {2}  {3}  {4}' -f $r.Hash, $r.Size, $r.Arch, $r.Path, $st
}
$prov | Set-Content (Join-Path $out 'provenance.txt') -Encoding ascii
"raw files: $($rows.Count), depot paths: $($use.Count)"
$files = $use.Values | ForEach-Object { Get-Item $_ }
foreach ($f in ($files | Where-Object { $_.Extension -in @('.anims', '.ent', '.rig') })) {
  & $wkit convert serialize $f.FullName -o $json | Out-File (Join-Path $logs ('json_' + $f.Name + '.log')) -Encoding utf8
}
if ($SkipExport) { 'export skipped'; return }
$env:AnimationExportArgs__incRootMotion = 'true'
$anims = @($files | Where-Object { $_.Extension -eq '.anims' })
for ($i = 0; $i -lt $anims.Count; $i += $Parallel) {
  $batch = $anims[$i..([Math]::Min($i + $Parallel, $anims.Count) - 1)]
  $jobs = foreach ($f in $batch) {
    $a = @('export', ('"' + $f.FullName + '"'), '-o', ('"' + $glb + '"'), '-gp', ('"' + $Game + '"'))
    Start-Process -FilePath $wkit -ArgumentList $a -RedirectStandardOutput (Join-Path $logs ('export_' + $f.Name + '.log')) `
      -RedirectStandardError (Join-Path $logs ('export_' + $f.Name + '.err')) -PassThru -WindowStyle Hidden
  }
  $jobs | Wait-Process -Timeout $BatchTimeoutSec -ErrorAction SilentlyContinue
  foreach ($j in $jobs) { if (!$j.HasExited) { "TIMEOUT kill pid $($j.Id)"; Stop-Process -Id $j.Id -Force } }
  "export batch $([int]($i / $Parallel) + 1): $($batch.Count) files"
}
Get-ChildItem $glb, $json -File | Select-Object Name, Length | Format-Table -AutoSize | Out-String -Width 200
