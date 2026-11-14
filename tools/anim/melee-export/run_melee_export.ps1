# TPP (full body) melee export: re-solved melee clips -> utpp\melee\utpp_<set>.anims -> utpp_melee.archive (2026-11-09).
#   powershell -ExecutionPolicy Bypass -File run_melee_export.ps1 -Input <clips dir> -Rig woman|man|both -Sets <a,b|all> -Out <new dir>
#              [-Suffix v4] [-Vanilla <frozen vanilla dir>] [-Registration <registration.json>] [-Identity] [-NoPack]
#              [-ExtraStaged <dir>] [-Only clip,clip] [-NoSimdCanon]
# <clips dir> holds <set>__<clip>.<Suffix>.npz (+ .info.json); the files of the selected sets are copied into <Out>\input first and only that
# copy is read. Per set: build_melee_set.py (GLB, anim buffer frame) -> animset_subset.py (vanilla set reduced to the built clips) -> WolvenKit
# bridge import -> patch_melee_json.py (vanilla motion extraction / metadata) -> deserialize into <Out>\utpp_melee\utpp\melee -> serialize +
# export with and without root motion -> verify_melee_set.py. Clips that fail a gate (round trip, or a SIMD buffer the importer gave a wrong
# frame count: patch exit 4) are collected and the set is rebuilt ONCE without them (--reject-json; they stay vanilla; gates unchanged;
# the first attempt is kept in <Out>\attempt1, the rejected clips in manifest set_details). A set that still fails is NOT packed (it stays
# vanilla in game). Then pack, unbundle, hash compare, manifest. Nothing is installed; the game folder is only read (rig lookup).
# UTPP-only clip names (melee_export_lib.UTPP_CLIP_TEMPLATES, e.g. parried_left_combo_01): before the sets run, aug_vanilla.py adds a clone of the
# template clip (parried_left) to a copy of the vanilla set data, <Out>\vanilla-aug, which is then used as -Vanilla (the manifest keeps both);
# without such input clips the vanilla dir is used as given. Unknown names that are not allowlisted are skipped as before.
# -ExtraStaged <dir>: the *.anims of that dir are copied byte for byte into the staging folder (sets built elsewhere, e.g. the installed fists
# sets, so one utpp_melee.archive carries them; logged with SHA-256). -Only: build only those clips of every selected set (test runs).
# -NoSimdCanon: TEST ONLY, negative control of test_melee_export.ps1 (the pre-2026-11-10 sign handling of SIMD clips).
param(
  [Parameter(Mandatory = $true)][Alias('Input')][string]$InputDir,
  [ValidateSet('woman', 'man', 'both')][string]$Rig = 'both',
  [string]$Sets = 'all',
  [Parameter(Mandatory = $true)][string]$Out,
  [string]$Suffix = 'v4',
  [string]$Vanilla = '',
  [string]$Registration = '',
  [string]$Bridge = '',
  [switch]$Identity,
  [switch]$NoPack,
  [string]$ExtraStaged = '',
  [string]$Only = '',
  [switch]$NoSimdCanon
)
. (Join-Path $PSScriptRoot '..\..\utpp_paths.ps1')
$root = Get-UtppProjectRoot
if (-not $Vanilla) { $Vanilla = Join-Path $root 'Build\work\melee-export-20261109\vanilla-frozen' }
if (-not $Registration) { $Registration = Join-Path $root 'Build\work\melee-export-20261109\evidence\registration.json' }
if (-not $Bridge) { $Bridge = Join-Path $root 'Build\work\vehicle-entry-car\bridge-bin\WkitBridge.exe' }
$ErrorActionPreference = 'Stop'
$tools = Split-Path -Parent $MyInvocation.MyCommand.Path
$ve = Join-Path $root 'Source\tools\anim\vehicle-entry'
$py = Join-Path $root 'Tools\ai-motion\envs\kimodo\Scripts\python.exe'
$wkit = Join-Path $root 'Tools\WolvenKit.CLI\WolvenKit.CLI.exe'
$Out = [IO.Path]::GetFullPath($Out)
if (!$Out.StartsWith($root + '\Build\work\')) { throw "Out must be under Build\work: $Out" }
if (Test-Path $Out) { throw "output exists, choose a new -Out: $Out" }
$all = @('default_melee_weapon', 'pma_default_melee_weapon', 'pwa_fists', 'pma_fists', 'pwa_katana', 'pma_katana', 'pwa_knife', 'pma_knife',
         'pwa_one_handed_blunt', 'pma_one_handed_blunt', 'pwa_two_handed_blunt', 'pma_two_handed_blunt', 'pwa_two_handed_hammer', 'pma_two_handed_hammer')
# known but not in 'all' (melee_export_lib.EXTRA_SETS; the default vanilla-frozen dir has no json for them: pass -Vanilla with them)
$known = $all + @('pwa_baton', 'pma_baton', 'pwa_strongarms_melee_fists', 'pma_strongarms_melee_fists', 'pwa_mantisblade', 'pma_mantisblade')
$sel = if ($Sets -eq 'all') { $all } else { $Sets -split ',' | ForEach-Object { $_.Trim() } }
foreach ($s in $sel) { if ($known -notcontains $s) { throw "unknown set $s" } }
$sel = @($sel | Where-Object { $Rig -eq 'both' -or (($Rig -eq 'man') -eq $_.StartsWith('pma_')) })
if (!$sel) { throw 'no set selected' }
$stage = Join-Path $Out 'utpp_melee\utpp\melee'
foreach ($d in @("$Out\input", "$Out\glb", "$Out\base-json", "$Out\base", "$Out\import-json", "$Out\patched-json", "$Out\final-json", "$Out\roundtrip", "$Out\roundtrip-nort", "$Out\reports", $stage)) {
  New-Item -ItemType Directory -Force $d | Out-Null
}
$log = Join-Path $Out 'run.log'
function Log($m) { $l = '{0:HH:mm:ss} {1}' -f (Get-Date), $m; $l | Tee-Object -FilePath $log -Append }
function Check($what) { if ($LASTEXITCODE -ne 0) { throw "$what failed ($LASTEXITCODE)" } }
Log "run_melee_export Input=$InputDir Rig=$Rig Sets=$($sel -join ',') Suffix=$Suffix Identity=$Identity Vanilla=$Vanilla Registration=$Registration"
# 1 frozen copy of the inputs
if (!$Identity) {
  $n = 0
  foreach ($s in $sel) {
    Get-ChildItem -LiteralPath $InputDir -File | Where-Object { $_.Name -like "${s}__*.$Suffix.npz" -or $_.Name -like "${s}__*.$Suffix.info.json" } | ForEach-Object {
      Copy-Item -LiteralPath $_.FullName (Join-Path "$Out\input" $_.Name); $n++ }
  }
  Get-ChildItem "$Out\input" -File | ForEach-Object { '{0}  {1}  {2}' -f (Get-FileHash $_.FullName -Algorithm SHA256).Hash, $_.Length, $_.Name } |
    Set-Content (Join-Path $Out 'input-sha256.txt') -Encoding ascii
  Log "inputs copied: $n files"
}
$VanillaGiven = $Vanilla
if (!$Identity) {
  # UTPP-only clip names: clones of the template clips in <Out>\vanilla-aug (aug_vanilla.py; exit 2 = an allowlisted name has no template in its set)
  & $py -I -B "$tools\aug_vanilla.py" --vanilla $Vanilla --input "$Out\input" --suffix $Suffix --sets ($sel -join ',') --out "$Out\vanilla-aug" |
    Tee-Object -FilePath $log -Append | Out-Host
  if ($LASTEXITCODE -ne 0) { throw "aug_vanilla failed ($LASTEXITCODE)" }
  $augLine = Get-Content $log | Where-Object { $_ -like 'AUG_CLONES *' } | Select-Object -Last 1
  if ($augLine -and [int]($augLine -split ' ')[1] -gt 0) { $Vanilla = "$Out\vanilla-aug"; Log "UTPP-only clip names: vanilla copy with clones = $Vanilla" }
  else { Remove-Item -LiteralPath "$Out\vanilla-aug" -Recurse -Force }
}
$results = [ordered]@{}
$details = [ordered]@{}
# One attempt of one set: build -> subset -> import -> patch -> final -> round trip -> verify.
# Returns @{ status = OK | PATCH_UNPATCHABLE | VERIFY_FAIL; bad = [ordered]{clip: reason} }; throws on tool errors.
function Invoke-SetAttempt([string]$s, [string]$rejectJson) {
  $u = "utpp_$s"
  $bargs = @('-I', "$tools\build_melee_set.py", '--set', $s, '--input', "$Out\input", '--suffix', $Suffix, '--vanilla-glb', "$Vanilla\glb\$s.anims.glb",
             '--vanilla-json', "$Vanilla\json\$s.anims.json", '--vanilla-glb-nort', "$Vanilla\glb-nort\$s.anims.glb", '--out-glb', "$Out\glb\$u.anims.glb",
             '--report', "$Out\reports\build-$s.json")
  if ($Registration) { $bargs += @('--exclude-json', $Registration) }
  if ($Identity) { $bargs += '--identity' }
  if ($rejectJson) { $bargs += @('--reject-json', $rejectJson) }
  if ($Only) { $bargs += @('--only', $Only) }
  if ($NoSimdCanon) { $bargs += '--no-simd-canon' }
  & $py @bargs | Tee-Object -FilePath $log -Append | Out-Host
  if ($LASTEXITCODE -ne 0) { throw "build $s exit $LASTEXITCODE" }
  $br = Get-Content "$Out\reports\build-$s.json" -Raw | ConvertFrom-Json
  $keep = @(); foreach ($c in $br.built) { $keep += @('--keep', $c) }
  & $py -I "$ve\animset_subset.py" "$Vanilla\json\$s.anims.json" "$Out\base-json\$u.anims.json" @keep | Out-Null; Check "subset $s"
  & $wkit convert deserialize "$Out\base-json\$u.anims.json" -o "$Out\base" | Out-Null; Check "deserialize base $s"
  & $Bridge import "$Out\glb\$u.anims.glb" "$Out\base\$u.anims" "$Out\import-$s" | Select-String 'ANIMS_IMPORT_EXIT|rror' | ForEach-Object { Log "  bridge: $_" | Out-Host }
  Check "import $s"
  & $wkit convert serialize "$Out\import-$s\$u.anims" -o "$Out\import-json" | Out-Null; Check "serialize import $s"
  & $py -I "$tools\patch_melee_json.py" "$Out\import-json\$u.anims.json" "$Vanilla\json\$s.anims.json" "$Out\patched-json\$u.anims.json" --report "$Out\reports\patch-$s.json" |
    Tee-Object -FilePath $log -Append | Out-Host
  $pexit = $LASTEXITCODE
  if ($pexit -ne 0 -and $pexit -ne 4) { throw "patch $s failed ($pexit)" }
  $bad = [ordered]@{}
  if ($pexit -eq 4) {
    # a SIMD buffer whose frame count the importer got wrong (left as imported, so the file stays readable): rejected; the attempt goes on
    # to the verification so that every failing clip of this attempt is known for the single retry
    $pr = Get-Content "$Out\reports\patch-$s.json" -Raw | ConvertFrom-Json
    foreach ($p in $pr.unpatchable.PSObject.Properties) { $bad[$p.Name] = "import: $($p.Value)" }
  }
  & $wkit convert deserialize "$Out\patched-json\$u.anims.json" -o $stage | Out-Null; Check "deserialize final $s"
  & $wkit convert serialize "$stage\$u.anims" -o "$Out\final-json" | Out-Null; Check "serialize final $s"
  & "$ve\export_glb.ps1" -Files "$stage\$u.anims" -OutDir "$Out\roundtrip" -RootMotion | Out-Null; Check "roundtrip export $s"
  & "$ve\export_glb.ps1" -Files "$stage\$u.anims" -OutDir "$Out\roundtrip-nort" | Out-Null; Check "roundtrip export (no root motion) $s"
  & $py -I "$tools\verify_melee_set.py" --set $s --built-glb "$Out\glb\$u.anims.glb" --roundtrip-glb "$Out\roundtrip\$u.anims.glb" --vanilla-glb "$Vanilla\glb\$s.anims.glb" `
    --vanilla-json "$Vanilla\json\$s.anims.json" --final-json "$Out\final-json\$u.anims.json" --roundtrip-nort-glb "$Out\roundtrip-nort\$u.anims.glb" --vanilla-glb-nort "$Vanilla\glb-nort\$s.anims.glb" --report "$Out\reports\verify-$s.json" | Tee-Object -FilePath $log -Append | Out-Host
  $vexit = $LASTEXITCODE
  if ($vexit -ne 0) {
    $vr = Get-Content "$Out\reports\verify-$s.json" -Raw | ConvertFrom-Json
    foreach ($p in $vr.clips.PSObject.Properties) { if (-not $p.Value.ok -and -not $bad.Contains($p.Name)) { $bad[$p.Name] = 'round trip: ' + ($p.Value.why -join '; ') } }
  }
  if ($pexit -eq 4) { return @{ status = 'PATCH_UNPATCHABLE'; bad = $bad } }
  if ($vexit -ne 0) { return @{ status = 'VERIFY_FAIL'; bad = $bad } }
  return @{ status = 'OK'; bad = $bad }
}
# the files of a failed first attempt go to <Out>\attempt1\<folder>__<file> (kept as evidence; nothing of it is packed)
function Save-Attempt1([string]$s) {
  $u = "utpp_$s"; $a1 = Join-Path $Out 'attempt1'; New-Item -ItemType Directory -Force $a1 | Out-Null
  foreach ($f in @("$stage\$u.anims", "$Out\glb\$u.anims.glb", "$Out\base-json\$u.anims.json", "$Out\base\$u.anims", "$Out\import-json\$u.anims.json",
                   "$Out\patched-json\$u.anims.json", "$Out\final-json\$u.anims.json", "$Out\roundtrip\$u.anims.glb", "$Out\roundtrip-nort\$u.anims.glb",
                   "$Out\reports\build-$s.json", "$Out\reports\patch-$s.json", "$Out\reports\verify-$s.json")) {
    if (Test-Path -LiteralPath $f) { Move-Item -LiteralPath $f (Join-Path $a1 ((Split-Path (Split-Path $f -Parent) -Leaf) + '__' + (Split-Path $f -Leaf))) }
  }
  if (Test-Path "$Out\import-$s") { Move-Item "$Out\import-$s" (Join-Path $a1 "import-$s") }
}
foreach ($s in $sel) {
  $u = "utpp_$s"
  $rej = [ordered]@{}; $attempts = 0; $first = $null
  try {
    $r = Invoke-SetAttempt $s ''
    $attempts = 1
    if ($r.status -ne 'OK' -and $r.bad.Count -gt 0) {
      # per-clip rejection: rebuild the set ONCE without the clips that failed a gate (they stay vanilla); gates unchanged
      $first = $r.status
      foreach ($k in $r.bad.Keys) { $rej[$k] = $r.bad[$k] }
      Log "SET $s attempt 1 -> $($r.status); rebuilding without: $(@($rej.Keys) -join ', ')"
      Save-Attempt1 $s
      $rj = "$Out\reports\reject-$s.json"
      $rej | ConvertTo-Json -Depth 3 | Set-Content $rj -Encoding utf8
      $r = Invoke-SetAttempt $s $rj
      $attempts = 2
    }
    if ($r.status -eq 'OK') { $results[$s] = 'OK' } else {
      if (Test-Path "$stage\$u.anims") { Move-Item "$stage\$u.anims" (Join-Path $Out "rejected-$u.anims") }
      $results[$s] = "$($r.status) (not packed): $(@($r.bad.Keys) -join ', ')"
    }
  } catch {
    $results[$s] = "ERROR $($_.Exception.Message)"
    if (Test-Path "$stage\$u.anims") { Move-Item "$stage\$u.anims" (Join-Path $Out "rejected-$u.anims") }
  }
  $details[$s] = [ordered]@{ result = $results[$s]; attempts = $attempts; attempt1 = $first; rejected_clips = $rej }
  Log "SET $s -> $($results[$s])$(if ($rej.Count) { " (attempt $attempts; rejected clips, stay vanilla: $(@($rej.Keys) -join ', '))" })"
}
if ($ExtraStaged) {
  Get-ChildItem -LiteralPath $ExtraStaged -Filter *.anims -File | ForEach-Object {
    $t = Join-Path $stage $_.Name
    if (Test-Path -LiteralPath $t) { throw "extra staged file collides with a built set: $($_.Name)" }
    Copy-Item -LiteralPath $_.FullName $t
    Log "EXTRA STAGED (byte copy, not rebuilt): $($_.Name) sha256 $((Get-FileHash $t -Algorithm SHA256).Hash) from $($_.FullName)" }
}
$staged = @(Get-ChildItem $stage -File)
$man = [ordered]@{ created = (Get-Date).ToString('s'); input = $InputDir; suffix = $Suffix; identity = [bool]$Identity; rig = $Rig; sets = $results; set_details = $details;
  vanilla = $Vanilla; vanilla_given = $VanillaGiven; extra_staged = $ExtraStaged; only = $Only; registration = $Registration; registration_sha256 = $(if ($Registration -and (Test-Path $Registration)) { (Get-FileHash $Registration -Algorithm SHA256).Hash } else { $null });
  tools = @(Get-ChildItem $tools -File -Include *.py, *.ps1 -Recurse | ForEach-Object { [ordered]@{ file = $_.Name; sha256 = (Get-FileHash $_.FullName -Algorithm SHA256).Hash } });
  staged = @($staged | ForEach-Object { [ordered]@{ path = 'utpp\melee\' + $_.Name; size = $_.Length; sha256 = (Get-FileHash $_.FullName -Algorithm SHA256).Hash } }) }
if (!$NoPack -and $staged.Count -gt 0) {
  & $wkit pack (Join-Path $Out 'utpp_melee') -o "$Out\packed" | Out-Null; Check 'pack'
  $arch = Join-Path $Out 'packed\utpp_melee.archive'
  & $wkit unbundle $arch -o "$Out\unpacked" | Out-Null; Check 'unbundle'
  $bad = 0; $cmp = @()
  foreach ($f in $staged) {
    $p = Join-Path "$Out\unpacked\utpp\melee" $f.Name
    $same = (Test-Path $p) -and ((Get-FileHash $p -Algorithm SHA256).Hash -eq (Get-FileHash $f.FullName -Algorithm SHA256).Hash)
    if (!$same) { $bad++ }
    $cmp += [ordered]@{ file = $f.Name; packed_equals_unpacked = $same }
  }
  $extra = @(Get-ChildItem "$Out\unpacked" -Recurse -File | Where-Object { $staged.Name -notcontains $_.Name })
  $man.archive = [ordered]@{ path = $arch; size = (Get-Item $arch).Length; sha256 = (Get-FileHash $arch -Algorithm SHA256).Hash; files = $staged.Count;
    unbundle_compare = $cmp; mismatches = $bad; unexpected_files = @($extra | ForEach-Object { $_.FullName.Substring("$Out\unpacked".Length + 1) }) }
  Log "PACK $arch sha256 $($man.archive.sha256) size $($man.archive.size) files $($staged.Count) mismatches $bad unexpected $($extra.Count)"
  if ($bad -or $extra.Count) { $man.archive.ok = $false } else { $man.archive.ok = $true }
}
$man | ConvertTo-Json -Depth 8 | Set-Content (Join-Path $Out 'manifest.json') -Encoding utf8
Log ("RESULT " + (($results.GetEnumerator() | ForEach-Object { "$($_.Key)=$($_.Value)" }) -join '; '))
Log "RUN=$Out"
if (@($results.Values | Where-Object { $_ -ne 'OK' }).Count) { exit 1 }
exit 0
