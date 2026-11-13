# Offline verifier only. No game writes. Run in a fresh PowerShell process.
param([string]$OutputRelative='Build/test-data/ReloadPhaseProbe/graph-schema-tool-check.animgraph',
      [string]$ReportRelative='Research/inventory/player-ranged-schema-tool-check-20261016.json',
      [switch]$DraftNueReload,[switch]$DraftNueMagazine,[switch]$IncludeNormalReload,[switch]$TwoHandReload,
      [string]$SchemaRelative='Build/test-data/ReloadPhaseProbe/retail-player_ranged-schema.json')
$ErrorActionPreference='Stop'
$projectRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../../..'))
foreach($relative in @($OutputRelative,$ReportRelative,$SchemaRelative)){
 if([IO.Path]::IsPathRooted($relative) -or ![IO.Path]::GetFullPath((Join-Path $projectRoot $relative)).StartsWith($projectRoot+'\')){throw 'Path outside project'}
}
$schema=Get-Content (Join-Path $projectRoot $SchemaRelative) -Raw|ConvertFrom-Json -AsHashtable
if(![IO.Path]::GetFullPath($schema.source).StartsWith($projectRoot+'\')){throw 'Source outside project'}
if($DraftNueMagazine -and [IO.Path]::GetFileName($schema.source) -ne 'weapon.animgraph'){throw 'Magazine mode requires weapon graph'}
if($DraftNueReload -and [IO.Path]::GetFileName($schema.source) -ne 'player_ranged.animgraph'){throw 'Body mode requires player graph'}
if((Get-FileHash $schema.source -Algorithm SHA256).Hash -ine $schema.sourceSHA256){throw 'Schema source hash mismatch'}
$base=(Join-Path $projectRoot 'Tools\WolvenKit.CLI')+'\'
[AppContext]::SetData('APP_CONTEXT_BASE_DIRECTORY',$base)
$a=[Reflection.Assembly]::LoadFrom($base+'WolvenKit.CLI.dll')
$h=$a.GetType('WolvenKit.CLI.GenericHost').GetMethod('CreateHostBuilder').Invoke($null,@(,[string[]]@())).Build()
try {
$r=[Reflection.Assembly]::LoadFrom($base+'WolvenKit.RED4.dll')
$assemblyName=[Reflection.AssemblyName]::new('UTPPGraphSchemaProbe')
$dyn=[Reflection.Emit.AssemblyBuilder]::DefineDynamicAssembly($assemblyName,[Reflection.Emit.AssemblyBuilderAccess]::Run)
$module=$dyn.DefineDynamicModule('UTPPGraphSchemaProbe')
$allNames=@($schema.classNames)+@($schema.inlineClasses)
$known=[WolvenKit.RED4.Types.RedReflection]::GetTypes()
$requirements=$schema.parents
foreach($typeName in ($allNames|Sort-Object -Unique)) {
 if($known.ContainsKey($typeName)){continue}
 $parent=[WolvenKit.RED4.Types.RedBaseClass]
 foreach($required in $requirements[$typeName]) {
  if($required -eq $typeName){continue}
  if(!$known.ContainsKey($required)){throw "Unresolved parent $required for $typeName"}
  $candidate=$known[$required]
  if($parent.IsAssignableFrom($candidate)){$parent=$candidate}
  elseif(!$candidate.IsAssignableFrom($parent)){throw 'Incompatible parent requirements'}
 }
 $tb=$module.DefineType($typeName,[Reflection.TypeAttributes]::Public,$parent)
 [void]$tb.DefineDefaultConstructor([Reflection.MethodAttributes]::Public)
 $created=$tb.CreateType()
 if(![WolvenKit.RED4.Types.RedReflection]::AddRedType($created)){throw "Could not register $typeName"}
 $known.Add($typeName,$created)
}
$enumCache=[WolvenKit.RED4.Types.RedReflection].GetField('s_redEnumCache',[Reflection.BindingFlags]'NonPublic,Static').GetValue($null)
foreach($enumName in $schema.enums.Keys){
 $values=@($schema.enums[$enumName])
 if($enumCache.ContainsKey($enumName)){
  $values+=@([Enum]::GetNames($enumCache[$enumName].Type))
  if(!$enumCache.Remove($enumName)){throw 'Enum replacement failed'}
 }
 $eb=$module.DefineEnum($enumName,[Reflection.TypeAttributes]::Public,[int]);$i=0
 foreach($value in ($values|Sort-Object -Unique)){[void]$eb.DefineLiteral($value,$i);$i++}
 if(![WolvenKit.RED4.Types.RedReflection]::AddEnumType($eb.CreateType())){throw "Enum registration failed: $enumName"}
}
$inputStream=[IO.File]::OpenRead($schema.source)
$reader=[Activator]::CreateInstance($r.GetType('WolvenKit.RED4.Archive.IO.CR2WReader'),@($inputStream))
$script:unknowns=[Collections.Generic.HashSet[string]]::new()
$handler=[WolvenKit.RED4.Types.ParsingErrorEventHandler]{param($errorInfo)
 if($errorInfo -is [WolvenKit.RED4.Types.UnknownTypeEventArgs]){[void]$script:unknowns.Add($errorInfo.RedName)}
 return $false
}
$reader.add_ParsingError($handler)
$file=$null
$code=$reader.ReadFile([ref]$file,$false)
'READ_RESULT='+$code
'UNKNOWNS='+($script:unknowns -join ',');if($script:unknowns.Count){throw 'Unhandled source type'}
if($code.ToString() -ne 'NoError'){throw 'Read failed'}
$out=Join-Path $projectRoot $OutputRelative
if((Test-Path $out) -or (Test-Path (Join-Path $projectRoot $ReportRelative))){throw 'Output or report exists'}
if($DraftNueReload -and $DraftNueMagazine){throw 'Choose body OR magazine graph'}
if($IncludeNormalReload -and !($DraftNueReload -or $DraftNueMagazine)){throw 'Normal routing requires a Nue draft graph'}
if($TwoHandReload -and !($DraftNueReload -or $DraftNueMagazine)){throw 'Two-hand routing requires a Nue draft graph'}
if($DraftNueReload -or $DraftNueMagazine){
 $patcher=if($TwoHandReload){'patch_nue_reload_graph_twohand.ps1'}else{'patch_nue_reload_graph_objects.ps1'}
 . (Join-Path $PSScriptRoot $patcher)
}
$os=[IO.File]::Create($out)
try {$writer=[Activator]::CreateInstance($r.GetType('WolvenKit.RED4.Archive.IO.CR2WWriter'),@($os));$writer.WriteFile($file)}finally{$os.Dispose()}
'OUTPUT_BYTES='+(Get-Item $out).Length

$secondStream=[IO.File]::OpenRead($out)
$secondReader=[Activator]::CreateInstance($r.GetType('WolvenKit.RED4.Archive.IO.CR2WReader'),@($secondStream))
$secondFile=$null
$secondReader.add_ParsingError($handler)
$secondCode=$secondReader.ReadFile([ref]$secondFile,$false)
if($script:unknowns.Count){throw 'Unhandled roundtrip type'}
if($secondCode.ToString() -ne 'NoError'){throw 'Second read failed'}
$forward=[Collections.Generic.Dictionary[object,object]]::new([Collections.Generic.ReferenceEqualityComparer]::Instance)
$reverse=[Collections.Generic.Dictionary[object,object]]::new([Collections.Generic.ReferenceEqualityComparer]::Instance)
$stack=[Collections.Generic.Stack[object]]::new();$stack.Push(@($file.RootChunk,$secondFile.RootChunk,'root'))
$checked=0;$classCount=0;$buffers=0;$failures=[Collections.Generic.List[string]]::new()
while($stack.Count) {
 $pair=$stack.Pop();$left=$pair[0];$right=$pair[1];$path=$pair[2];$checked++
 if($null -eq $left -or $null -eq $right){if($null -ne $left -or $null -ne $right){$failures.Add($path+': null')};continue}
 if($left.GetType() -ne $right.GetType()){$failures.Add($path+': type');continue}
 if($left -is [WolvenKit.RED4.Types.RedBaseClass]) {
  if($forward.ContainsKey($left)){if(![object]::ReferenceEquals($forward[$left],$right)){$failures.Add($path+': alias')};continue}
  if($reverse.ContainsKey($right)){$failures.Add($path+': reverse alias');continue}
  $forward.Add($left,$right);$reverse.Add($right,$left);$classCount++
  $lp=@($left.GetPropertyNames()|Sort-Object);$rp=@($right.GetPropertyNames()|Sort-Object)
  if(($lp -join '|') -cne ($rp -join '|')){$failures.Add($path+': properties');continue}
  foreach($name in $lp){$stack.Push(@($left.GetProperty($name),$right.GetProperty($name),$path+'.'+$name))}
 } elseif($left -is [WolvenKit.RED4.Types.IRedBaseHandle]) {
  $stack.Push(@($left.GetValue(),$right.GetValue(),$path+'->'))
 } elseif($left -is [WolvenKit.RED4.Types.IRedArray]) {
  if($left.Count -ne $right.Count){$failures.Add($path+': array count');continue}
  for($i=0;$i -lt $left.Count;$i++){$stack.Push(@($left[$i],$right[$i],$path+'['+$i+']'))}
 } elseif($left -is [WolvenKit.RED4.Types.IRedBufferPointer]) {
  $lb=$left.GetValue();$rb=$right.GetValue();$buffers++
  if($lb.Flags -ne $rb.Flags -or [Convert]::ToBase64String($lb.GetBytes()) -cne [Convert]::ToBase64String($rb.GetBytes())){$failures.Add($path+': raw buffer')}
 } elseif(!$left.Equals($right)) {$failures.Add($path+': scalar '+$left.ToString()+' != '+$right.ToString())}
}
$report=@{patch=$script:nueGraphPatchReport;sourceSHA256=$schema.sourceSHA256;outputSHA256=(Get-FileHash $out -Algorithm SHA256).Hash;outputBytes=(Get-Item $out).Length;checkedValues=$checked;reachableClasses=$classCount;buffers=$buffers;failures=@($failures.ToArray());runtimeVerified=$false;source=$schema.source;output=$out}
$report|ConvertTo-Json -Depth 6|Set-Content (Join-Path $projectRoot $ReportRelative)
$report|ConvertTo-Json -Depth 6
$secondReader.Dispose();$secondStream.Dispose()
if($failures.Count){throw 'Object graph differs'}
$reader.Dispose();$inputStream.Dispose()
} finally {$h.Dispose()}
