# Dot-sourced only after the bounded schema reader has loaded $file.
# Local 1H/2H draft: no publisher, registration, or deployment.
$clipPrefix=if($DraftNueMagazine){'utpp_nue_empty_reload_mag_'}else{'utpp_nue_empty_reload_'}
$nodes=[Collections.Generic.List[object]]::new()
$handles=[Collections.Generic.List[object]]::new()
$seen=[Collections.Generic.HashSet[object]]::new([Collections.Generic.ReferenceEqualityComparer]::Instance)
$pending=[Collections.Generic.Stack[object]]::new();$pending.Push($file.RootChunk)
while($pending.Count){
 $v=$pending.Pop();if($null -eq $v){continue}
 if($v -is [WolvenKit.RED4.Types.IRedBaseHandle]){$handles.Add($v);$pending.Push($v.GetValue())}
 elseif($v -is [WolvenKit.RED4.Types.RedBaseClass]){
  if(!$seen.Add($v)){continue};$nodes.Add($v)
  foreach($name in $v.GetPropertyNames()){$pending.Push($v.GetProperty($name))}
 }elseif($v -is [WolvenKit.RED4.Types.IRedArray]){foreach($element in $v){$pending.Push($element)}}
}
function Copy-NueRecord($v){
 if($null -eq $v){return $null}
 if($v -is [WolvenKit.RED4.Types.IRedBaseHandle]){
  $copy=[Activator]::CreateInstance($v.GetType());$copy.SetValue($v.GetValue());return ,$copy
 }
 if($v -is [WolvenKit.RED4.Types.IRedArray]){
  $copy=[Activator]::CreateInstance($v.GetType());foreach($element in $v){$copy.Add((Copy-NueRecord $element))};return ,$copy
 }
 if($v -is [WolvenKit.RED4.Types.RedBaseClass]){
  $copy=[Activator]::CreateInstance($v.GetType())
  foreach($name in $v.GetPropertyNames()){$copy.SetProperty($name,(Copy-NueRecord ($v.GetProperty($name))))}
  return ,$copy
 }
 return ,$v
}
function Set-NueLink($link,$target){$link.GetProperty('node').SetValue($target)}
$template=$null
foreach($node in $nodes){
 if($node.GetType().Name -ne 'animAnimNode_Switch'){continue}
 if($node.GetProperty('inputNodes').Count -ne 2){continue}
 $weight=$node.GetProperty('weightNode').GetProperty('node').GetValue()
 if($null -eq $weight -or $weight.GetType().Name -ne 'animAnimNode_BoolToFloatConverter'){continue}
 $input=$weight.GetProperty('inputNode').GetProperty('node').GetValue()
 if($null -ne $input -and $input.GetType().Name -eq 'animAnimNode_BoolInput'){$template=$node;$converter=$weight;$boolInput=$input;break}
}
if($null -eq $template){throw 'Native bool switch template missing'}
$targets=@($nodes|Where-Object {$_.GetType().Name -eq 'animAnimNode_SkSpeedAnim' -and $_.GetProperty('animation').ToString() -in @('empty_reload','empty_reload_01','empty_reload_02')})
if($targets.Count -ne 6){throw "Unexpected EMPTY leaf count: $($targets.Count)"}
if($IncludeNormalReload){
 $normalTargets=@($nodes|Where-Object {$_.GetType().Name -eq 'animAnimNode_SkSpeedAnim' -and $_.GetProperty('animation').ToString() -in @('reload','reload_01','reload_02')})
 if($normalTargets.Count -ne 3){throw "Unexpected NORMAL leaf count: $($normalTargets.Count)"}
 foreach($clip in @('reload','reload_01','reload_02')){
  if(@($normalTargets|Where-Object {$_.GetProperty('animation').ToString() -eq $clip}).Count -ne 1){throw "NORMAL leaf missing or duplicated: $clip"}
 }
 $targets+=@($normalTargets)
}
$newNodes=[Collections.Generic.List[object]]::new()
function New-NueGate($group,$falseNode,$trueNode){
 $b=Copy-NueRecord $boolInput;$b.SetProperty('group',[WolvenKit.RED4.Types.CName]$group);$b.SetProperty('name',[WolvenKit.RED4.Types.CName]'IsActive')
 $c=Copy-NueRecord $converter;Set-NueLink ($c.GetProperty('inputNode')) $b
 $g=Copy-NueRecord $template
 Set-NueLink ($g.GetProperty('inputNodes')[0]) $falseNode
 Set-NueLink ($g.GetProperty('inputNodes')[1]) $trueNode
 Set-NueLink ($g.GetProperty('weightNode')) $c
 $newNodes.Add($b);$newNodes.Add($c);$newNodes.Add($g);return ,$g
}
$featureList=$file.RootChunk.GetProperty('animFeatures');$initList=$file.RootChunk.GetProperty('nodesToInit')
$beforeInit=$initList.Count;$beforeFeatures=$featureList.Count
$backendHandle=if($file.RootChunk.HasProperty('backendData')){$file.RootChunk.GetProperty('backendData')}else{$null}
$backendData=if($null -ne $backendHandle){$backendHandle.GetValue()}else{$null}
if($null -ne $backendData){
 if(!$backendData.HasProperty('graph')){throw 'Unknown backend shape'}
 $backendGraph=$backendData.GetProperty('graph').GetValue()
 if(![object]::ReferenceEquals($backendGraph,$file.RootChunk)){throw 'Unexpected backend/runtime graph alias'}
}
$featureTemplate=@($featureList|Where-Object {$_.GetProperty('className').ToString() -eq 'gameAnimFeature_TPPRepresentation'})
if($featureTemplate.Count -eq 0 -and $DraftNueMagazine){$featureTemplate=@($featureList[0])}
if($featureTemplate.Count -ne 1){throw 'Feature template missing or ambiguous'}
foreach($group in @('UTPPNueReload','UTPPNueReloadAim','UTPPNueReload2H')){
 if(@($featureList|Where-Object {$_.GetProperty('name').ToString() -eq $group}).Count){throw 'Gate already exists'}
 $feature=Copy-NueRecord $featureTemplate[0];$feature.SetProperty('className',[WolvenKit.RED4.Types.CName]'gameAnimFeature_TPPRepresentation');$feature.SetProperty('name',[WolvenKit.RED4.Types.CName]$group);$featureList.Add($feature)
}
$checks=[Collections.Generic.List[object]]::new();$initTemplate=$initList[0]
foreach($native in $targets){
 $selectedPrefix=$clipPrefix
 if(!$native.GetProperty('animation').ToString().StartsWith('empty_')){
  $selectedPrefix=if($DraftNueMagazine){'utpp_nue_reload_mag_'}else{'utpp_nue_reload_'}
 }
 $ready=Copy-NueRecord $native;$ready.SetProperty('animation',[WolvenKit.RED4.Types.CName]($selectedPrefix+'ready'))
 $aim=Copy-NueRecord $native;$aim.SetProperty('animation',[WolvenKit.RED4.Types.CName]($selectedPrefix+'aim'))
 $ready2H=Copy-NueRecord $native;$ready2H.SetProperty('animation',[WolvenKit.RED4.Types.CName]($selectedPrefix+'2h_ready'))
 $aim2H=Copy-NueRecord $native;$aim2H.SetProperty('animation',[WolvenKit.RED4.Types.CName]($selectedPrefix+'2h_aim'))
 $newNodes.Add($ready);$newNodes.Add($aim);$newNodes.Add($ready2H);$newNodes.Add($aim2H)
 # Select grip inside each Aim branch. OFF remains the original authored1H
 # branch; the outer action gate's OFF still points to the SAME native object.
 $readyChoice=New-NueGate 'UTPPNueReload2H' $ready $ready2H
 $aimChoice=New-NueGate 'UTPPNueReload2H' $aim $aim2H
 $inner=New-NueGate 'UTPPNueReloadAim' $readyChoice $aimChoice
 $outer=New-NueGate 'UTPPNueReload' $native $inner
 $changed=0
 foreach($handle in $handles){if([object]::ReferenceEquals($handle.GetValue(),$native)){$handle.SetValue($outer);$changed++}}
 if(!$changed){throw 'Unreferenced native leaf'}
 if(![object]::ReferenceEquals($outer.GetProperty('inputNodes')[0].GetProperty('node').GetValue(),$native)){throw 'Native fallback changed'}
 # Outer replaces native in the existing init list; native remains initialized.
 foreach($n in @($native)+@($newNodes.ToArray()|Where-Object {![object]::ReferenceEquals($_,$outer)})){
  $handle=Copy-NueRecord $initTemplate;$handle.SetValue($n);$initList.Add($handle)
 }
 $checks.Add(@{nativeAnimation=$native.GetProperty('animation').ToString();clipPrefix=$selectedPrefix;incomingHandles=$changed;nativeFalseBranchSameObject=$true;oneHandReady=($selectedPrefix+'ready');oneHandAim=($selectedPrefix+'aim');twoHandReady=($selectedPrefix+'2h_ready');twoHandAim=($selectedPrefix+'2h_aim')})
 $newNodes.Clear()
}
$expectedAdded=16*$targets.Count
if($initList.Count-$beforeInit -ne $expectedAdded -or $featureList.Count-$beforeFeatures -ne 3){throw 'Unexpected graph additions'}
$script:nueGraphPatchReport=@{editorBackendPresent=($null -ne $backendData);nativeInitCount=$beforeInit;finalInitCount=$initList.Count;clipPrefix=$clipPrefix;includeNormalReload=[bool]$IncludeNormalReload;wrappedLeaves=$checks.ToArray();addedInitNodes=$expectedAdded;privateFeatures=@('UTPPNueReload','UTPPNueReloadAim','UTPPNueReload2H');deployable=$false;runtimeVerified=$false}
