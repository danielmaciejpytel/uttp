// Pure admission and hot-gate rules of the helper recompute path (mm/BodyHelperAdmission.hpp): what may write, for which body.
#include "../src/mm/BodyHelperAdmission.hpp"
#include "../src/mm/GameIntegration.hpp"

#include <algorithm>
#include <array>
#include <cstdio>
#include <string>
#include <vector>

namespace a=utpp::mm::helperadmission;

int main()
{
    int checks=0,failures=0;
    auto check=[&](bool ok,const char* label){++checks;if(!ok){++failures;std::printf("FAIL %s\n",label);}};
    static_assert(a::kApproved.size()==2,"one approved bundle per body");
    const auto& man=a::kApproved[0];
    const auto& woman=a::kApproved[1];
    const std::string manProgram(man.programSha256),manBinding(man.bindingSha256);
    const std::string womanBundleProgram(woman.programSha256),womanBundleBinding(woman.bindingSha256);
    // first (woman-rig, NPC-derived) candidate bundle: hashes of its exact program.ir / binding.dat files
    const std::string womanProgram="d71653b7baa7d1dd623513e41bb16afb305c096834c38e82c6b6a7edb9e7469d";
    const std::string womanBinding="0b0a9d43843cd3f5fb39072c00f272241efcc50c8346a0da63497d345893a609";

    // --- admission
    {
        const auto v=a::Admit(a::Mode::Apply,manProgram,manBinding,man.owned,man.children);
        check(v.decision==a::Decision::LoadApply&&v.approved&&v.body==a::kBodyMan,"exact approved man bundle is admitted for APPLY, body man");
        const auto w=a::Admit(a::Mode::Apply,womanBundleProgram,womanBundleBinding,woman.owned,woman.children);
        check(w.decision==a::Decision::LoadApply&&w.approved&&w.body==a::kBodyWoman,"exact approved woman bundle is admitted for APPLY, body woman");
        const auto wprobe=a::Admit(a::Mode::Probe,womanBundleProgram,womanBundleBinding,woman.owned,woman.children);
        check(wprobe.decision==a::Decision::LoadProbe&&wprobe.approved&&wprobe.body==a::kBodyWoman,"PROBE of the approved woman bundle loads and is marked approved");
        check(man.programSha256!=woman.programSha256&&man.bindingSha256!=woman.bindingSha256&&man.body!=woman.body,"the two entries are different bundles for different bodies");
        const auto probe=a::Admit(a::Mode::Probe,manProgram,manBinding,man.owned,man.children);
        check(probe.decision==a::Decision::LoadProbe&&probe.approved,"PROBE of the approved bundle loads and is marked approved");
        check(a::Admit(a::Mode::Off,"","",0,0).decision==a::Decision::LoadOff,"OFF needs no bundle");
    }
    // --- negative controls: the old NPC bundle and any modified bundle never write
    {
        const auto w=a::Admit(a::Mode::Apply,womanProgram,womanBinding,33,2);
        check(w.decision==a::Decision::RefuseApply&&!w.approved,"old NPC-derived woman candidate: APPLY refused (not an approved bundle)");
        const auto wp=a::Admit(a::Mode::Probe,womanProgram,womanBinding,33,2);
        check(wp.decision==a::Decision::LoadProbe&&!wp.approved&&wp.body==a::kBodyNone,"old NPC-derived woman candidate: PROBE loads as before, approves nothing");
        std::string changed=manProgram;changed[10]=changed[10]=='0'?'1':'0';
        check(a::Admit(a::Mode::Apply,changed,manBinding,man.owned,man.children).decision==a::Decision::RefuseApply,"modified program bytes: APPLY refused");
        changed=manBinding;changed[0]=changed[0]=='0'?'1':'0';
        check(a::Admit(a::Mode::Apply,manProgram,changed,man.owned,man.children).decision==a::Decision::RefuseApply,"modified binding bytes: APPLY refused");
        check(a::Admit(a::Mode::Apply,manProgram,manBinding,man.owned+1,man.children).decision==a::Decision::RefuseApply,"changed owned count: APPLY refused");
        check(a::Admit(a::Mode::Apply,manProgram,manBinding,man.owned,man.children+1).decision==a::Decision::RefuseApply,"changed child count: APPLY refused");
        check(a::Admit(a::Mode::Apply,manBinding,manProgram,man.owned,man.children).decision==a::Decision::RefuseApply,"swapped file hashes: APPLY refused");
        check(a::Admit(a::Mode::Apply,"","",man.owned,man.children).decision==a::Decision::RefuseApply,"empty hashes: APPLY refused");
        const std::array<a::Approved,1> emptyTable{{{a::kBodyMan,"","",68,0,619,143,0xbfad5496d7938ec0ull}}};
        check(a::Admit(a::Mode::Apply,"","",68,0,emptyTable).decision==a::Decision::RefuseApply,"table entry without a hash approves nothing");
        // bundles of the two bodies are not interchangeable: another body's program with this binding (and the reverse) is refused
        check(a::Admit(a::Mode::Apply,womanBundleProgram,manBinding,68,0).decision==a::Decision::RefuseApply,"woman program + man binding: APPLY refused");
        check(a::Admit(a::Mode::Apply,manProgram,womanBundleBinding,68,0).decision==a::Decision::RefuseApply,"man program + woman binding: APPLY refused");
        const auto wm=a::Admit(a::Mode::Apply,manProgram,manBinding,68,0);
        check(wm.body==a::kBodyMan&&!a::BodyMatches(wm.approved,wm.body,a::kBodyWoman),"man bundle is never accepted for the woman body");
        const auto ww=a::Admit(a::Mode::Apply,womanBundleProgram,womanBundleBinding,68,0);
        check(ww.body==a::kBodyWoman&&!a::BodyMatches(ww.approved,ww.body,a::kBodyMan),"woman bundle is never accepted for the man body");
        // the compiled-in table has exactly one entry per body
        int men=0,women=0;for(const auto& e:a::kApproved){if(e.body==a::kBodyMan)++men;if(e.body==a::kBodyWoman)++women;}
        check(men==1&&women==1,"the compiled-in table has exactly one man and one woman entry");
    }
    // --- rig signature: an approved bundle is valid for a rig that has the base and the bundle's bones in the approved layout
    {
        // synthetic rig: bone i has name hash 1000+i and parent i-1 (a chain); the bundle reads/writes bone 1003 (and so its ancestors 1000..1002)
        auto makeRig=[](uint32_t n,std::vector<uint64_t>& names,std::vector<int16_t>& parents)
        { names.resize(n);parents.resize(n);for(uint32_t i=0;i<n;++i){names[i]=1000u+i;parents[i]=int16_t(i?int(i)-1:-1);} };
        std::vector<uint64_t> names;std::vector<int16_t> parents;makeRig(8,names,parents);
        const std::vector<uint64_t> roots{1003};
        const auto sig=a::ComputeRigSignature<int16_t>(names,parents,roots);
        check(sig.ok&&sig.bones==4&&sig.fingerprint!=0,"signature of the chain 1000..1003: 4 bones");
        std::vector<int32_t> parents32(parents.begin(),parents.end());
        check(a::ComputeRigSignature<int32_t>(names,parents32,roots).fingerprint==sig.fingerprint,"signature is the same for int16 and int32 parent storage");
        a::Verdict v;v.approved=true;v.body=a::kBodyMan;v.baseMinBones=8;v.closureBones=sig.bones;v.closureFingerprint=sig.fingerprint;
        auto admits=[&](const a::Verdict& verdict,const std::vector<uint64_t>& n,const std::vector<int16_t>& p)
        { return a::CheckRig<int16_t>(verdict,n,p,roots); };
        check(admits(v,names,parents)==a::RigCheck::Ok,"approved bundle: the exact rig is admitted");
        // equipment / customisation: extra bones anywhere, in any order, never matter
        auto hairNames=names;auto hairParents=parents;
        for(uint32_t i=0;i<26;++i){hairNames.push_back(5000u+i);hairParents.push_back(int16_t(i<7?1:int(hairNames.size())-8));}
        check(admits(v,hairNames,hairParents)==a::RigCheck::Ok,"approved bundle: 26 extra hair-like bones are admitted");
        auto reversedNames=hairNames;std::reverse(reversedNames.begin(),reversedNames.end());
        std::vector<int16_t> reversedParents(hairParents.size());
        {const int32_t last=int32_t(hairParents.size())-1;for(int32_t i=0;i<=last;++i){const int32_t p=hairParents[last-i];reversedParents[i]=int16_t(p<0?-1:last-p);}}
        check(admits(v,reversedNames,reversedParents)==a::RigCheck::Ok,"approved bundle: the same rig in another bone order is admitted (index independent)");
        auto otherLeaf=hairParents;otherLeaf[hairParents.size()-1]=2;
        check(admits(v,hairNames,otherLeaf)==a::RigCheck::Ok,"approved bundle: another parent of a bone outside the bundle is admitted");
        // the bundle's own bones (and their ancestors) must keep the approved layout
        auto reparented=parents;reparented[3]=0;
        check(admits(v,names,reparented)==a::RigCheck::SignatureMismatch,"approved bundle: another parent of a bundle bone is refused");
        auto reparentedAncestor=parents;reparentedAncestor[1]=-1;
        check(admits(v,names,reparentedAncestor)==a::RigCheck::SignatureMismatch,"approved bundle: another parent of an ancestor of a bundle bone is refused");
        auto renamed=names;renamed[3]=9999;
        check(admits(v,renamed,parents)==a::RigCheck::SignatureMismatch,"approved bundle: a bundle bone missing from the rig is refused");
        auto duplicated=names;duplicated[6]=names[2];
        check(admits(v,duplicated,parents)==a::RigCheck::SignatureMismatch,"approved bundle: duplicate bone names are refused");
        auto outOfRange=parents;outOfRange[2]=int16_t(names.size());
        check(admits(v,names,outOfRange)==a::RigCheck::SignatureMismatch,"approved bundle: a parent index outside the rig is refused");
        // the base: fewer bones than the approved minimum (the 294-bone loading rig) is refused even if it has the bundle's bones
        a::Verdict base=v;base.baseMinBones=10;
        check(admits(base,names,parents)==a::RigCheck::TooFewBones,"approved bundle: fewer bones than the approved base are refused");
        check(admits(base,hairNames,hairParents)==a::RigCheck::Ok,"approved bundle: base plus extras is admitted");
        check(a::CheckRig<int16_t>(v,names,std::span<const int16_t>(parents.data(),4),roots)==a::RigCheck::TooFewBones,"approved bundle: names/parents length mismatch is refused");
        a::Verdict zeroBase=v;zeroBase.baseMinBones=0;
        check(admits(zeroBase,names,parents)!=a::RigCheck::Ok,"approved entry with base 0 accepts no rig");
        a::Verdict zeroCount=v;zeroCount.closureBones=0;
        check(admits(zeroCount,names,parents)!=a::RigCheck::Ok,"approved entry with closure size 0 accepts no rig");
        a::Verdict zeroFp=v;zeroFp.closureFingerprint=0;
        check(admits(zeroFp,names,parents)!=a::RigCheck::Ok,"approved entry with fingerprint 0 accepts no rig");
        a::Verdict unapproved;
        check(admits(unapproved,hairNames,hairParents)==a::RigCheck::Ok,"unapproved (PROBE-only) bundle: no rig restriction, it never writes");
        // the table: both bodies need the 619-bone base and the 143-bone closure; the closure is the same for both (the woman rig is the
        // man rig plus hair bones), so the rig can NOT tell the bodies apart - the live pose snapshot does (BodyMatches, every call)
        check(man.baseMinBones==619&&man.closureBones==143&&man.closureFingerprint!=0,"man entry: 619-bone base, 143-bone closure");
        check(woman.baseMinBones==619&&woman.closureBones==143&&woman.closureFingerprint==man.closureFingerprint,"woman entry: same base and the same closure as the man entry");
        const auto manVerdict=a::Admit(a::Mode::Apply,manProgram,manBinding,man.owned,man.children);
        std::vector<uint64_t> loadingNames;std::vector<int16_t> loadingParents;makeRig(294,loadingNames,loadingParents);
        check(a::CheckRig<int16_t>(manVerdict,loadingNames,loadingParents,roots)==a::RigCheck::TooFewBones,"294-bone loading rig is smaller than the approved base");
    }
    // --- owner and body rules
    {
        check(a::OwnerAllows(true,false,false),"Mapping owner (experiment) keeps working for any bundle");
        check(a::OwnerAllows(false,true,true),"FK owner (shipped default) allowed for an approved bundle");
        check(!a::OwnerAllows(false,true,false),"FK owner NOT allowed for an unapproved bundle");
        check(!a::OwnerAllows(false,false,true),"no owner: not allowed");
        check(a::BodyMatches(true,a::kBodyMan,a::kBodyMan),"approved man bundle writes for the man body");
        check(a::BodyMatches(true,a::kBodyWoman,a::kBodyWoman),"approved woman bundle writes for the woman body");
        check(!a::BodyMatches(true,a::kBodyWoman,a::kBodyMan),"approved woman bundle never writes for the man body");
        check(!a::BodyMatches(true,a::kBodyWoman,a::kBodyNone),"approved woman bundle: unknown body (-1): no write");
        check(!a::BodyMatches(true,a::kBodyMan,a::kBodyWoman),"approved man bundle never writes for the woman body");
        check(!a::BodyMatches(true,a::kBodyMan,a::kBodyNone),"unknown body (-1): no write");
        check(a::BodyMatches(false,a::kBodyNone,a::kBodyWoman),"unapproved bundle keeps the legacy woman expectation (PROBE behaviour unchanged)");
        check(!a::BodyMatches(false,a::kBodyNone,a::kBodyMan),"unapproved bundle is never accepted for the man body");
    }
    // --- hot gate: every guard denies, in a fixed order
    {
        a::GateInputs ok;
        check(a::Gate(ok)==a::Reject::None,"all guards satisfied: no rejection");
        auto g=ok;g.ragdollGameOwned=true;
        check(a::Gate(g)==a::Reject::Ragdoll,"ragdoll episode (engine owns the pose): no write");
        g=ok;g.perspectiveOk=false;
        check(a::Gate(g)==a::Reject::Perspective,"first-person / stale perspective window: no write");
        g=ok;g.receiptComplete=false;
        check(a::Gate(g)==a::Reject::Receipt,"incomplete mapping-cycle receipt: no write");
        g=ok;g.fault=true;check(a::Gate(g)==a::Reject::Gate,"fault latched: no write");
        g=ok;g.enabled=false;check(a::Gate(g)==a::Reject::Gate,"disabled: no write");
        g=ok;g.ownerOk=false;check(a::Gate(g)==a::Reject::Gate,"owner not allowed: no write");
        g=ok;g.armed=false;check(a::Gate(g)==a::Reject::Gate,"disarmed: no write");
        g=ok;g.mmActive=false;check(a::Gate(g)==a::Reject::Gate,"motion matching inactive: no write");
        g=ok;g.identityOk=false;check(a::Gate(g)==a::Reject::Identity,"descriptor / rig / count identity mismatch: no write");
        // a missing plan also has no approved bundle, so ownerOk is false too: it must still be counted as identity, not gate
        g=ok;g.planPresent=false;g.ownerOk=false;g.identityOk=false;check(a::Gate(g)==a::Reject::Identity,"missing plan counts as identity, not gate");
        g=ok;g.planPresent=false;g.armed=false;check(a::Gate(g)==a::Reject::Gate,"disarmed beats a missing plan (the real gate reason wins)");
        g=ok;g.bodyOk=false;check(a::Gate(g)==a::Reject::Body,"wrong body: no write");
        g=ok;g.ragdollGameOwned=true;g.perspectiveOk=false;g.receiptComplete=false;g.bodyOk=false;
        check(a::Gate(g)==a::Reject::Ragdoll,"ragdoll wins over every other reason");
    }
    // --- host (v1) publishes carry the body the script reported (mm/GameIntegration.hpp SetKnownBody / StampHostIdentity)
    {
        namespace gi=utpp::mm::integ;
        gi::PoseSnapshot s;
        check(gi::KnownBody()==a::kBodyNone,"no body reported at start: unknown (-1)");
        gi::StampHostIdentity(s);
        check(s.bodyType==a::kBodyNone&&s.source==gi::kPoseSourceHost,"host publish before a body is reported: unknown body, host source");
        check(!a::BodyMatches(true,a::kBodyMan,s.bodyType)&&!a::BodyMatches(true,a::kBodyWoman,s.bodyType),"host publish, body not reported: no approved bundle admitted");
        gi::SetKnownBody(a::kBodyMan);gi::StampHostIdentity(s);
        check(a::BodyMatches(true,a::kBodyMan,s.bodyType)&&!a::BodyMatches(true,a::kBodyWoman,s.bodyType),"host publish, man reported: only the man bundle is admitted");
        gi::SetKnownBody(a::kBodyWoman);gi::StampHostIdentity(s);
        check(a::BodyMatches(true,a::kBodyWoman,s.bodyType)&&!a::BodyMatches(true,a::kBodyMan,s.bodyType),"host publish, body changed to woman: only the woman bundle is admitted");
        s.source=gi::kPoseSourceDriver;s.bodyType=a::kBodyMan;gi::StampHostIdentity(s);
        check(s.bodyType==a::kBodyWoman&&s.source==gi::kPoseSourceHost,"the stamp replaces the identity a reused snapshot kept from a driver pose");
        gi::SetKnownBody(2);gi::StampHostIdentity(s);
        check(s.bodyType==a::kBodyNone&&!a::BodyMatches(true,a::kBodyMan,s.bodyType)&&!a::BodyMatches(true,a::kBodyWoman,s.bodyType),
              "an invalid reported body is unknown (-1): no bundle admitted");
        gi::SetKnownBody(a::kBodyNone);
    }
    std::printf("%d checks, %d failures\n",checks,failures);
    return failures?1:0;
}
