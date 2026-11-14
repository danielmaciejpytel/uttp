#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#include <bcrypt.h>

#include "BodyHelperRuntime.hpp"
#include "CommonUtil.hpp"
#include "EarlyPoseRuntime.hpp"
#include "State.hpp"
#include "ragdoll/RagdollEpisode.hpp"
#include "mm/BodyDeformerProgramFile.hpp"
#include "mm/BodyHelperBindingFile.hpp"
#include "mm/BodyHelperAdmission.hpp"
#include "mm/BodyHelperCommit.hpp"
#include "mm/BodyDeformerTrace.hpp"
#include "mm/CoreWriteRecord.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string_view>
#include <vector>

namespace utpp::bodyhelperruntime
{
namespace
{
constexpr size_t kMaxBundleFile=2u*1024u*1024u;
enum class Mode:uint32_t { Off,Probe,Apply };
struct Bundle
{
    std::unique_ptr<mm::deformoffline::Program> program;
    std::array<uint64_t,mm::deformoffline::kMaxNodes> programNames{};
    std::string identity,programFileHash;
    mm::helperbinding::Data binding;
    bool approved=false;               // exact bundle compiled into the DLL (mm/BodyHelperAdmission.hpp)
    int32_t body=mm::helperadmission::kBodyNone;
    mm::helperadmission::Verdict verdict;  // also carries the rig requirement (minimum bones + closure signature) of the approval
    std::vector<uint64_t> bones;       // every bone the plan reads or writes by name (binding CORE, OWNED, CHILD, ANCHOR): the rig signature roots
    const char* label="";              // folder the bundle was loaded from ("man" / "woman")
};
// One bundle per body (skin_helpers\man, skin_helpers\woman), each admitted on its own.
constexpr size_t kMaxBundles=2;
struct Slot
{
    std::shared_ptr<const Bundle> bundle;
    mm::bodyhelpercandidate::Plan plan;
};
struct Active
{
    // A plan for every loaded bundle the armed rig fits. The man and the woman rig have the same closure, so usually both are
    // here and Evaluate picks the one whose body equals the live pose snapshot's body.
    std::array<Slot,kMaxBundles> slots;
    uint32_t slotCount=0;
    bool anyApproved=false;
    uint64_t rig=0;
    uint32_t count=0,requiredCount=0;
    // All-ones availability of the rig, filled once at arm and only read on the hot path (every hook thread shares it).
    std::array<uint8_t,mm::bodyhelpercandidate::kMaxMergedBones> available{};
};

std::atomic<Mode> g_mode{Mode::Off};
std::atomic<bool> g_loaded{false};
std::atomic<Active*> g_active{nullptr};
std::mutex g_coldMutex;
std::vector<std::shared_ptr<const Bundle>> g_bundles;
// Retain every published plan/bundle until DLL shutdown; callbacks only load
// g_active and never participate in reclamation or locking.
std::vector<std::unique_ptr<Active>> g_retained;
std::string g_status="not loaded";
std::atomic<uint64_t> g_evaluated{0},g_rejected{0},g_buildFailed{0},g_proposed{0},g_proposedRecords{0};
std::atomic<uint64_t> g_applied{0},g_appliedRecords{0},g_commitRejected{0},g_jumpRejected{0};
std::array<std::atomic<uint64_t>,size_t(mm::helperadmission::Reject::Count)> g_rejectReason{};
static_assert(kWriterRows==size_t(mm::corewrite::kWriterCount)+1&&kNoWriter==size_t(mm::corewrite::kWriterCount));
static_assert(size_t(Outcome::Body)==size_t(mm::helperadmission::Reject::Body)&&size_t(Outcome::Ragdoll)==size_t(mm::helperadmission::Reject::Ragdoll));
std::atomic<uint64_t> g_covCalls{0};
std::array<std::array<std::atomic<uint64_t>,size_t(Outcome::Count)>,kWriterRows> g_cov{};
// One player FK call: its outcome under every writer of its core write record (or under kNoWriter).
void Tally(uint32_t record,Outcome outcome) noexcept
{
    g_covCalls.fetch_add(1,std::memory_order_relaxed);
    uint32_t writers=record&mm::corewrite::kWriterMask;
    if(!writers){g_cov[kNoWriter][size_t(outcome)].fetch_add(1,std::memory_order_relaxed);return;}
    for(uint32_t w=0;writers;++w,writers>>=1)
        if(writers&1u)g_cov[w][size_t(outcome)].fetch_add(1,std::memory_order_relaxed);
}
thread_local mm::bodyhelpercandidate::Scratch g_scratch;
thread_local mm::helperpatch::Patch g_patch;

bool ReadBounded(const std::filesystem::path& path,std::vector<uint8_t>& bytes)
{
    std::ifstream file(path,std::ios::binary);
    if(!file)return false;
    file.seekg(0,std::ios::end);const auto end=static_cast<std::streamoff>(file.tellg());
    if(end<=0||static_cast<uint64_t>(end)>kMaxBundleFile)return false;
    file.seekg(0,std::ios::beg);
    std::vector<uint8_t> next(static_cast<size_t>(end));
    file.read(reinterpret_cast<char*>(next.data()),static_cast<std::streamsize>(next.size()));
    if(!file||static_cast<size_t>(file.gcount())!=next.size())return false;
    bytes=std::move(next);return true;
}

using util::Sha256Hex;

bool ParseMode(const std::vector<uint8_t>& bytes,Mode& mode)
{
    const std::string text(bytes.begin(),bytes.end());std::istringstream input(text);
    std::string token,extra;if(!(input>>token)||(input>>extra))return false;
    if(token=="OFF")mode=Mode::Off;
    else if(token=="PROBE")mode=Mode::Probe;
    else if(token=="APPLY")mode=Mode::Apply;
    else return false;
    return true;
}

void SetStatus(std::string value)
{
    std::lock_guard<std::mutex> lock(g_coldMutex);g_status=std::move(value);
}
bool Contains(std::span<const uint64_t> values,uint64_t value)
{ return std::find(values.begin(),values.end(),value)!=values.end(); }
} // namespace

bool Enabled() noexcept
{
    return g_loaded.load(std::memory_order_acquire)&&g_mode.load(std::memory_order_acquire)!=Mode::Off;
}

bool ApplyEnabled() noexcept
{
    return g_loaded.load(std::memory_order_acquire)&&g_mode.load(std::memory_order_acquire)==Mode::Apply;
}

namespace
{
struct BodyFolder { const wchar_t* dir; const char* label; int32_t body; };
constexpr std::array<BodyFolder,kMaxBundles> kBodyFolders{{
    {L"man","man",mm::helperadmission::kBodyMan},{L"woman","woman",mm::helperadmission::kBodyWoman}}};

// One bundle folder. Returns null with `why` set when the folder has no usable bundle for this mode. The folder decides nothing by
// itself: APPLY needs the exact approved bytes (mm/BodyHelperAdmission.hpp: SHA-256 of both files + declared counts), the
// structural/hash checks below prove identity, not anatomical validity, and an approved bundle must sit in the folder of its body.
std::shared_ptr<const Bundle> LoadBundleFolder(const std::filesystem::path& folder,Mode mode,const BodyFolder& which,std::string& why)
{
    std::vector<uint8_t> programBytes,bindingBytes;
    std::error_code ec;
    if(!std::filesystem::exists(folder,ec)){why="no folder";return nullptr;}
    if(!ReadBounded(folder/L"program.ir",programBytes)||!ReadBounded(folder/L"binding.dat",bindingBytes))
    { why="program.ir or binding.dat missing or exceeds 2 MiB";return nullptr; }
    auto next=std::make_shared<Bundle>();next->program=std::make_unique<mm::deformoffline::Program>();next->label=which.label;
    next->programFileHash=Sha256Hex(programBytes);
    if(next->programFileHash.empty()){why="CNG SHA-256 failed";return nullptr;}
    const std::string programText(programBytes.begin(),programBytes.end());
    std::istringstream programInput(programText);
    if(!mm::deformerfile::Read(programInput,*next->program,next->programNames,next->identity))
    { why="program.ir parser rejected bundle";return nullptr; }
    const std::string bindingText(bindingBytes.begin(),bindingBytes.end());
    std::istringstream bindingInput(bindingText);
    if(!mm::helperbinding::Read(bindingInput,*next->program,
        {next->programNames.data(),next->program->nodeCount},next->identity,next->programFileHash,next->binding))
    { why="binding.dat parser or byte-hash identity rejected bundle";return nullptr; }
    const auto verdict=mm::helperadmission::Admit(static_cast<mm::helperadmission::Mode>(mode),next->programFileHash,Sha256Hex(bindingBytes),
        static_cast<uint32_t>(next->binding.owned.size()),static_cast<uint32_t>(next->binding.children.size()));
    if(verdict.decision==mm::helperadmission::Decision::RefuseApply)
    { why="APPLY blocked: bundle is not an approved helper bundle for runtime writes; use OFF or PROBE";return nullptr; }
    if(verdict.approved&&verdict.body!=which.body)
    { why="bundle is approved for the other body, wrong folder";return nullptr; }
    next->approved=verdict.approved;next->body=verdict.body;next->verdict=verdict;
    const auto& b=next->binding;
    next->bones=b.physicalCore;
    for(const auto* group:{&b.programCore,&b.owned,&b.children})next->bones.insert(next->bones.end(),group->begin(),group->end());
    next->bones.push_back(b.anchor);
    return next;
}
} // namespace

bool Load(const std::wstring& directoryDll)
{
    std::lock_guard<std::mutex> lock(g_coldMutex);
    if(directoryDll.empty()||g_hot.armed.load(std::memory_order_acquire))
    { g_status="load requires a nonempty DLL directory while disarmed";return false; }
    g_active.store(nullptr,std::memory_order_release);
    g_mode.store(Mode::Off,std::memory_order_release);
    g_loaded.store(false,std::memory_order_release);
    g_bundles.clear();
    try
    {
        const auto root=std::filesystem::path(directoryDll)/L"skin_helpers";
        std::vector<uint8_t> modeBytes;
        if(!ReadBounded(root/L"mode.txt",modeBytes))
        { g_status="skin_helpers/mode.txt missing or exceeds 2 MiB";return false; }
        Mode mode=Mode::Off;
        if(!ParseMode(modeBytes,mode)){g_status="invalid skin helpers mode";return false;}
        if(mode==Mode::Off)
        { g_loaded.store(true,std::memory_order_release);g_status="OFF";return true; }
        // skin_helpers/man and skin_helpers/woman are admitted independently: a missing, broken or unapproved bundle
        // of one body never disables the other. mode.txt only selects the mode and can never promote an unapproved bundle.
        std::vector<std::shared_ptr<const Bundle>> loaded;
        std::string loadedText,skippedText;
        for(const auto& which:kBodyFolders)
        {
            std::string why;
            auto bundle=LoadBundleFolder(root/which.dir,mode,which,why);
            if(bundle)
            {
                loadedText+=std::string(loadedText.empty()?"":", ")+which.label+(bundle->approved?" (body "+std::to_string(bundle->body)+")":" (not approved)");
                loaded.push_back(std::move(bundle));
            }
            else skippedText+=std::string(skippedText.empty()?"":"; ")+which.label+": "+why;
        }
        if(loaded.empty()){g_status="no usable helper bundle ("+skippedText+")";return false;}
        g_bundles=std::move(loaded);g_mode.store(mode,std::memory_order_release);
        g_loaded.store(true,std::memory_order_release);
        g_status=(mode==Mode::Apply?"APPLY approved helper bundles loaded: "+loadedText:"PROBE bundles loaded: "+loadedText+"; writes disabled")+
                 (skippedText.empty()?"":"; skipped - "+skippedText);
        return true;
    }
    catch(...)
    { g_bundles.clear();g_status="bundle load allocation or filesystem failure";return false; }
}

bool PrepareArm(uint64_t rig,std::span<const int16_t> parents,
                std::span<const uint64_t> names,std::span<const uint64_t> requiredHelpers)
{
    g_active.store(nullptr,std::memory_order_release);
    if(g_hot.armed.load(std::memory_order_acquire)||!Enabled()||!rig||parents.empty()||
       parents.size()!=names.size()||parents.size()>mm::bodyhelpercandidate::kMaxMergedBones||
       requiredHelpers.empty()||requiredHelpers.size()>mm::mappingcycle::kMaxBones)
        return false;
    try
    {
        std::lock_guard<std::mutex> lock(g_coldMutex);
        if(g_hot.armed.load(std::memory_order_acquire)||g_bundles.empty()||g_mode.load(std::memory_order_relaxed)==Mode::Off)
            return false;
        // g_coldMutex is held here: set g_status directly (SetStatus would deadlock).
        for(uint32_t i=0;i<requiredHelpers.size();++i)
        {
            if(!requiredHelpers[i]||Contains(requiredHelpers.first(i),requiredHelpers[i]))
            { g_status="mapping-cycle required helper catalog is invalid";return false; }
        }
        std::array<int32_t,mm::bodyhelpercandidate::kMaxMergedBones> converted{};
        for(uint32_t i=0;i<parents.size();++i)converted[i]=parents[i];
        auto active=std::make_unique<Active>();active->rig=rig;
        active->count=static_cast<uint32_t>(parents.size());
        std::fill_n(active->available.begin(),parents.size(),uint8_t(1));
        active->requiredCount=static_cast<uint32_t>(requiredHelpers.size());
        const std::array<uint64_t,3> protectedTranslations{
            mm::integ::CNameHash("Root"),mm::integ::CNameHash("Hips"),mm::integ::CNameHash("Trajectory")};
        // One plan per loaded bundle that fits the armed rig. A bundle fits when the rig has at least the approved base and the bones
        // the bundle reads and writes have the approved layout (mm/BodyHelperAdmission.hpp CheckRig); other bones (hair, accessories,
        // garment dangles) do not matter. The 294-bone loading rig and a changed game layout publish no plan; the previous plan was
        // already withdrawn above. The man and woman bundles share one layout, so the body is chosen per call from the live pose.
        std::string notes;char text[256];
        for(const auto& bundle:g_bundles)
        {
            const auto skip=[&](const std::string& why){notes+=std::string(notes.empty()?"":"; ")+bundle->label+": "+why;};
            mm::helperadmission::RigSignature sig;
            const auto check=mm::helperadmission::CheckRig<int16_t>(bundle->verdict,names,parents,bundle->bones,&sig);
            if(check==mm::helperadmission::RigCheck::TooFewBones)
            {
                std::snprintf(text,sizeof text,"arm refused: rig of %u bones is smaller than the merged player rig the approved helper bundle is valid for (at least %u bones)",
                    static_cast<uint32_t>(parents.size()),bundle->verdict.baseMinBones);
                skip(text);continue;
            }
            if(check==mm::helperadmission::RigCheck::SignatureMismatch)
            {
                std::snprintf(text,sizeof text,"arm refused: the bones the approved helper bundle reads and writes (%u bones, fingerprint %016llx, %s) differ from the approved layout (%u bones, %016llx; rig of %u bones)",
                    sig.bones,static_cast<unsigned long long>(sig.fingerprint),sig.ok?"complete":"incomplete or malformed",bundle->verdict.closureBones,
                    static_cast<unsigned long long>(bundle->verdict.closureFingerprint),static_cast<uint32_t>(parents.size()));
                skip(text);continue;
            }
            Slot& slot=active->slots[active->slotCount];
            const auto& b=bundle->binding;
            auto result=mm::bodyhelpercandidate::Prepare(*bundle->program,
                {bundle->programNames.data(),bundle->program->nodeCount},
                {converted.data(),parents.size()},names,b.programCore,b.physicalCore,b.locals,
                protectedTranslations,b.owned,b.children,b.outputs,b.anchor,slot.plan);
            if(!result){skip("skin helpers Prepare rejected catalog, code="+std::to_string(uint32_t(result.error))+" index="+std::to_string(result.index));continue;}
            bool catalogOk=true;
            for(uint32_t k=0;k<slot.plan.helperPlan.outputCount;++k)
                if(!Contains(requiredHelpers,names[slot.plan.helperPlan.outputs[k]]))catalogOk=false;
            if(!catalogOk){skip("skin helper output is absent from mapping-cycle required helper catalog");continue;}
            slot.bundle=bundle;++active->slotCount;
            if(bundle->approved)active->anyApproved=true;
        }
        if(!active->slotCount){g_status=notes;return false;}
        std::string armed;
        for(uint32_t i=0;i<active->slotCount;++i)
            armed+=std::string(i?", ":"")+active->slots[i].bundle->label+" (body "+std::to_string(active->slots[i].bundle->body)+")";
        auto* published=active.get();g_retained.push_back(std::move(active));
        g_active.store(published,std::memory_order_release);
        g_status="arm plan prepared: "+armed+(notes.empty()?"":"; skipped - "+notes);return true;
    }
    catch(...)
    { SetStatus("skin helpers arm preparation allocation failure");return false; }
}

bool Evaluate(uint8_t* desc,uint8_t* bones,uint32_t count,uint64_t rig,
              const mm::mappingcycle::Receipt& receipt,int64_t now) noexcept
{
    g_evaluated.fetch_add(1,std::memory_order_relaxed);
    const uint32_t record=mm::corewrite::Current(); // which layers wrote core bones in this FK call
    mm::corewrite::MarkEvaluated();
    namespace ha=mm::helperadmission;
    auto reject=[record](ha::Reject why){g_rejected.fetch_add(1,std::memory_order_relaxed);g_rejectReason[size_t(why)].fetch_add(1,std::memory_order_relaxed);
        Tally(record,Outcome(uint32_t(why)));return false;};
    Active* active=g_active.load(std::memory_order_acquire);
    const bool approved=active&&active->anyApproved;
    // The hot gate (mm/BodyHelperAdmission.hpp Gate): ragdoll episode = the engine owns V's pose; session owner Mapping (experiment) or,
    // for an approved bundle only, FK (the shipped default); armed + MM active; descriptor/rig/count identity; TPP perspective
    // write window; complete mapping-cycle receipt (every required helper still an unmodified native copy); then the body.
    ha::GateInputs gi;
    gi.ragdollGameOwned=ragdoll::GameOwned();
    const auto owner=earlyruntime::CurrentOwner();
    gi.enabled=Enabled();gi.fault=g_hot.fault.load(std::memory_order_acquire);
    gi.planPresent=active!=nullptr;
    gi.ownerOk=ha::OwnerAllows(owner==earlyruntime::Owner::Mapping,owner==earlyruntime::Owner::FK,approved);
    gi.armed=g_hot.armed.load(std::memory_order_acquire);gi.mmActive=g_hot.mmActive.load(std::memory_order_acquire);
    gi.identityOk=active&&desc&&bones&&reinterpret_cast<uint64_t>(desc)==g_hot.playerDesc.load(std::memory_order_relaxed)&&rig==active->rig&&
        count==active->count&&count==g_hot.playerBones.load(std::memory_order_relaxed)&&rig==g_hot.playerRig.load(std::memory_order_relaxed)&&
        reinterpret_cast<uintptr_t>(bones)%alignof(float)==0;
    const int64_t maxAge=g_hot.perspectiveWriteMaxAge.load(std::memory_order_relaxed);
    gi.perspectiveOk=maxAge>0&&g_hot.perspectiveWrite.Allows(now,maxAge);
    gi.receiptComplete=active&&receipt.status==mm::mappingcycle::ConsumeStatus::Complete&&receipt.complete&&
        receipt.required==active->requiredCount&&receipt.covered==active->requiredCount&&
        receipt.equal==active->requiredCount&&receipt.startedTicks>0&&
        receipt.lastTicks>=receipt.startedTicks&&now>=receipt.lastTicks&&now-receipt.lastTicks<=maxAge;
    if(const auto why=ha::Gate(gi);why!=ha::Reject::None)return reject(why);
    mm::integ::PoseSnapshot body;
    if (!g_mmPose.Read(body) || body.rig!=rig || !(body.weight>0) ||
        body.publishTicks<=0 || now<body.publishTicks || now-body.publishTicks>maxAge) return reject(ha::Reject::Body);
    // The bundle of the live body: the man and the woman bundle fit the same rig, so the pose snapshot decides which plan may run.
    // A body without a loaded bundle (missing / unapproved / refused at arm) writes nothing; the other body is not affected.
    Slot* slot=nullptr;
    for(uint32_t i=0;i<active->slotCount;++i)
        if(ha::BodyMatches(active->slots[i].bundle->approved,active->slots[i].bundle->body,body.bodyType)){slot=&active->slots[i];break;}
    if(!slot)return reject(ha::Reject::Body);
    if(!ha::OwnerAllows(owner==earlyruntime::Owner::Mapping,owner==earlyruntime::Owner::FK,slot->bundle->approved))return reject(ha::Reject::Gate);
    const std::span<const float> records(reinterpret_cast<const float*>(bones),
        size_t(count)*mm::helperpatch::kRecordFloats);
    const auto result=mm::bodyhelpercandidate::Build(slot->plan,records,
        {active->available.data(),count},g_scratch,g_patch);
    if(!result){g_buildFailed.fetch_add(1,std::memory_order_relaxed);Tally(record,Outcome::Build);return false;}
    g_proposed.fetch_add(1,std::memory_order_relaxed);
    g_proposedRecords.fetch_add(g_patch.count,std::memory_order_relaxed);
    if(g_mode.load(std::memory_order_acquire)==Mode::Apply)
    {
        struct CommitContext { Active* active;const Slot* slot;uint8_t* desc;uint8_t* bones;uint64_t rig,tag;uint32_t count;int64_t receiptTicks; };
        CommitContext context{active,slot,desc,bones,rig,*reinterpret_cast<const uint64_t*>(desc+kDescTagOff),count,receipt.lastTicks};
        const auto permission=[](void* opaque) noexcept
        {
            const auto& c=*static_cast<const CommitContext*>(opaque);
            LARGE_INTEGER clock{};QueryPerformanceCounter(&clock);
            const auto age=g_hot.perspectiveWriteMaxAge.load(std::memory_order_relaxed);
            const auto liveOwner=earlyruntime::CurrentOwner();
            if(ragdoll::GameOwned()||!Enabled()||g_mode.load(std::memory_order_acquire)!=Mode::Apply||
               g_active.load(std::memory_order_acquire)!=c.active||g_hot.fault.load(std::memory_order_acquire)||
               !g_hot.armed.load(std::memory_order_acquire)||!g_hot.mmActive.load(std::memory_order_acquire)||
               !mm::helperadmission::OwnerAllows(liveOwner==earlyruntime::Owner::Mapping,liveOwner==earlyruntime::Owner::FK,c.slot->bundle->approved)||
               reinterpret_cast<uint64_t>(c.desc)!=g_hot.playerDesc.load(std::memory_order_relaxed)||
               c.rig!=g_hot.playerRig.load(std::memory_order_relaxed)||
               c.count!=g_hot.playerBones.load(std::memory_order_relaxed)||
               !g_hot.perspectiveWrite.Allows(clock.QuadPart,age)||c.receiptTicks<=0||
               clock.QuadPart<c.receiptTicks||clock.QuadPart-c.receiptTicks>age)return false;
            if(!c.tag||(c.tag!=g_hot.trackA.load(std::memory_order_relaxed)&&c.tag!=g_hot.trackB.load(std::memory_order_relaxed))||
               *reinterpret_cast<uint8_t* const*>(c.desc+kDescBonesOff)!=c.bones||
               *reinterpret_cast<const uint32_t*>(c.desc+kDescCountOff)!=c.count||
               *reinterpret_cast<const uint64_t*>(c.desc+kDescTagOff)!=c.tag)return false;
            mm::integ::PoseSnapshot current;
            return g_mmPose.Read(current)&&mm::helperadmission::BodyMatches(c.slot->bundle->approved,c.slot->bundle->body,current.bodyType)&&current.rig==c.rig&&current.weight>0&&
                current.publishTicks>0&&clock.QuadPart>=current.publishTicks&&clock.QuadPart-current.publishTicks<=age;
        };
        const auto committed=mm::bodyhelpercommit::Commit(slot->plan.helperPlan,
            {reinterpret_cast<float*>(bones),size_t(count)*mm::helperpatch::kRecordFloats},g_patch,permission,&context);
        if(!committed)
        {
            g_commitRejected.fetch_add(1,std::memory_order_relaxed);
            if(committed.error==mm::bodyhelpercommit::Error::TranslationJump)g_jumpRejected.fetch_add(1,std::memory_order_relaxed);
            Tally(record,Outcome::Commit);
            return false;
        }
        g_applied.fetch_add(1,std::memory_order_relaxed);g_appliedRecords.fetch_add(g_patch.count,std::memory_order_relaxed);
        Tally(record,Outcome::Committed);
        // This is the first point where the local records include the applied
        // helper correction. Keep it separate from FkHook's earlier MM/Aim pair.
        if(mm::bodytrace::g_postHelperCapture.Active())
            mm::bodytrace::g_postHelperCapture.Capture(bones,count,rig,
                reinterpret_cast<uint64_t>(desc),now,GetCurrentThreadId(),receipt.generation,
                receipt.cycle,receipt.startedTicks,receipt.lastTicks);
    }
    else Tally(record,Outcome::ProposedOnly);
    return true;
}

void NoteNotEvaluated(uint32_t aRecord) noexcept
{
    if(Enabled())Tally(aRecord,Outcome::NotEvaluated);
}

Coverage ReadCoverage() noexcept
{
    Coverage out;
    out.calls=g_covCalls.load(std::memory_order_relaxed);
    for(size_t w=0;w<kWriterRows;++w)
        for(size_t o=0;o<size_t(Outcome::Count);++o)out.byWriter[w][o]=g_cov[w][o].load(std::memory_order_relaxed);
    return out;
}

void Disable() noexcept
{
    g_loaded.store(false,std::memory_order_release);
    g_mode.store(Mode::Off,std::memory_order_release);
    g_active.store(nullptr,std::memory_order_release);
}

Counters ReadCounters() noexcept
{
    return {g_evaluated.load(std::memory_order_relaxed),g_rejected.load(std::memory_order_relaxed),
        g_buildFailed.load(std::memory_order_relaxed),g_proposed.load(std::memory_order_relaxed),
        g_proposedRecords.load(std::memory_order_relaxed),g_applied.load(std::memory_order_relaxed),
        g_appliedRecords.load(std::memory_order_relaxed),g_commitRejected.load(std::memory_order_relaxed),g_jumpRejected.load(std::memory_order_relaxed),
        {g_rejectReason[0].load(std::memory_order_relaxed),g_rejectReason[1].load(std::memory_order_relaxed),
         g_rejectReason[2].load(std::memory_order_relaxed),g_rejectReason[3].load(std::memory_order_relaxed),
         g_rejectReason[4].load(std::memory_order_relaxed),g_rejectReason[5].load(std::memory_order_relaxed),
         g_rejectReason[6].load(std::memory_order_relaxed)}};
}

std::vector<uint64_t> RequiredCatalog()
{
    std::vector<uint64_t> out;
    try
    {
        std::lock_guard<std::mutex> lock(g_coldMutex);
        if(g_mode.load(std::memory_order_relaxed)==Mode::Off)return out;
        // Union over the approved bundles (the same hashes for the man and the woman bundle: they own the same 40 helper bones).
        for(const auto& bundle:g_bundles)
        {
            if(!bundle->approved)continue;
            for(const auto* group:{&bundle->binding.owned,&bundle->binding.children})
                for(const uint64_t hash:*group)if(!Contains(out,hash))out.push_back(hash);
        }
    }
    catch(...){out.clear();}
    return out;
}

std::string Status()
{
    std::lock_guard<std::mutex> lock(g_coldMutex);return g_status;
}
} // namespace utpp::bodyhelperruntime
