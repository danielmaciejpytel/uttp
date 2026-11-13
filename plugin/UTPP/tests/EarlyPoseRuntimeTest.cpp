// Focused owner/admission regression using owned hashes/pairs from retail census.
// No game process, RED4ext, database assets, or pose-writing hook is linked.
#include "../src/EarlyPoseRuntime.hpp"
#include "../src/State.hpp"
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace utpp
{
HotState g_hot;
mm::integ::SeqlockBuffer<mm::integ::PoseSnapshot> g_mmPose;
}
#define CHECK(x) do { if (!(x)) { std::fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #x); std::abort(); } } while (false)

namespace
{
using namespace utpp;
using namespace utpp::earlyruntime;
struct Record
{
    uint64_t sourceFingerprint = 0, sourceRig = 0, mergedRig = 0, tag = 0;
    std::vector<uint64_t> source, destination;
    std::vector<uint32_t> pairs;
};
std::vector<std::string> Split(const std::string& s)
{
    std::vector<std::string> out;
    std::istringstream stream(s); std::string field;
    while (std::getline(stream, field, ',')) out.push_back(field);
    if (!out.empty() && !out.back().empty() && out.back().back() == '\r') out.back().pop_back();
    return out;
}
std::map<unsigned, Record> Read(const char* path)
{
    std::ifstream input(path); CHECK(input.good());
    std::string line; std::map<std::string, size_t> columns; std::map<unsigned, Record> records;
    while (std::getline(input, line))
    {
        if (line.empty() || line[0] == '#') continue;
        const auto values = Split(line);
        if (columns.empty()) { for (size_t i = 0; i < values.size(); ++i) columns[values[i]] = i; continue; }
        auto get = [&](const char* key) -> const std::string& { return values.at(columns.at(key)); };
        const unsigned id = static_cast<unsigned>(std::stoul(get("record")));
        auto& rec = records[id];
        if (get("kind") == "identity")
        {
            rec.sourceFingerprint = std::stoull(get("sourceNamesFingerprint"), nullptr, 16);
            rec.sourceRig = std::stoull(get("sourceRig")); rec.mergedRig = std::stoull(get("destinationRig"));
            rec.tag = std::stoull(get("tag"));
            rec.source.resize(std::stoul(get("sourceCount")));
            rec.destination.resize(std::stoul(get("destinationCount")));
            rec.pairs.resize(std::stoul(get("pairCount")) * 2);
        }
        else
        {
            const size_t index = std::stoul(get("index"));
            if (get("kind") == "sourceName") rec.source.at(index) = std::stoull(get("nameHash"), nullptr, 16);
            else if (get("kind") == "destinationName") rec.destination.at(index) = std::stoull(get("nameHash"), nullptr, 16);
            else if (get("kind") == "pair")
            {
                rec.pairs.at(index * 2) = static_cast<uint32_t>(std::stoul(get("sourceIndex")));
                rec.pairs.at(index * 2 + 1) = static_cast<uint32_t>(std::stoul(get("destinationIndex")));
            }
        }
    }
    CHECK(!records.empty()); return records;
}
struct Callback
{
    const mm::integ::PoseSnapshot* expected = nullptr;
    uint32_t calls = 0, result = 71;
    uint8_t* bones = nullptr;
    float observedWeight = 0;
};
uint32_t CountApply(void* user, uint8_t* desc, uint8_t* bones, uint32_t count, uint64_t rig,
               const mm::integ::PoseSnapshot& snapshot, int64_t now)
{
    auto& c = *static_cast<Callback*>(user);
    CHECK(desc && bones == c.bones && count == 619 && rig == snapshot.rig && now == 1000);
    CHECK(&snapshot == c.expected); // caller's one latched snapshot, no second pose read
    ++c.calls; c.observedWeight = snapshot.weight;
    if (c.result) bones[0] = static_cast<uint8_t>(c.calls); // observable callback, not a skeletal correction
    return c.result;
}
}

int main(int argc, char** argv)
{
    CHECK(argc == 2);
    auto records = Read(argv[1]);
    Record* main = nullptr; Record* deformer = nullptr;
    // Full-name fingerprints are evidence from the validated owned fixture,
    // never production pass routing by count.
    for (auto& [id, r] : records)
    {
        if (r.sourceFingerprint == 0xDC0F857EC1627C40ULL) main = &r;
        if (r.sourceFingerprint == 0xC09A9F1B89C6AB98ULL) deformer = &r;
    }
    CHECK(main && deformer && main->destination == deformer->destination);
    std::vector<uint64_t> helpers;
    for (uint64_t h : deformer->source)
        if (std::find(main->source.begin(), main->source.end(), h) == main->source.end()) helpers.push_back(h);
    CHECK(helpers.size() == 116 && main->source.size() >= 73 && main->destination.size() == 619);
    mm::integ::PoseSnapshot snapshot;
    snapshot.rig = main->mergedRig; snapshot.numBones = 73; snapshot.firstBone = 2;
    snapshot.weight = 1; snapshot.publishTicks = 990;
    for (uint32_t i = 2; i < 73; ++i)
    {
        const auto it = std::find(main->destination.begin(), main->destination.end(), main->source[i]);
        CHECK(it != main->destination.end());
        snapshot.gameIndex[i] = static_cast<int32_t>(it - main->destination.begin());
    }
    std::vector<uint8_t> bones(619 * 48); uint8_t desc[32]{};
    MappingView mapping{desc, bones.data(), main->tag, main->sourceRig, 619, main->source, main->pairs};
    Callback callback{&snapshot, 0, 71, bones.data()};
    g_hot.armed.store(false);
    CHECK(SelectOwner(Owner::FK));
    CHECK(EvaluateMapping(mapping, snapshot, 1000, CountApply, &callback) == Result::WrongOwner);
    CHECK(callback.calls == 0);
    CHECK(SelectOwner(Owner::Mapping));
    CHECK(EvaluateMapping(mapping, snapshot, 1000, CountApply, &callback) == Result::NotReady);
    CHECK(!PrepareArm(main->mergedRig, 619, main->tag, 0, main->destination, {}));
    CHECK(PrepareArm(main->mergedRig, 619, main->tag, 0, main->destination, helpers));
    g_hot.playerRig.store(main->mergedRig); g_hot.playerBones.store(619);
    g_hot.trackA.store(main->tag); g_hot.trackB.store(0); g_hot.mmActive.store(true);
    g_hot.mmStaleTicks.store(100); g_hot.perspectiveWriteMaxAge.store(100);
    g_hot.perspectiveWrite.Publish(true, 990); g_hot.armed.store(true);
    CHECK(!SelectOwner(Owner::FK)); // no mid-flight owner handoff
    CHECK(EvaluateMapping(mapping, snapshot, 1000, CountApply, &callback) == Result::Applied);
    CHECK(callback.calls == 1 && bones[0] == 1);

    auto wrong = mapping; wrong.tag ^= 0x100;
    CHECK(EvaluateMapping(wrong, snapshot, 1000, CountApply, &callback) == Result::IdentityRejected);
    auto badPairs = main->pairs; badPairs[0] = static_cast<uint32_t>(main->source.size());
    wrong = mapping; wrong.pairs = badPairs;
    CHECK(EvaluateMapping(wrong, snapshot, 1000, CountApply, &callback) == Result::SemanticRejected);
    badPairs = main->pairs; badPairs[1] = badPairs[3]; wrong.pairs = badPairs;
    CHECK(EvaluateMapping(wrong, snapshot, 1000, CountApply, &callback) == Result::SemanticRejected);
    for (const auto& [id, r] : records) if (&r != main)
    {
        wrong = mapping; wrong.sourceRig = r.sourceRig; wrong.sourceNames = r.source; wrong.pairs = r.pairs;
        CHECK(EvaluateMapping(wrong, snapshot, 1000, CountApply, &callback) == Result::SemanticRejected);
    }
    CHECK(callback.calls == 1);
    g_hot.perspectiveWrite.Publish(false, 991); // FPP denies even with fresh MM/owner
    CHECK(EvaluateMapping(mapping, snapshot, 1000, CountApply, &callback) == Result::PerspectiveRejected);
    CHECK(CurrentOwner() == Owner::Mapping && callback.calls == 1); // no FK fallback
    g_hot.perspectiveWrite.Publish(true, 800);
    CHECK(EvaluateMapping(mapping, snapshot, 1000, CountApply, &callback) == Result::PerspectiveRejected);
    g_hot.perspectiveWrite.Publish(true, 990);
    auto stale = snapshot; stale.publishTicks = 800;
    CHECK(EvaluateMapping(mapping, stale, 1000, CountApply, &callback) == Result::PoseRejected);
    auto future = snapshot; future.publishTicks = 1001;
    CHECK(EvaluateMapping(mapping, future, 1000, CountApply, &callback) == Result::PoseRejected);

    g_mmPose.Publish(snapshot);
    mm::integ::PoseSnapshot latched; CHECK(g_mmPose.Read(latched));
    auto other = snapshot; other.rig = 1; other.weight = 0; g_mmPose.Publish(other);
    callback.expected = &latched;
    CHECK(EvaluateMapping(mapping, latched, 1000, CountApply, &callback) == Result::Applied);
    CHECK(callback.calls == 2 && callback.observedWeight == 1); // global now contains another pose
    callback.result = 0;
    CHECK(EvaluateMapping(mapping, latched, 1000, CountApply, &callback) == Result::CallbackNoWrite);
    CHECK(callback.calls == 3);
    Disarm();
    CHECK(EvaluateMapping(mapping, latched, 1000, CountApply, &callback) == Result::NotReady);
    g_hot.armed.store(false); CHECK(SelectOwner(Owner::FK));
    CHECK(EvaluateMapping(mapping, snapshot, 1000, CountApply, &callback) == Result::WrongOwner);
    CHECK(callback.calls == 3);
    std::puts("early pose runtime targeted checks PASS");
    return 0;
}
