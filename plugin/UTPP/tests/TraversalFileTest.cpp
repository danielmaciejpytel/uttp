// UTPP motion stage 10.8 part 2 - offline test of src/mm/TraversalFile.hpp + BaseTraversal.hpp SolveWarp on real baked
// traversal montages (Data/traversal/utpp_traversal_{woman,man}.utpptrav, motion-derived, outside git).
// Usage: utpp_mm_travfile_test <utpp_traversal_woman.utpptrav> <utpp_traversal_man.utpptrav>
//  1) both files load (fail-closed reader), 34 montages / 55 MotionWarping windows each, same montages and window
//     frames for woman and man, different rig identity; corrupted copies are rejected and leave the output empty;
//  2) SolveWarp on one Hurdle, one Mantle and one Vault montage against synthetic ledges (rigid offsets of the
//     clip's own ledge: dx, dy +-0.3 m, ledge height +-0.2 m, yaw +-0.25 rad): the contact hand (UE attach bone
//     substitute) is within 5 cm of FrontLedge at the window end, no planar (xy) root step > 0.30 m per frame, no
//     warp-added root step (warped - animated, 3D) > 0.30 m per frame, and the identity ledge reproduces the clip root
//     track. The full 3D step is reported only: the clips' own vertical root speed (root_z, RTZ) already exceeds
//     0.30 m per frame at 30 fps (vault / mantle take-off, falling end of the run vault).
// trav, optional arguments 3 / 4: the FORMAT_TRAVERSAL version 2 files (utpp_traversal_{woman,man}_v2.utpptrav,
//  travbake.py --format 2 --variant knee): 3) version 2, POSEBONES (73, unique, the hand / hips bones), all 34 montages
//     with frames x bones x 7 pose values, unit quaternions, montages / frames / windows as the version 1 file; version 1
//     files carry no poses; corrupted version 2 copies (missing / short P line, non-unit quaternion, POSEBONES count,
//     duplicate pose bone, version 1 header) and a P line in a version 1 file are rejected.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "../src/mm/TraversalFile.hpp"

using namespace utpp::mm::traversal;

static int g_pass = 0, g_fail = 0;
static void Check(bool aOk, const std::string& aWhat)
{
    if (aOk)
        ++g_pass;
    else
    {
        ++g_fail;
        std::printf("FAIL: %s\n", aWhat.c_str());
    }
}

static bool ReadAll(const char* aPath, std::string& aOut)
{
    std::ifstream f(aPath, std::ios::binary);
    if (!f)
        return false;
    std::ostringstream ss;
    ss << f.rdbuf();
    aOut = ss.str();
    return true;
}

static size_t CountWindows(const TraversalFile& aF)
{
    size_t n = 0;
    for (const auto& m : aF.montages)
        n += m.windows.size();
    return n;
}

static void CorruptionTests(const std::string& aText)
{
    struct Mut
    {
        const char* what;
        std::string text;
    };
    std::vector<Mut> muts;
    auto replaceFirst = [&](const char* aWhat, const std::string& aFrom, const std::string& aTo) {
        std::string s = aText;
        const size_t p = s.find(aFrom);
        if (p == std::string::npos)
        {
            Check(false, std::string("mutation anchor missing: ") + aWhat);
            return;
        }
        s.replace(p, aFrom.size(), aTo);
        muts.push_back({aWhat, s});
    };
    replaceFirst("version 2", "utpp-traversal 1", "utpp-traversal 2");
    replaceFirst("missing E", "\nE\n", "\n");
    replaceFirst("extra R token", "\nR ", "\nR 1.000000 ");
    replaceFirst("float format", "\nR 0.000000", "\nR 0.0");
    replaceFirst("CRLF", "\n", "\r\n");
    replaceFirst("double space", "\nR ", "\nR  ");
    replaceFirst("bad target", "\nW FrontLedge", "\nW SideLedge");
    replaceFirst("COUNT too big", "COUNT 34", "COUNT 35");
    muts.push_back({"truncated", aText.substr(0, aText.size() - 4)});
    muts.push_back({"trailing data", aText + "E\n"});
    for (const auto& m : muts)
    {
        TraversalFile f;
        f.rigBones = 7;
        f.montages.resize(1);
        std::string err;
        const bool ok = ParseTraversalText(m.text, f, err);
        Check(!ok && f.montages.empty() && f.rigBones == 0 && !err.empty(),
              std::string("corrupt file accepted or output not cleared: ") + m.what);
    }
}

struct WarpStats
{
    int cases = 0;
    double maxContactErr = 0;
    double maxStep = 0;     // 3D root step, reported only (the clip's own vertical root speed exceeds 0.30 m/frame)
    double maxPlanar = 0;   // xy root step, checked <= 0.30 m
    double maxExtra = 0;    // warped step - animated step (3D), checked <= 0.30 m
    double maxClipStep = 0; // 3D root step of the unwarped clip
};

static WarpStats RunMontage(const TravMontage& aM, const char* aTag)
{
    WarpStats st;
    const std::vector<RootKey>& clip = aM.root;
    Check(WindowsSolvable(aM), std::string(aTag) + " " + aM.name + ": windows overlap");
    int front = -1;
    for (size_t i = 0; i < aM.windows.size(); ++i)
        if (aM.windows[i].target == "FrontLedge" && front < 0)
            front = static_cast<int>(i);
    Check(front >= 0, std::string(aTag) + " " + aM.name + ": no FrontLedge window");
    if (front < 0)
        return st;
    for (size_t f = 1; f < clip.size(); ++f)
        st.maxClipStep = std::max(st.maxClipStep, Len(clip[f].pos - clip[f - 1].pos));
    const TravWindow& fw = aM.windows[static_cast<size_t>(front)];
    const V3 pivot = WarpPointWorld(clip[static_cast<size_t>(fw.endFrame)], fw);
    const double offs[3] = {-0.3, 0.0, 0.3};
    const double dzs[3] = {-0.2, 0.0, 0.2};
    const double dyaws[3] = {-0.25, 0.0, 0.25};
    for (double dx : offs)
        for (double dy : offs)
            for (double dz : dzs)
                for (double dyaw : dyaws)
                {
                    // synthetic ledge = the clip's own ledge rotated by dyaw about the front contact point, moved by
                    // (dx, dy) and raised by dz (front / back ledge; the back floor keeps its height)
                    std::vector<RootKey> targets;
                    for (const auto& w : aM.windows)
                    {
                        const RootKey& re = clip[static_cast<size_t>(w.endFrame)];
                        const V3 nat = WarpPointWorld(re, w);
                        V3 p = pivot + RotZ(nat - pivot, dyaw) + V3{dx, dy, w.target == "BackFloor" ? 0.0 : dz};
                        targets.push_back({p, re.yaw + dyaw});
                    }
                    const auto out = SolveWarp(clip, clip[0], MakeWarpWindows(aM, targets));
                    ++st.cases;
                    const RootKey& oe = out[static_cast<size_t>(fw.endFrame)];
                    V3 err = WarpPointWorld(oe, fw) - targets[static_cast<size_t>(front)].pos;
                    if (fw.ignoreZ)
                        err.z = 0;
                    const double e = Len(err);
                    st.maxContactErr = std::max(st.maxContactErr, e);
                    double maxStep = 0, maxPlanar = 0, maxExtra = 0;
                    for (size_t f = 1; f < out.size(); ++f)
                    {
                        const V3 d = out[f].pos - out[f - 1].pos;
                        const V3 anim = RotZ(RotZ(clip[f].pos - clip[f - 1].pos, -clip[f - 1].yaw), out[f - 1].yaw);
                        maxStep = std::max(maxStep, Len(d));
                        maxPlanar = std::max(maxPlanar, std::hypot(d.x, d.y));
                        maxExtra = std::max(maxExtra, Len(d - anim));
                    }
                    st.maxStep = std::max(st.maxStep, maxStep);
                    st.maxPlanar = std::max(st.maxPlanar, maxPlanar);
                    st.maxExtra = std::max(st.maxExtra, maxExtra);
                    char buf[320];
                    std::snprintf(buf, sizeof buf,
                                  "%s %s ledge (%.2f %.2f %.2f yaw %.2f): contact err %.4f m, max planar step %.4f m, "
                                  "max warp-added step %.4f m",
                                  aTag, aM.name.c_str(), dx, dy, dz, dyaw, e, maxPlanar, maxExtra);
                    Check(e <= 0.05, std::string(buf) + " (contact > 5 cm)");
                    Check(maxPlanar <= 0.30, std::string(buf) + " (planar root step > 0.30 m)");
                    Check(maxExtra <= 0.30, std::string(buf) + " (warp-added root step > 0.30 m)");
                    if (dx == 0 && dy == 0 && dz == 0 && dyaw == 0)
                    {
                        double dev = 0;
                        for (size_t f = 0; f < out.size(); ++f)
                            dev = std::max({dev, Len(out[f].pos - clip[f].pos), std::fabs(WrapPi(out[f].yaw - clip[f].yaw))});
                        Check(dev < 1e-6, std::string(aTag) + " " + aM.name + ": identity ledge changes the root track");
                    }
                }
    std::printf("%s %s: %d ledges, %zu windows, max contact err %.4f m, max planar step %.4f m, max warp-added "
                "step %.4f m, max 3D step %.4f m (unwarped clip 3D %.4f m)\n",
                aTag, aM.name.c_str(), st.cases, aM.windows.size(), st.maxContactErr, st.maxPlanar, st.maxExtra,
                st.maxStep, st.maxClipStep);
    return st;
}

static void RejectAll(const std::vector<std::pair<const char*, std::string>>& aMuts)
{
    for (const auto& m : aMuts)
    {
        TraversalFile f;
        f.rigBones = 7;
        f.montages.resize(1);
        f.poseBones.resize(2);
        std::string err;
        const bool ok = ParseTraversalText(m.second, f, err);
        Check(!ok && f.montages.empty() && f.poseBones.empty() && f.rigBones == 0 && !err.empty(),
              std::string("corrupt file accepted or output not cleared: ") + m.first);
    }
}

// trav: version 2 checks (see the header, item 3)
static void CheckMotion(const TraversalFile& aBaseStack, const char* aPath, const char* aTag)
{
    TraversalFile f;
    std::string err;
    const bool ok = LoadTraversalFile(aPath, f, err);
    Check(ok, std::string("load motion ") + aPath + ": " + err);
    Check(aBaseStack.version == 1 && aBaseStack.poseBones.empty(), std::string(aTag) + ": version 1 file has version 1, no pose bones");
    for (const auto& m : aBaseStack.montages)
        Check(!m.HasPose(), std::string(aTag) + ": version 1 montage without poses " + m.name);
    Check(f.version == 2, std::string(aTag) + " motion: version 2");
    Check(f.poseBones.size() == 73, std::string(aTag) + " motion: 73 pose bones, got " + std::to_string(f.poseBones.size()));
    for (const char* need : {"Root", "Trajectory", "Hips", "LeftHand", "RightHand"})
        Check(std::find(f.poseBones.begin(), f.poseBones.end(), std::string(need)) != f.poseBones.end(),
              std::string(aTag) + " motion: pose bone " + need);
    Check(f.rigName == aBaseStack.rigName && f.rigHash == aBaseStack.rigHash && f.rigBones == aBaseStack.rigBones, std::string(aTag) + " motion: rig as base");
    Check(f.montages.size() == 34 && f.montages.size() == aBaseStack.montages.size(), std::string(aTag) + " motion: 34 montages");
    size_t withPose = 0;
    double worstNorm = 0;
    for (size_t k = 0; k < std::min(f.montages.size(), aBaseStack.montages.size()); ++k)
    {
        const TravMontage& m = f.montages[k];
        const TravMontage& o = aBaseStack.montages[k];
        bool same = m.name == o.name && m.sequence == o.sequence && m.frames == o.frames && m.action == o.action &&
                    m.windows.size() == o.windows.size();
        for (size_t w = 0; same && w < m.windows.size(); ++w)
            same = m.windows[w].target == o.windows[w].target && m.windows[w].startFrame == o.windows[w].startFrame &&
                   m.windows[w].endFrame == o.windows[w].endFrame && m.windows[w].contactBone == o.windows[w].contactBone;
        Check(same, std::string(aTag) + " motion: montage / frames / windows as version 1: " + m.name);
        const bool sized = m.pose.size() == static_cast<size_t>(m.frames) * f.poseBones.size() * 7;
        Check(sized, std::string(aTag) + " motion: pose size " + m.name);
        withPose += sized && m.HasPose() ? 1 : 0;
        for (size_t i = 0; sized && i < m.pose.size(); i += 7)
        {
            const double n = std::sqrt(double(m.pose[i + 3]) * m.pose[i + 3] + double(m.pose[i + 4]) * m.pose[i + 4] +
                                       double(m.pose[i + 5]) * m.pose[i + 5] + double(m.pose[i + 6]) * m.pose[i + 6]);
            worstNorm = std::max(worstNorm, std::fabs(n - 1.0));
        }
    }
    Check(withPose == 34, std::string(aTag) + " motion: all 34 montages carry poses");
    Check(worstNorm <= 1e-5, std::string(aTag) + " motion: unit quaternions");
    std::printf("%s v2: version %d, %zu pose bones, %zu of %zu montages with poses, max |1-|q|| %.2e\n", aTag, f.version,
                f.poseBones.size(), withPose, f.montages.size(), worstNorm);
    std::string text;
    if (!ReadAll(aPath, text))
    {
        Check(false, std::string("read ") + aPath);
        return;
    }
    std::vector<std::pair<const char*, std::string>> muts;
    auto rep = [&](const char* aWhat, std::string s, const std::string& aFrom, const std::string& aTo) {
        const size_t p = s.find(aFrom);
        if (p == std::string::npos)
        {
            Check(false, std::string("motion mutation anchor missing: ") + aWhat);
            return;
        }
        s.replace(p, aFrom.size(), aTo);
        muts.emplace_back(aWhat, std::move(s));
    };
    {
        const size_t p = text.find("\nP ");
        const size_t e = p == std::string::npos ? p : text.find('\n', p + 1);
        if (p == std::string::npos || e == std::string::npos)
            Check(false, "motion: no P line");
        else
        {
            std::string drop = text;
            drop.erase(p, e - p); // first P line missing
            muts.emplace_back("missing P line", drop);
            const std::string line = text.substr(p, e - p);
            rep("short P line", text, line, line.substr(0, line.rfind(' ')));
            // qx of the first pose bone (4th value of the line) -> 5.000000: quaternion not unit
            size_t q = p + 3; // first value
            for (int k = 0; k < 3; ++k)
                q = text.find(' ', q) + 1;
            const size_t qe = text.find(' ', q);
            std::string nonUnit = text;
            nonUnit.replace(q, qe - q, "5.000000");
            muts.emplace_back("non-unit quaternion", nonUnit);
        }
    }
    rep("POSEBONES count", text, "POSEBONES 73 ", "POSEBONES 72 ");
    rep("duplicate pose bone", text, " Trajectory Hips ", " Trajectory Root ");
    rep("version 1 header on a version 2 body", text, "utpp-traversal 2", "utpp-traversal 1");
    Check(muts.size() == 6, "motion: 6 corruptions built");
    RejectAll(muts);
}

static void CheckBaseRejectsPoses(const std::string& aBaseText)
{
    std::vector<std::pair<const char*, std::string>> muts;
    std::string s = aBaseText;
    const size_t p = s.find("\nE\n");
    if (p == std::string::npos)
    {
        Check(false, "base anchor E");
        return;
    }
    s.insert(p, "\nP 0.000000 0.000000 0.000000 0.000000 0.000000 0.000000 1.000000");
    muts.emplace_back("P line in a version 1 file", s);
    RejectAll(muts);
}

int main(int argc, char** argv)
{
    if (argc != 3 && argc != 5)
    {
        std::printf("usage: utpp_mm_travfile_test <utpp_traversal_woman.utpptrav> <utpp_traversal_man.utpptrav>"
                    " [<utpp_traversal_woman_motion.utpptrav> <utpp_traversal_man_motion.utpptrav>]\n");
        return 2;
    }
    TraversalFile files[2];
    const char* tags[2] = {"woman", "man"};
    for (int i = 0; i < 2; ++i)
    {
        std::string err;
        const bool ok = LoadTraversalFile(argv[i + 1], files[i], err);
        Check(ok, std::string("load ") + argv[i + 1] + ": " + err);
        Check(files[i].montages.size() == 34, std::string(tags[i]) + ": 34 montages");
        Check(CountWindows(files[i]) == 55, std::string(tags[i]) + ": 55 warp windows");
        Check(files[i].rigName == std::string("player_") + tags[i] + "_skeleton", std::string(tags[i]) + ": rig name");
        std::printf("%s: %s rig %s 0x%016llx %d bones, %zu montages, %zu windows\n", tags[i], ok ? "loaded" : "FAILED",
                    files[i].rigName.c_str(), static_cast<unsigned long long>(files[i].rigHash), files[i].rigBones,
                    files[i].montages.size(), CountWindows(files[i]));
    }
    Check(files[0].rigHash != files[1].rigHash, "woman / man rig identity differs");
    Check(files[0].montages.size() == files[1].montages.size(), "woman / man montage count");
    for (size_t k = 0; k < std::min(files[0].montages.size(), files[1].montages.size()); ++k)
    {
        const auto& a = files[0].montages[k];
        const auto& b = files[1].montages[k];
        bool same = a.name == b.name && a.frames == b.frames && a.windows.size() == b.windows.size();
        for (size_t w = 0; same && w < a.windows.size(); ++w)
            same = a.windows[w].target == b.windows[w].target && a.windows[w].startFrame == b.windows[w].startFrame &&
                   a.windows[w].endFrame == b.windows[w].endFrame;
        Check(same, "woman / man montage + windows " + a.name);
    }
    std::string text;
    Check(ReadAll(argv[1], text), "read woman text for corruption tests");
    if (!text.empty())
    {
        CorruptionTests(text);
        CheckBaseRejectsPoses(text);
    }
    if (argc == 5) // trav: FORMAT_TRAVERSAL version 2
        for (int i = 0; i < 2; ++i)
            CheckMotion(files[i], argv[i + 3], tags[i]);

    const char* names[3] = {"AM_M_Neutral_Traversal_Hurdle_1_0_run_F_Lfoot", "AM_M_Neutral_Traversal_Mantle_1_0_run_F_Lfoot",
                            "AM_M_Neutral_Traversal_Vault_1_0_run_F_Lfoot"};
    const char* actions[3] = {"Hurdle", "Mantle", "Vault"};
    for (int i = 0; i < 2; ++i)
        for (int k = 0; k < 3; ++k)
        {
            const TravMontage* m = files[i].Find(names[k]);
            Check(m != nullptr, std::string(tags[i]) + ": montage " + names[k]);
            if (!m)
                continue;
            Check(m->action == actions[k], std::string(tags[i]) + " " + names[k] + ": action " + m->action);
            Check(m->rootZFromRtz, std::string(tags[i]) + " " + names[k] + ": root z from rtz");
            RunMontage(*m, tags[i]);
        }
    std::printf("utpp_mm_travfile_test: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
