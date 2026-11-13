#pragma once
// UTPP motion stage 10.8 part 2 - reader of the .utpptrav traversal montage files (tools/mmbake/FORMAT_TRAVERSAL.txt
// version 1, written by tools/mmbake/travbake.py; motion-derived data).
// trav: also version 2 (travbake.py --format 2) = version 1 + "POSEBONES" + per montage one P line per frame
// (local pose, 7 floats per pose bone); a version 1 file parses exactly as before (version 1, no poses).
// Fail closed: any deviation from the spec (header, token count, float format "%.6f", flags, frame ranges, window
// order, contact bone rule, root line count, curve length, pose lines / bones / unit quaternions, END, trailing data,
// CR bytes) returns false with an error and leaves the output empty. Header only, no game dependency.
#include "BaseTraversal.hpp"

#include <charconv>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

namespace utpp::mm::traversal
{
struct TravSection
{
    std::string name;
    double startS = 0;
    std::string next; // "-" = none
};

struct TravWindow
{
    std::string target; // FrontLedge | BackLedge | BackFloor
    double startS = 0, endS = 0;
    int startFrame = 0, endFrame = 0;
    std::string modifierClass;
    bool warpTranslation = true, warpRotation = true, ignoreZ = false, warpToFeet = false;
    std::string rotationType;
    double rotationTimeMultiplier = 1;
    std::string provider, ueBone, contactBone; // contactBone: LeftHand | RightHand | Root
    V3 offset;                                 // contact bone - root at endFrame, root space, metres
    double maxSpeedClampRatio = 0;
};

struct TravMontage
{
    std::string name, sequence;
    double lengthS = 0, rateScale = 1, blendInS = 0;
    std::string blendInOption;
    double blendOutS = 0;
    std::string blendOutOption;
    bool autoBlendOut = true;
    double blendOutTriggerS = -1;
    std::string action; // Hurdle | Vault | Mantle | ClimbStart | Catch | Other
    double fps = 30;
    int frames = 0;
    bool rootZFromRtz = false;
    std::vector<TravSection> sections;
    std::vector<TravWindow> windows;
    std::vector<RootKey> root;              // frames entries: x y z (m, CP axes), yaw (rad about +Z)
    std::vector<double> distanceFromLedge;  // empty or frames entries (UE units)
    // version 2 only: frames x poseBones x 7 (x y z qx qy qz qw, bone order = TraversalFile::poseBones, local transforms
    // of the retarget clip frame = the .utppmm POSE convention); empty in a version 1 file
    std::vector<float> pose;
    bool HasPose() const { return !pose.empty(); }
};

struct TraversalFile
{
    int version = 1;
    std::string rigName;
    uint64_t rigHash = 0;
    int rigBones = 0;
    std::vector<std::string> poseBones; // version 2: bone names of the P lines (empty in version 1)
    std::vector<TravMontage> montages;

    const TravMontage* Find(std::string_view aName) const
    {
        for (const auto& m : montages)
            if (m.name == aName)
                return &m;
        return nullptr;
    }
};

namespace travfile_detail
{
inline std::vector<std::string_view> Split(std::string_view aLine)
{
    std::vector<std::string_view> t;
    size_t p = 0;
    while (p <= aLine.size())
    {
        const size_t q = aLine.find(' ', p);
        const size_t e = q == std::string_view::npos ? aLine.size() : q;
        t.push_back(aLine.substr(p, e - p));
        if (q == std::string_view::npos)
            break;
        p = q + 1;
    }
    return t;
}

// "%.6f" only: optional '-', digits, '.', exactly 6 digits; "-0.000000" is not allowed.
inline bool Float(std::string_view aTok, double& aOut)
{
    const size_t dot = aTok.find('.');
    if (dot == std::string_view::npos || aTok.size() - dot - 1 != 6 || aTok == "-0.000000")
        return false;
    size_t i = aTok[0] == '-' ? 1 : 0;
    if (i >= dot)
        return false;
    for (size_t k = i; k < aTok.size(); ++k)
        if (k != dot && (aTok[k] < '0' || aTok[k] > '9'))
            return false;
    const auto r = std::from_chars(aTok.data(), aTok.data() + aTok.size(), aOut);
    return r.ec == std::errc() && r.ptr == aTok.data() + aTok.size() && std::isfinite(aOut);
}

inline bool Int(std::string_view aTok, int& aOut)
{
    if (aTok.empty() || aTok.size() > 9)
        return false;
    for (char c : aTok)
        if (c < '0' || c > '9')
            return false;
    const auto r = std::from_chars(aTok.data(), aTok.data() + aTok.size(), aOut);
    return r.ec == std::errc() && r.ptr == aTok.data() + aTok.size();
}

inline bool Flag(std::string_view aTok, bool& aOut)
{
    if (aTok != "0" && aTok != "1")
        return false;
    aOut = aTok == "1";
    return true;
}

inline bool Name(std::string_view aTok) { return !aTok.empty(); }
} // namespace travfile_detail

inline bool ParseTraversalText(const std::string& aText, TraversalFile& aOut, std::string& aErr)
{
    using namespace travfile_detail;
    aOut = TraversalFile{};
    TraversalFile f;
    std::vector<std::string_view> lines;
    {
        const std::string_view all(aText);
        if (all.empty() || all.back() != '\n' || all.find('\r') != std::string_view::npos ||
            all.find('\0') != std::string_view::npos)
        {
            aErr = "not LF-terminated ASCII text";
            return false;
        }
        size_t p = 0;
        while (p < all.size())
        {
            const size_t q = all.find('\n', p);
            lines.push_back(all.substr(p, q - p));
            p = q + 1;
        }
    }
    size_t li = 0;
    std::vector<std::string_view> t;
    auto next = [&]() -> bool {
        if (li >= lines.size())
            return false;
        t = Split(lines[li++]);
        for (auto& s : t)
            if (s.empty())
                return false; // leading / trailing / double spaces
        return true;
    };
    auto fail = [&](const std::string& aWhat) {
        aErr = "line " + std::to_string(li) + ": " + aWhat;
        return false;
    };
    if (!next() || t.size() != 2 || t[0] != "utpp-traversal" || (t[1] != "1" && t[1] != "2"))
        return fail("header 'utpp-traversal 1' or 'utpp-traversal 2' expected");
    f.version = t[1] == "2" ? 2 : 1;
    if (!next() || t.size() != 4 || t[0] != "RIG" || t[2].size() != 18 || t[2].substr(0, 2) != "0x")
        return fail("RIG <name> 0x<16 hex> <bones> expected");
    {
        for (char c : t[2].substr(2))
            if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f')))
                return fail("rig hash hex");
        const auto r = std::from_chars(t[2].data() + 2, t[2].data() + t[2].size(), f.rigHash, 16);
        if (r.ec != std::errc() || !Int(t[3], f.rigBones) || f.rigBones <= 0)
            return fail("rig hash / bone count");
        f.rigName = std::string(t[1]);
    }
    if (f.version == 2)
    {
        int nb = 0;
        if (!next() || t.size() < 3 || t[0] != "POSEBONES" || !Int(t[1], nb) || nb <= 0 || nb > f.rigBones ||
            t.size() != static_cast<size_t>(nb) + 2)
            return fail("POSEBONES <n> <names> expected");
        for (size_t k = 2; k < t.size(); ++k)
        {
            for (const std::string& b : f.poseBones)
                if (b == t[k])
                    return fail("duplicate pose bone " + std::string(t[k]));
            f.poseBones.emplace_back(t[k]);
        }
    }
    int count = 0;
    if (!next() || t.size() != 2 || t[0] != "COUNT" || !Int(t[1], count) || count <= 0)
        return fail("COUNT <n> expected");
    for (int mi = 0; mi < count; ++mi)
    {
        TravMontage m;
        int autoBO = 0, rtz = 0;
        if (!next() || t.size() != 15 || t[0] != "M")
            return fail("M line with 15 tokens expected");
        m.name = std::string(t[1]);
        m.sequence = std::string(t[2]);
        m.blendInOption = std::string(t[6]);
        m.blendOutOption = std::string(t[8]);
        m.action = std::string(t[11]);
        if (!Float(t[3], m.lengthS) || !Float(t[4], m.rateScale) || !Float(t[5], m.blendInS) ||
            !Float(t[7], m.blendOutS) || !Flag(t[9], m.autoBlendOut) || !Float(t[10], m.blendOutTriggerS) ||
            !Float(t[12], m.fps) || !Int(t[13], m.frames) || !Flag(t[14], m.rootZFromRtz))
            return fail("M field format (" + m.name + ")");
        (void)autoBO;
        (void)rtz;
        if (m.lengthS <= 0 || m.rateScale <= 0 || m.blendInS < 0 || m.blendOutS < 0 || m.fps <= 0 || m.frames < 2)
            return fail("M value range (" + m.name + ")");
        if (m.action != "Hurdle" && m.action != "Vault" && m.action != "Mantle" && m.action != "ClimbStart" &&
            m.action != "Catch" && m.action != "Other")
            return fail("M action (" + m.name + ")");
        if (f.Find(m.name))
            return fail("duplicate montage " + m.name);
        if (!next())
            return fail("SEC expected");
        while (t[0] == "SEC")
        {
            TravSection s;
            if (t.size() != 4 || !Float(t[2], s.startS) || s.startS < 0)
                return fail("SEC <name> <start> <next>");
            s.name = std::string(t[1]);
            s.next = std::string(t[3]);
            m.sections.push_back(std::move(s));
            if (!next())
                return fail("truncated after SEC");
        }
        if (m.sections.empty())
            return fail("at least one SEC line expected (" + m.name + ")");
        while (t[0] == "W")
        {
            TravWindow w;
            if (t.size() != 20)
                return fail("W line with 20 tokens expected (" + m.name + ")");
            w.target = std::string(t[1]);
            w.modifierClass = std::string(t[6]);
            w.rotationType = std::string(t[11]);
            w.provider = std::string(t[13]);
            w.ueBone = std::string(t[14]);
            w.contactBone = std::string(t[15]);
            if (!Float(t[2], w.startS) || !Float(t[3], w.endS) || !Int(t[4], w.startFrame) || !Int(t[5], w.endFrame) ||
                !Flag(t[7], w.warpTranslation) || !Flag(t[8], w.warpRotation) || !Flag(t[9], w.ignoreZ) ||
                !Flag(t[10], w.warpToFeet) || !Float(t[12], w.rotationTimeMultiplier) || !Float(t[16], w.offset.x) ||
                !Float(t[17], w.offset.y) || !Float(t[18], w.offset.z) || !Float(t[19], w.maxSpeedClampRatio))
                return fail("W field format (" + m.name + ")");
            if (w.target != "FrontLedge" && w.target != "BackLedge" && w.target != "BackFloor")
                return fail("W target " + w.target);
            if (w.endS < w.startS || w.startFrame > w.endFrame || w.endFrame >= m.frames)
                return fail("W range (" + m.name + ")");
            if (!m.windows.empty() && w.startS < m.windows.back().startS)
                return fail("W not sorted by start (" + m.name + ")");
            const bool floor = w.target == "BackFloor";
            if (floor ? (w.contactBone != "Root" || w.offset.x != 0 || w.offset.y != 0 || w.offset.z != 0)
                      : (w.contactBone != "LeftHand" && w.contactBone != "RightHand"))
                return fail("W contact bone rule (" + m.name + " " + w.target + ")");
            if (std::fabs(w.offset.x) > 3 || std::fabs(w.offset.y) > 3 || std::fabs(w.offset.z) > 3)
                return fail("W contact offset > 3 m (" + m.name + ")");
            m.windows.push_back(std::move(w));
            if (!next())
                return fail("truncated after W");
        }
        m.root.reserve(static_cast<size_t>(m.frames));
        for (int k = 0; k < m.frames; ++k)
        {
            if (k > 0 && !next())
                return fail("truncated root track (" + m.name + ")");
            RootKey r;
            if (t.size() != 5 || t[0] != "R" || !Float(t[1], r.pos.x) || !Float(t[2], r.pos.y) ||
                !Float(t[3], r.pos.z) || !Float(t[4], r.yaw))
                return fail("R <x> <y> <z> <yaw> expected (" + m.name + " frame " + std::to_string(k) + ")");
            m.root.push_back(r);
        }
        if (!next() || t.size() < 3 || t[0] != "C" || t[1] != "Distance_From_Ledge")
            return fail("C Distance_From_Ledge expected (" + m.name + ")");
        if (!(t.size() == 3 && t[2] == "-"))
        {
            if (t.size() != static_cast<size_t>(m.frames) + 2)
                return fail("Distance_From_Ledge length (" + m.name + ")");
            m.distanceFromLedge.resize(static_cast<size_t>(m.frames));
            for (int k = 0; k < m.frames; ++k)
                if (!Float(t[static_cast<size_t>(k) + 2], m.distanceFromLedge[static_cast<size_t>(k)]))
                    return fail("Distance_From_Ledge value (" + m.name + ")");
        }
        if (f.version == 2)
        {
            const size_t nb = f.poseBones.size();
            m.pose.resize(static_cast<size_t>(m.frames) * nb * 7);
            for (int k = 0; k < m.frames; ++k)
            {
                if (!next() || t.size() != nb * 7 + 1 || t[0] != "P")
                    return fail("P line with 7 x " + std::to_string(nb) + " values expected (" + m.name + " frame " +
                                std::to_string(k) + ")");
                float* dst = &m.pose[static_cast<size_t>(k) * nb * 7];
                for (size_t i = 0; i < nb * 7; ++i)
                {
                    double v = 0;
                    if (!Float(t[i + 1], v))
                        return fail("P value (" + m.name + ")");
                    dst[i] = static_cast<float>(v);
                }
                for (size_t b = 0; b < nb; ++b)
                {
                    const float* q = dst + b * 7 + 3;
                    const double n = std::sqrt(double(q[0]) * q[0] + double(q[1]) * q[1] + double(q[2]) * q[2] + double(q[3]) * q[3]);
                    if (!(std::fabs(n - 1.0) <= 1e-4))
                        return fail("P quaternion not unit (" + m.name + ")");
                }
            }
        }
        if (!next() || t.size() != 1 || t[0] != "E")
            return fail("E expected (" + m.name + ")");
        f.montages.push_back(std::move(m));
    }
    if (!next() || t.size() != 1 || t[0] != "END")
        return fail("END expected");
    if (li != lines.size())
        return fail("data after END");
    aOut = std::move(f);
    aErr.clear();
    return true;
}

inline bool LoadTraversalFile(const std::string& aPath, TraversalFile& aOut, std::string& aErr)
{
    aOut = TraversalFile{};
    std::ifstream in(aPath, std::ios::binary);
    if (!in)
    {
        aErr = "cannot open " + aPath;
        return false;
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    return ParseTraversalText(ss.str(), aOut, aErr);
}

// SolveWarp needs sorted, non-overlapping windows (a window may start on the end frame of the previous one).
inline bool WindowsSolvable(const TravMontage& aM)
{
    for (size_t i = 1; i < aM.windows.size(); ++i)
        if (aM.windows[i].startFrame < aM.windows[i - 1].endFrame)
            return false;
    return true;
}

// Warp point of a window in world space for a root key (root + RotZ(yaw) * offset).
inline V3 WarpPointWorld(const RootKey& aRoot, const TravWindow& aW) { return aRoot.pos + RotZ(aW.offset, aRoot.yaw); }

// Builds SolveWarp windows from the baked windows with the given world targets (one per window, same order).
inline std::vector<WarpWindow> MakeWarpWindows(const TravMontage& aM, const std::vector<RootKey>& aTargets)
{
    std::vector<WarpWindow> out;
    for (size_t i = 0; i < aM.windows.size() && i < aTargets.size(); ++i)
    {
        const TravWindow& w = aM.windows[i];
        WarpWindow ww;
        ww.startFrame = w.startFrame;
        ww.endFrame = w.endFrame;
        ww.target = aTargets[i].pos;
        ww.targetYaw = aTargets[i].yaw;
        ww.warpPointOffset = w.offset;
        ww.warpTranslation = w.warpTranslation;
        ww.warpYaw = w.warpRotation;
        ww.ignoreZ = w.ignoreZ;
        out.push_back(ww);
    }
    return out;
}
} // namespace utpp::mm::traversal
