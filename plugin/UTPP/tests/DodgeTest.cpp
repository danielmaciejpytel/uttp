// Ground dodge clips (src/mm/Dodge.hpp): sectors and hysteresis, the impulse feed pick, frame-rate independence of the clip
// clock (30 / 60 / 120 Hz), the data file validation (a damaged file is rejected). Optional arguments: utpp_dodge_woman.utpptrav
// and utpp_dodge_man.utpptrav (the baked files are validated against their own pose bones and a truncated copy is rejected).
#include "../src/mm/Dodge.hpp"

#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using namespace utpp::mm;
using dodge::Sector;

namespace
{
int g_fail = 0;
void Expect(bool aOk, const char* aWhat)
{
    std::printf("%s %s\n", aOk ? "ok  " : "FAIL", aWhat);
    if (!aOk)
        ++g_fail;
}

// minimal version 2 file: aBones pose bones, the four dodge montages (or fewer), aFrames frames each
std::string MakeFile(const std::vector<std::string>& aBones, int aMontages = 4, int aFrames = 3, float aQw = 1.0f)
{
    std::ostringstream o;
    o << "utpp-traversal 2\nRIG player_woman_skeleton 0x0000000000000001 152\nPOSEBONES " << aBones.size();
    for (const auto& b : aBones)
        o << ' ' << b;
    o << "\nCOUNT " << aMontages << '\n';
    for (int m = 0; m < aMontages; ++m)
    {
        char len[32];
        std::snprintf(len, sizeof len, "%.6f", (aFrames - 1) / 30.0);
        o << "M " << dodge::kMontageNames[m] << " clip" << m << ' ' << len
          << " 1.000000 0.080000 SmoothStep 0.200000 SmoothStep 1 -1.000000 Other 30.000000 " << aFrames << " 0\n";
        o << "SEC Default 0.000000 -\n";
        for (int f = 0; f < aFrames; ++f)
            o << "R 0.000000 0.000000 0.000000 0.000000\n";
        o << "C Distance_From_Ledge -\n";
        for (int f = 0; f < aFrames; ++f)
        {
            o << 'P';
            for (size_t b = 0; b < aBones.size(); ++b)
            {
                char row[160];
                std::snprintf(row, sizeof row, " %.6f 0.000000 %.6f 0.000000 0.000000 0.000000 %.6f", 0.01 * f, 0.1 * double(b), aQw);
                o << row;
            }
            o << '\n';
        }
        o << "E\n";
    }
    o << "END\n";
    return o.str();
}

struct Run
{
    int pickFeed = -1;   // feed index of the clip start
    Sector sector = Sector::None;
    float activeS = 0.0f; // feed time with the clip shown
    float clipAt02 = -1.0f; // clip time 0.2 s after the dodge's first feed
    int handbacks = 0, starts = 0;
};

// One dodge at aHz: the first dodge feed carries the old velocity (aOldV along +Y), the impulse arrives with the next feed and
// decays at 62 m/s^2 along aDirDeg (yaw, positive = left); the dodge detail lasts 0.2 s, then the capsule stands.
Run Simulate(float aHz, float aDirDeg, float aOldV, float aPeak, float aFacing = 0.0f, float aFwdMax = 3.5f)
{
    dodge::DodgePlayer p;
    const std::array<float, 4> len = {0.367f, 0.4f, 0.433f, 0.433f};
    const float dt = 1.0f / aHz;
    Run r;
    const float dx = -std::sin(aDirDeg * 0.0174533f), dy = std::cos(aDirDeg * 0.0174533f);
    for (int i = 0; i < int(aHz * 1.2f); ++i)
    {
        const float t = (i - 10) * dt; // dodge starts at feed 10
        dodge::Input in;
        in.dt = dt;
        in.standingGround = true;
        in.facingYawDeg = aFacing;
        in.forwardClipMaxEntryMps = aFwdMax;
        in.dodgeDetail = t >= 0.0f && t < 0.2f;
        if (t <= 0.0f)
            in.velX = 0.0f, in.velY = aOldV;
        else
        {
            const float s = std::fmax(0.0f, aPeak - 62.0f * (t - dt));
            in.velX = dx * s, in.velY = dy * s;
        }
        const dodge::Frame f = p.Tick(in, len);
        if (f.start)
        {
            ++r.starts;
            if (r.pickFeed < 0)
                r.pickFeed = i - 10, r.sector = f.sector;
        }
        r.handbacks += f.handback ? 1 : 0;
        if (f.active)
            r.activeS += dt;
        if (r.clipAt02 < 0.0f && t >= 0.2f - 1e-4f)
            r.clipAt02 = f.active ? f.clipTime : -2.0f;
    }
    return r;
}
} // namespace

int main(int argc, char** argv)
{
    // ---- sectors and hysteresis
    Expect(dodge::ClassifySector(0.0f) == Sector::Forward && dodge::ClassifySector(90.0f) == Sector::Left &&
               dodge::ClassifySector(-90.0f) == Sector::Right && dodge::ClassifySector(180.0f) == Sector::Back &&
               dodge::ClassifySector(-179.0f) == Sector::Back && dodge::ClassifySector(44.0f) == Sector::Forward &&
               dodge::ClassifySector(46.0f) == Sector::Left && dodge::ClassifySector(-136.0f) == Sector::Back && dodge::ClassifySector(530.0f) == Sector::Back,
           "sectors: borders at +-45 / +-135 deg, positive = left, wrapped");
    Expect(dodge::ClassifySector(50.0f, Sector::Forward) == Sector::Forward && dodge::ClassifySector(56.0f, Sector::Forward) == Sector::Left &&
               dodge::ClassifySector(40.0f, Sector::Left) == Sector::Left && dodge::ClassifySector(34.0f, Sector::Left) == Sector::Forward &&
               dodge::ClassifySector(-128.0f, Sector::Back) == Sector::Back && dodge::ClassifySector(-124.0f, Sector::Back) == Sector::Right &&
               dodge::ClassifySector(-40.0f, Sector::Right) == Sector::Right,
           "hysteresis: the previous sector is kept up to 10 deg past its border");

    // ---- the impulse feed picks the clip (the first dodge feed still carries the old velocity)
    {
        const Run r = Simulate(60.0f, -90.0f, 3.0f, 11.5f);
        std::printf("     side dodge from a walk: pick feed %d sector %s starts %d handbacks %d active %.3f s\n", r.pickFeed,
                    dodge::SectorName(r.sector), r.starts, r.handbacks, double(r.activeS));
        Expect(r.pickFeed == 1 && r.sector == Sector::Right && r.starts == 1 && r.handbacks == 1,
               "right dodge from a forward walk: Dodge_R picked on the impulse feed (not the first, old-velocity feed), one start, one hand-back");
        const Run b = Simulate(60.0f, 180.0f, 3.0f, 9.3f);
        Expect(b.sector == Sector::Back, "back dodge: Dodge_B");
        const Run fw = Simulate(60.0f, 0.0f, 8.2f, 15.0f);
        Expect(fw.pickFeed < 0 && fw.activeS == 0.0f && fw.handbacks == 0, "forward dodge from a sprint: no clip, the playing locomotion goes on");
        const Run fs = Simulate(60.0f, 0.0f, 1.5f, 11.0f);
        Expect(fs.sector == Sector::Forward, "forward dodge from a walk: Dodge_F");
        const Run rel = Simulate(60.0f, 0.0f, 0.0f, 11.0f, 90.0f);
        Expect(rel.sector == Sector::Right, "direction relative to the body facing: world forward with the body facing left = Dodge_R");
    }

    // ---- frame-rate independence: same clip, same clip time 0.2 s into the dodge, same shown time (within one feed)
    {
        const Run a = Simulate(30.0f, -90.0f, 3.0f, 11.5f), b = Simulate(60.0f, -90.0f, 3.0f, 11.5f), c = Simulate(120.0f, -90.0f, 3.0f, 11.5f);
        std::printf("     30 / 60 / 120 Hz: clip time at 0.2 s %.3f / %.3f / %.3f, shown %.3f / %.3f / %.3f s\n", double(a.clipAt02),
                    double(b.clipAt02), double(c.clipAt02), double(a.activeS), double(b.activeS), double(c.activeS));
        const float expect = 0.433f / dodge::kRate;
        const bool same = a.sector == b.sector && b.sector == c.sector && a.sector == Sector::Right;
        const bool clock = std::fabs(a.clipAt02 - b.clipAt02) <= dodge::kRate / 30.0f + 1e-3f && std::fabs(c.clipAt02 - b.clipAt02) <= dodge::kRate / 60.0f + 1e-3f;
        const bool shown = std::fabs(a.activeS - expect) <= 2.0f / 30.0f && std::fabs(b.activeS - expect) <= 2.0f / 60.0f &&
                           std::fabs(c.activeS - expect) <= 2.0f / 120.0f;
        Expect(same && clock && shown, "30 / 60 / 120 Hz: same clip, clip clock in time (kRate), shown for length / rate within the impulse latency");
    }

    // ---- leaving the standing ground state hands back at once
    {
        dodge::DodgePlayer p;
        const std::array<float, 4> len = {0.4f, 0.4f, 0.4f, 0.4f};
        dodge::Input in;
        in.dt = 1.0f / 60.0f;
        in.standingGround = true;
        in.forwardClipMaxEntryMps = 3.5f;
        in.dodgeDetail = true;
        p.Tick(in, len);
        in.velX = 10.0f;
        const dodge::Frame f1 = p.Tick(in, len);
        in.standingGround = false;
        const dodge::Frame f2 = p.Tick(in, len);
        Expect(f1.active && f1.sector == Sector::Right && !f2.active && f2.handback && !p.Active(),
               "air / slide / crouch / traversal during the clip: hand-back on that feed");
    }

    // ---- data file validation
    {
        const std::vector<std::string> bones = {"Root", "Hips", "Spine"};
        const auto name = [&](uint32_t i) { return std::string_view(bones[i]); };
        traversal::TraversalFile f;
        std::string err;
        const bool parsed = traversal::ParseTraversalText(MakeFile(bones), f, err);
        dodge::DodgeClips c;
        const bool ok = parsed && dodge::BuildDodgeClips(f, "player_woman_skeleton", 3, name, c);
        Expect(ok && c.ok && std::fabs(c.length[2] - 2.0f / 30.0f) < 1e-5f, "valid file: four clips accepted");
        float out[21];
        Expect(ok && dodge::SampleDodge(c, Sector::Back, 1.5 / 30.0, out) && std::fabs(out[7 + 0] - 0.015f) < 1e-5f && std::fabs(out[7 * 2 + 2] - 0.2f) < 1e-5f,
               "sampling: frame lerp in the database bone order");
        traversal::TraversalFile f3;
        dodge::DodgeClips c3;
        Expect(traversal::ParseTraversalText(MakeFile(bones, 3), f3, err) && !dodge::BuildDodgeClips(f3, "player_woman_skeleton", 3, name, c3) && !c3.ok,
               "rejected: a dodge montage missing");
        dodge::DodgeClips c4;
        const std::vector<std::string> dbMore = {"Root", "Hips", "Spine", "Neck"};
        const auto name4 = [&](uint32_t i) { return std::string_view(dbMore[i]); };
        Expect(!dodge::BuildDodgeClips(f, "player_woman_skeleton", 4, name4, c4) && !c4.ok, "rejected: a database bone without a pose bone");
        dodge::DodgeClips c5;
        Expect(!dodge::BuildDodgeClips(f, "player_man_skeleton", 3, name, c5), "rejected: another body's rig");
        traversal::TraversalFile f6;
        Expect(!traversal::ParseTraversalText(MakeFile(bones, 4, 3, 0.9f), f6, err), "rejected: a quaternion that is not unit length");
        std::string trunc = MakeFile(bones);
        trunc.resize(trunc.size() / 2);
        traversal::TraversalFile f7;
        Expect(!traversal::ParseTraversalText(trunc, f7, err), "rejected: a truncated file");
    }

    // ---- the baked files (optional arguments)
    for (int a = 1; a < argc; ++a)
    {
        traversal::TraversalFile f;
        std::string err;
        if (!traversal::LoadTraversalFile(argv[a], f, err))
        {
            std::printf("FAIL load %s: %s\n", argv[a], err.c_str());
            ++g_fail;
            continue;
        }
        const std::string rig = f.rigName;
        const auto name = [&](uint32_t i) { return std::string_view(f.poseBones[i]); };
        dodge::DodgeClips c;
        const bool ok = dodge::BuildDodgeClips(f, rig, static_cast<uint32_t>(f.poseBones.size()), name, c);
        std::printf("     %s: %s bones %zu lengths F %.3f B %.3f L %.3f R %.3f\n", argv[a], ok ? "ok" : c.err.c_str(), f.poseBones.size(),
                    double(c.length[0]), double(c.length[1]), double(c.length[2]), double(c.length[3]));
        Expect(ok && f.poseBones.size() == 73, "baked file: four clips over the 73 database bones");
        std::ifstream in(argv[a], std::ios::binary);
        std::stringstream ss;
        ss << in.rdbuf();
        std::string t = ss.str();
        t.resize(t.size() - 200);
        traversal::TraversalFile ft;
        Expect(!traversal::ParseTraversalText(t, ft, err), "baked file truncated: rejected");
    }
    std::printf("%s (%d failures)\n", g_fail ? "FAILED" : "PASSED", g_fail);
    return g_fail ? 1 : 0;
}
