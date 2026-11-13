// Offline tool of the hood slide comparison film (tools/films/vaulttorso_film.py): runs the production chain of src/mm/VaultTorso.hpp (gate ->
// fade with the exit hold -> limiter on the local records) over captured frames and writes the corrected records. Not a test (EXCLUDE_FROM_ALL).
// Usage: utpp_vaulttorso_film_tool <in.txt> <out.txt>
// Input (text): "rig N" + N lines "bone idx name parent" (the first 14 bones: Root .. Spine3), then per vault
//   "vault <label> weapon=<0|1> detail=<n> release=<0|1> on=<s> off=<s> frames=<N> bones=<B>" and N lines "f <t> <B*7 floats: pos xyz, quat xyzw>"
// Output: the same structure with the corrected records (frames of a vault whose gate is off come out unchanged).
#include "../src/mm/VaultTorso.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using namespace utpp::mm;
namespace vt = utpp::mm::vaulttorso;

int main(int argc, char** argv)
{
    if (argc < 3)
        return 2;
    std::ifstream in(argv[1]);
    std::ofstream out(argv[2]);
    if (!in || !out)
        return 2;
    std::vector<std::string> names;
    std::vector<int16_t> parents;
    std::string line;
    vt::Rig rig;
    bool rigDone = false;
    while (std::getline(in, line))
    {
        std::istringstream is(line);
        std::string tag;
        is >> tag;
        if (tag == "bone")
        {
            int i, p;
            std::string n;
            is >> i >> n >> p;
            names.push_back(n);
            parents.push_back(static_cast<int16_t>(p));
            out << line << "\n";
        }
        else if (tag == "rig")
            out << line << "\n";
        else if (tag == "vault")
        {
            if (!rigDone)
            {
                std::vector<uint64_t> hashes;
                for (const std::string& n : names)
                    hashes.push_back(integ::CNameHash(n));
                rig = vt::ResolveRig(hashes, parents);
                rigDone = true;
                if (!rig.ok)
                    return 3;
            }
            std::istringstream hs(line);
            std::string label, kv;
            hs >> tag >> label;
            int weapon = 0, detail = 0, release = 0, frames = 0, bones = 0;
            float on = 0, off = 0;
            while (hs >> kv)
            {
                const size_t eq = kv.find('=');
                const std::string k = kv.substr(0, eq);
                const double v = std::atof(kv.c_str() + eq + 1);
                if (k == "weapon") weapon = int(v);
                else if (k == "detail") detail = int(v);
                else if (k == "release") release = int(v);
                else if (k == "on") on = float(v);
                else if (k == "off") off = float(v);
                else if (k == "frames") frames = int(v);
                else if (k == "bones") bones = int(v);
            }
            out << line << "\n";
            vt::Fade fade;
            float prevT = 0.0f, lastElev = 90.0f;
            bool have = false, first = true, episode = false;
            std::vector<uint8_t> buf(size_t(bones) * integ::kGameBoneStride);
            for (int fidx = 0; fidx < frames; ++fidx)
            {
                std::getline(in, line);
                std::istringstream fs(line);
                std::string ft;
                float t;
                fs >> ft >> t;
                std::vector<float> rec(size_t(bones) * 7);
                for (float& x : rec)
                    fs >> x;
                std::memset(buf.data(), 0, buf.size());
                for (int b = 0; b < bones; ++b)
                {
                    float* r = reinterpret_cast<float*>(buf.data() + size_t(b) * integ::kGameBoneStride);
                    r[0] = rec[size_t(b) * 7 + 0]; r[1] = rec[size_t(b) * 7 + 1]; r[2] = rec[size_t(b) * 7 + 2]; r[3] = 1.0f;
                    r[4] = rec[size_t(b) * 7 + 3]; r[5] = rec[size_t(b) * 7 + 4]; r[6] = rec[size_t(b) * 7 + 5]; r[7] = rec[size_t(b) * 7 + 6];
                    r[8] = r[9] = r[10] = r[11] = 1.0f;
                }
                vt::GateInput gi;
                gi.travRelease = release != 0 && t >= on && t < off;
                gi.psmDetail = gi.travRelease ? detail : -1; // after the release the PSM is back on plain locomotion
                gi.weaponDrawn = weapon != 0;
                gi.vehicleVaultRanged = weapon != 0 && detail == 9;
                const float dt = first ? 0.0f : t - prevT;
                first = false;
                prevT = t;
                if (vt::Wanted(gi))
                    episode = true;
                const bool hold = !vt::Wanted(gi) && vt::HoldAllowed(gi, episode) && have && lastElev < vt::kFloorDeg;
                const float w = fade.Step(dt, vt::Wanted(gi), hold);
                if (!(w > 0.0f))
                    episode = false;
                float e = 0.0f;
                if (vt::MeasureElevation(buf.data(), uint32_t(bones), rig, e))
                {
                    lastElev = e;
                    have = true;
                }
                vt::Apply(buf.data(), uint32_t(bones), rig, w);
                out << "f " << t;
                char tmp[64];
                for (int b = 0; b < bones; ++b)
                {
                    const float* r = reinterpret_cast<const float*>(buf.data() + size_t(b) * integ::kGameBoneStride);
                    const float v[7] = {r[0], r[1], r[2], r[4], r[5], r[6], r[7]};
                    for (float x : v)
                    {
                        std::snprintf(tmp, sizeof tmp, " %.8g", static_cast<double>(x));
                        out << tmp;
                    }
                }
                out << "\n";
            }
        }
    }
    return 0;
}
