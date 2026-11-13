#pragma once
// Test helper: reads a continuous helper-bone CSV capture (utpp-bodytrace-*.csv, legacy stages) into frames. Used by the compact pose
// capture tests (the slice cut by tools/analysis/make_pose_capture_fixture.py) and the measurement tool.
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "../src/mm/BodyDeformerTrace.hpp"

namespace posefixture
{
struct CsvCapture
{
    int64_t qpcHz = 0, armTicks = 0;
    uint16_t utc[7] = {0, 0, 0, 0, 0, 0, 0};
    uint32_t maxFrames = 0;
    std::vector<uint64_t> hashes;
    std::vector<int16_t> parents;
    std::vector<utpp::mm::bodytrace::Frame> frames;
    utpp::mm::bodytrace::Map map;
    bool ok = false;
};

// Reads at most aMaxFrames whole frames (0 = all) after skipping aSkipFrames frames. Frame fields come from the rows; Frame::index from the gameIndex column.
inline CsvCapture ReadCsv(const std::string& aPath, size_t aMaxFrames, size_t aSkipFrames = 0)
{
    using namespace utpp::mm::bodytrace;
    CsvCapture cap;
    std::ifstream in(aPath, std::ios::binary);
    if (!in)
        return cap;
    std::string line;
    std::unique_ptr<Frame> cur;
    uint32_t row = 0;
    size_t skipRows = 0;
    while (std::getline(in, line))
    {
        if (!line.empty() && line.back() == '\r')
            line.pop_back();
        if (line.rfind("# qpc_hz=", 0) == 0)
        {
            long long hz = 0, arm = 0;
            sscanf_s(line.c_str(), "# qpc_hz=%lld arm_ticks=%lld", &hz, &arm);
            cap.qpcHz = hz;
            cap.armTicks = arm;
            unsigned y = 0, mo = 0, d = 0, h = 0, mi = 0, sec = 0, ms = 0, maxFrames = 0;
            if (const char* u = std::strstr(line.c_str(), "utc="))
                sscanf_s(u, "utc=%u-%u-%uT%u:%u:%u.%uZ", &y, &mo, &d, &h, &mi, &sec, &ms);
            if (const char* m = std::strstr(line.c_str(), "max_frames="))
                sscanf_s(m, "max_frames=%u", &maxFrames);
            const unsigned parts[7] = {y, mo, d, h, mi, sec, ms};
            for (int i = 0; i < 7; ++i)
                cap.utc[i] = static_cast<uint16_t>(parts[i]);
            cap.maxFrames = maxFrames;
            continue;
        }
        if (line.rfind("# rig_bone ", 0) == 0)
        {
            unsigned index = 0;
            unsigned long long hash = 0;
            int parent = 0;
            sscanf_s(line.c_str(), "# rig_bone index=%u hash=%llX parent=%d", &index, &hash, &parent);
            cap.hashes.push_back(hash);
            cap.parents.push_back(static_cast<int16_t>(parent));
            continue;
        }
        if (line.empty() || line[0] == '#' || line.rfind("ticks,", 0) == 0)
            continue;
        if (aMaxFrames && cap.frames.size() >= aMaxFrames)
            break;
        if (skipRows < aSkipFrames * kMaxBones)
        {
            ++skipRows;
            continue;
        }
        // ticks,desc,thread,rig,generation,rigCount,name,gameIndex,s0_0..s0_11,s1_0..s1_11
        if (row == 0)
            cur = std::make_unique<Frame>();
        const char* p = line.c_str();
        char* end = nullptr;
        const long long ticks = std::strtoll(p, &end, 10);
        p = end + 1;
        const unsigned long long desc = std::strtoull(p, &end, 10);
        p = end + 1;
        const unsigned long thread = std::strtoul(p, &end, 10);
        p = end + 1;
        const unsigned long long rig = std::strtoull(p, &end, 10);
        p = end + 1;
        const unsigned long long generation = std::strtoull(p, &end, 10);
        p = end + 1;
        const unsigned long rigCount = std::strtoul(p, &end, 10);
        p = end + 1;
        const char* comma = std::strchr(p, ',');
        p = comma + 1;
        const long gameIndex = std::strtol(p, &end, 10);
        p = end;
        cur->ticks = ticks;
        cur->desc = desc;
        cur->thread = static_cast<uint32_t>(thread);
        cur->rig = rig;
        cur->generation = generation;
        cur->rigCount = static_cast<uint32_t>(rigCount);
        cur->index[row] = static_cast<int32_t>(gameIndex);
        for (uint32_t s = 0; s < 2; ++s)
            for (uint32_t v = 0; v < 12; ++v)
            {
                p = std::strchr(p, ',') + 1;
                cur->stage[s][row][v] = std::strtof(p, &end);
            }
        if (++row == kMaxBones)
        {
            row = 0;
            if (cap.frames.empty())
            {
                cap.map.rig = cur->rig;
                cap.map.rigCount = cur->rigCount;
                cap.map.index = cur->index;
            }
            cap.frames.push_back(*cur);
        }
    }
    cap.ok = !cap.frames.empty();
    return cap;
}
} // namespace posefixture
