#include "../src/mm/PoseTrace.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <vector>
using namespace utpp::mm;
int main(int argc, char** argv)
{
    if (argc != 2) return 2;
    std::filesystem::create_directories(argv[1]);
    posetrace::Frame f{};
    f.snap.numBones = 2; f.snap.gameIndex[0] = 1; f.snap.gameIndex[1] = 0;
    float records[24]; for (int i = 0; i < 24; ++i) records[i] = float(i + 1);
    if (!posetrace::Capture(f, 0, reinterpret_cast<uint8_t*>(records), 2)) return 3;
    if (f.stage[0][0][8] != 21 || f.stage[0][1][4] != 5) return 4;
    if (posetrace::Capture(f, -1, reinterpret_cast<uint8_t*>(records), 2) ||
        posetrace::Capture(f, 3, reinterpret_cast<uint8_t*>(records), 2)) return 12;
    if (!posetrace::Capture(f, 1, reinterpret_cast<uint8_t*>(records), 1) ||
        !std::isnan(f.stage[1][0][0]) || f.stage[1][1][0] != 1) return 5;
    if (posetrace::Capture(f, 1, reinterpret_cast<uint8_t*>(records), 0)) return 17;
    if (!posetrace::Capture(f, 1, reinterpret_cast<uint8_t*>(records), 2) ||
        !posetrace::Capture(f, 2, reinterpret_cast<uint8_t*>(records), 2)) return 6;
    const uint32_t validBones = f.snap.numBones;
    f.snap.numBones = integ::kMaxMMBones + 1;
    if (posetrace::Capture(f, 0, reinterpret_cast<uint8_t*>(records), 2)) return 13;
    f.snap.numBones = validBones;
    auto dumper = std::make_unique<posetrace::Dumper>();
    auto& d = *dumper;
    if (!d.Start(std::filesystem::path(argv[1]).wstring())) return 7;
    const auto csvPath = d.Path();
    if (csvPath.empty()) return 14;
    const auto headerBytes = std::filesystem::file_size(csvPath);
    if (headerBytes == 0) return 15;
    d.Push(f);
    d.Push(f);
    std::vector<std::thread> writers;
    for (int t = 0; t < 4; ++t) writers.emplace_back([&] { for (int i = 0; i < 25; ++i) d.Push(f); });
    for (auto& t : writers) t.join();
    bool flushedBeforeStop = false;
    for (int waited = 0; waited < 2000; waited += 20)
    {
        if (std::filesystem::file_size(csvPath) > headerBytes) { flushedBeforeStop = true; break; }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if (!flushedBeforeStop) return 16;
    d.Stop();
    std::cout << d.Status() << '\n';
    if (d.Enabled()) return 8;
    bool found = false;
    {
        std::ifstream in(csvPath); std::string line; int rows = 0;
        while (std::getline(in, line))
        {
            if (line.starts_with("# frames=")) found = line.find("failed=0") != std::string::npos;
            else if (!line.empty() && line[0] != '#' && !line.starts_with("ticks,"))
            {
                if (std::count(line.begin(), line.end(), ',') != 57) return 9;
                ++rows;
            }
        }
        if (rows < 4 || rows % 2) return 10;
    }
    if (!found) return 11;
    // Regression: a long session must continue beyond the former 7200-frame cap.
    // Two bones keep this focused writer check small; the RAM ring is unchanged.
    auto longDumper = std::make_unique<posetrace::Dumper>();
    auto& longSession = *longDumper;
    if (!longSession.Start(std::filesystem::path(argv[1]).wstring())) return 18;
    const auto longPath = longSession.Path();
    for (int i = 0; i < 7204; ++i)
    {
        f.ticks = i;
        longSession.Push(f);
        if (i % 8 == 0) std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    if (!longSession.Enabled()) return 19;
    longSession.Stop();
    std::ifstream longInput(longPath);
    std::string line;
    bool passedOldLimit = false;
    while (std::getline(longInput, line))
        if (line.starts_with("7203,")) passedOldLimit = true;
    if (!passedOldLimit) return 20;
    std::cout << "past_old_limit=7204 cap=" << posetrace::kMaxFrames << '\n';
    // A session that received no frame leaves no header-only file behind (the player's report lists only real files).
    auto emptyDumper = std::make_unique<posetrace::Dumper>();
    if (!emptyDumper->Start(std::filesystem::path(argv[1]).wstring())) return 21;
    const auto emptyPath = emptyDumper->Path();
    if (!std::filesystem::exists(emptyPath)) return 22;
    emptyDumper->Stop();
    if (std::filesystem::exists(emptyPath)) return 23;
    if (!std::filesystem::exists(longPath) || !std::filesystem::exists(csvPath)) return 24;
    return 0;
}
