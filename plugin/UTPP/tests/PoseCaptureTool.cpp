// Offline tool for the compact pose capture (not a test by itself):
//   utpp_pose_capture_tool bench  <recording.csv> [frames] [blockFrames] [skipFrames]
//       encodes the first `frames` frames (0 = all) of a continuous helper-bone CSV capture with every Windows compression algorithm
//       (XPRESS-Huffman, LZMS) and prints the size per minute of recording and the time cost of quantizing + compressing, per block and per
//       second of recording (the writer thread's budget: a block is built every blockFrames frames).
//   utpp_pose_capture_tool encode <recording.csv> <out.utpppose> [xpress|lzms] [frames] [blockFrames]
//       writes the frames as one compact file (header with the CSV's clock / utc / topology, blocks, trailer): the C++ side of the
//       cross-check with tools/analysis/pose_capture_to_csv.py (ctest utpp_pose_capture_converter_test).
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <string>
#include <vector>

#include "../src/PoseCaptureCompression.hpp"
#include "PoseCaptureFixture.hpp"

using namespace utpp::mm::posecapture;

namespace
{
struct Result
{
    uint64_t rawBytes = 0, fileBytes = 0;
    double encodeMs = 0.0, compressMs = 0.0, worstBlockMs = 0.0;
    uint32_t blocks = 0;
};

Result EncodeAll(const posefixture::CsvCapture& aCap, ICompressor& aComp, uint32_t aBlockFrames, std::vector<uint8_t>* aFile)
{
    using Clock = std::chrono::steady_clock;
    const auto ms = [](Clock::time_point a, Clock::time_point b) { return std::chrono::duration<double, std::milli>(b - a).count(); };
    Result r;
    FileHeader h;
    h.algorithm = aComp.Algorithm();
    h.maxFrames = aCap.maxFrames;
    h.qpcHz = aCap.qpcHz;
    h.armTicks = aCap.armTicks;
    for (int i = 0; i < 7; ++i)
        h.utc[i] = aCap.utc[i];
    h.rig = aCap.map.rig;
    h.rigCount = aCap.map.rigCount;
    h.hashes = aCap.hashes;
    h.parents = aCap.parents;
    h.gameIndex.assign(aCap.map.index.begin(), aCap.map.index.end());
    const uint32_t C = h.Columns(kStreamLegacy);
    std::vector<uint8_t> file, raw, scratch;
    AppendFileHeader(file, h, aCap.map.index);
    const uint32_t total = static_cast<uint32_t>(aCap.frames.size());
    std::vector<float> values;
    std::vector<int64_t> meta;
    for (uint32_t at = 0; at < total; at += aBlockFrames)
    {
        const uint32_t n = std::min(aBlockFrames, total - at);
        values.resize(size_t(n) * C);
        meta.resize(size_t(n) * 4);
        for (uint32_t f = 0; f < n; ++f)
        {
            const auto& fr = aCap.frames[at + f];
            size_t k = size_t(f) * C;
            for (uint32_t b = 0; b < kMaxBones; ++b)
                if (aCap.map.index[b] >= 0)
                    for (uint32_t s = 0; s < 2; ++s)
                        for (uint32_t v = 0; v < 12; ++v)
                            values[k++] = fr.stage[s][b][v];
            meta[size_t(f) * 4 + 0] = fr.ticks;
            meta[size_t(f) * 4 + 1] = static_cast<int64_t>(fr.desc);
            meta[size_t(f) * 4 + 2] = fr.thread;
            meta[size_t(f) * 4 + 3] = static_cast<int64_t>(fr.generation);
        }
        const auto t0 = Clock::now();
        EncodeBlockPayload(kStreamLegacy, n, C, meta.data(), values.data(), raw);
        const auto t1 = Clock::now();
        AppendDataBlock(file, kStreamLegacy, n, raw, aComp, scratch);
        const auto t2 = Clock::now();
        r.encodeMs += ms(t0, t1);
        r.compressMs += ms(t1, t2);
        r.worstBlockMs = std::max(r.worstBlockMs, ms(t0, t2));
        r.rawBytes += raw.size();
        ++r.blocks;
    }
    Trailer t;
    t.frames[0] = total;
    AppendTrailer(file, t);
    r.fileBytes = file.size();
    if (aFile)
        *aFile = std::move(file);
    return r;
}
} // namespace

int main(int argc, char** argv)
{
    if (argc < 3)
    {
        std::fprintf(stderr, "usage: utpp_pose_capture_tool bench <csv> [frames] [blockFrames] | encode <csv> <out> [xpress|lzms] [frames] [blockFrames]\n");
        return 2;
    }
    const std::string cmd = argv[1];
    const bool bench = cmd == "bench";
    const size_t frames = static_cast<size_t>(std::atoll(argc > (bench ? 3 : 5) ? argv[bench ? 3 : 5] : "0"));
    const uint32_t blockFrames = static_cast<uint32_t>(std::atoi(argc > (bench ? 4 : 6) ? argv[bench ? 4 : 6] : "300"));
    const size_t skip = static_cast<size_t>(std::atoll(argc > (bench ? 5 : 7) ? argv[bench ? 5 : 7] : "0"));
    const auto cap = posefixture::ReadCsv(argv[2], frames, skip);
    if (!cap.ok || blockFrames == 0)
    {
        std::fprintf(stderr, "cannot read %s\n", argv[2]);
        return 1;
    }
    const double seconds = static_cast<double>(cap.frames.back().ticks - cap.frames.front().ticks) / static_cast<double>(cap.qpcHz) +
                           static_cast<double>(cap.frames.back().ticks - cap.frames.front().ticks) / static_cast<double>(cap.frames.size() - 1) /
                               static_cast<double>(cap.qpcHz);
    if (bench)
    {
        std::printf("recording: %zu frames, %.1f s (%.1f frames/s), block = %u frames\n", cap.frames.size(), seconds,
                    static_cast<double>(cap.frames.size()) / seconds, blockFrames);
        for (uint32_t alg : {kAlgorithmXpress, kAlgorithmLzms})
        {
            utpp::posecapture::WindowsCompressor comp(alg);
            if (!comp.Valid())
            {
                std::printf("algorithm %u: not available\n", alg);
                continue;
            }
            std::vector<uint8_t> file;
            const Result r = EncodeAll(cap, comp, blockFrames, &file);
            // the file must decode again
            const DecodedFile d = DecodeFile(file.data(), file.size(), comp);
            std::printf("%-14s file %.2f MB = %.2f MB/min (%.0f bytes/frame; raw payload %.2f MB, ratio %.2f) | encode %.0f ms + compress %.0f ms = %.1f ms per second of recording | worst block %.0f ms | %u blocks | decode check %s\n",
                        alg == kAlgorithmXpress ? "XPRESS-Huffman" : "LZMS", static_cast<double>(r.fileBytes) / 1e6, static_cast<double>(r.fileBytes) / 1e6 / (seconds / 60.0),
                        static_cast<double>(r.fileBytes) / static_cast<double>(cap.frames.size()), static_cast<double>(r.rawBytes) / 1e6,
                        static_cast<double>(r.rawBytes) / static_cast<double>(r.fileBytes), r.encodeMs, r.compressMs, (r.encodeMs + r.compressMs) / seconds,
                        r.worstBlockMs, r.blocks, d.headerOk && !d.corrupt && d.FramesOf(0) == cap.frames.size() ? "ok" : "FAILED");
        }
        return 0;
    }
    if (cmd == "encode" && argc >= 4)
    {
        const std::string alg = argc > 4 ? argv[4] : "xpress";
        utpp::posecapture::WindowsCompressor comp(alg == "lzms" ? kAlgorithmLzms : kAlgorithmXpress);
        if (!comp.Valid())
            return 1;
        std::vector<uint8_t> file;
        EncodeAll(cap, comp, blockFrames, &file);
        std::error_code ec;
        std::filesystem::create_directories(std::filesystem::path(argv[3]).parent_path(), ec);
        FILE* f = nullptr;
        if (fopen_s(&f, argv[3], "wb") != 0 || !f)
            return 1;
        std::fwrite(file.data(), 1, file.size(), f);
        std::fclose(f);
        std::printf("wrote %zu bytes (%zu frames, %s)\n", file.size(), cap.frames.size(), alg.c_str());
        return 0;
    }
    return 2;
}
