// Compact pose capture format (src/mm/PoseCaptureFormat.hpp): block encoder / decoder, header, framing, truncation.
//   A) zigzag identity for every 16-bit value;
//   B) synthetic frames: quantization error of every value within the block range / 65535, constant columns exact, nan / inf preserved as nan,
//      exact timestamps and meta fields (including negative and wrapping differences), one-frame and many-block files;
//   C) a slice of a real recording (Build/test-data/posecapture/utpp-bodytrace-slice.csv, cut by tools/analysis/make_pose_capture_fixture.py):
//      encode -> decode, error bound against the CSV values, ticks exact, frame count, topology and bone names in the header;
//   D) a compressor is only an interface: a run-length test compressor round-trips through the compressed-block path (a block that would grow stays stored);
//   E) truncated files: frames before the cut block survive, a cut header is refused, a trailer is optional, a wrong version is refused.
// Mutation UTPP_PCF_MUTATION 1 (8-bit quantization) must print a FAIL line.
// Usage: utpp_pose_capture_format_test <slice.csv>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "../src/mm/PoseCaptureFormat.hpp"
#include "PoseCaptureFixture.hpp"

using namespace utpp::mm::posecapture;

namespace
{
int g_pass = 0, g_fail = 0;
void Check(bool aOk, const std::string& aWhat)
{
    (aOk ? g_pass : g_fail)++;
    std::printf("%s %s\n", aOk ? "PASS" : "FAIL", aWhat.c_str());
}
template <class... A> std::string F(const char* aFmt, A... a)
{
    char b[512];
    std::snprintf(b, sizeof(b), aFmt, a...);
    return b;
}

// Run-length test compressor: (count, byte) pairs.
class RleCompressor final : public ICompressor
{
public:
    int decompressCalls = 0;
    uint32_t Algorithm() const override { return 99; }
    bool Compress(const uint8_t* aSrc, size_t aSize, std::vector<uint8_t>& aOut) override
    {
        aOut.clear();
        for (size_t i = 0; i < aSize;)
        {
            size_t j = i;
            while (j < aSize && aSrc[j] == aSrc[i] && j - i < 255)
                ++j;
            aOut.push_back(static_cast<uint8_t>(j - i));
            aOut.push_back(aSrc[i]);
            i = j;
        }
        return true;
    }
    bool Decompress(const uint8_t* aSrc, size_t aStored, size_t aRawSize, std::vector<uint8_t>& aOut) override
    {
        ++decompressCalls;
        aOut.clear();
        for (size_t i = 0; i + 1 < aStored; i += 2)
            aOut.insert(aOut.end(), aSrc[i], aSrc[i + 1]);
        return aOut.size() == aRawSize;
    }
};

float Ulp(float v) { return std::nextafter(std::fabs(v), std::numeric_limits<float>::infinity()) - std::fabs(v); }

// Largest ratio error / (range / 65535) over all columns of a block, nan mismatches counted separately.
struct Compare
{
    double worst = 0.0;
    size_t nanMismatch = 0, exactBreak = 0, values = 0;
};
Compare CompareBlock(const std::vector<float>& aRef, const DecodedBlock& aGot, uint32_t aFrames, uint32_t aColumns, size_t aFrameOffset)
{
    Compare r;
    for (uint32_t c = 0; c < aColumns; ++c)
    {
        double mn = 1e300, mx = -1e300;
        for (uint32_t f = 0; f < aFrames; ++f)
        {
            const float v = aRef[(aFrameOffset + f) * aColumns + c];
            if (std::isfinite(v))
            {
                mn = std::min<double>(mn, v);
                mx = std::max<double>(mx, v);
            }
        }
        const double range = mx >= mn ? mx - mn : 0.0;
        for (uint32_t f = 0; f < aFrames; ++f)
        {
            const float v = aRef[(aFrameOffset + f) * aColumns + c];
            const float d = aGot.values[size_t(f) * aColumns + c];
            ++r.values;
            if (!std::isfinite(v))
            {
                r.nanMismatch += !std::isnan(d);
                continue;
            }
            if (std::isnan(d))
            {
                ++r.nanMismatch;
                continue;
            }
            if (range == 0.0)
            {
                r.exactBreak += d != v;
                continue;
            }
            const double err = std::fabs(static_cast<double>(d) - static_cast<double>(v)) - Ulp(v); // the float32 rounding of the result is allowed on top
            r.worst = std::max(r.worst, std::max(0.0, err) / (range / 65535.0));
        }
    }
    return r;
}

void WriteHeader(std::vector<uint8_t>& aOut, const FileHeader& h, const std::array<int32_t, kMaxBones>& aIndex) { AppendFileHeader(aOut, h, aIndex); }

FileHeader MakeHeader(const std::array<int32_t, kMaxBones>& aIndex, bool aPost, uint32_t aAlgorithm)
{
    FileHeader h;
    h.algorithm = aAlgorithm;
    h.segmentIndex = 3;
    h.maxFrames = 200;
    h.qpcHz = 10000000;
    h.armTicks = 6017298347926ll;
    const uint16_t utc[7] = {2026, 11, 10, 12, 0, 50, 694};
    for (int i = 0; i < 7; ++i)
        h.utc[i] = utc[i];
    h.rig = 0x9876543210ull;
    h.rigCount = 645;
    h.hasStream[1] = aPost;
    h.gameIndex.assign(aIndex.begin(), aIndex.end());
    for (uint32_t i = 0; i < 645; ++i)
    {
        h.hashes.push_back(0xCA7CFE2BEF51B2A5ull + i * 977);
        h.parents.push_back(static_cast<int16_t>(i == 0 ? -1 : (i - 1) / 2));
    }
    return h;
}

// Builds a file: header + the given frames split into blocks of aBlock frames (+ optional trailer).
struct Built
{
    std::vector<uint8_t> bytes;
    std::vector<size_t> blockEnds; // file offset after every block
};
Built BuildFile(const FileHeader& h, const std::array<int32_t, kMaxBones>& aIndex, uint32_t aStream, uint32_t aColumns, const std::vector<int64_t>& aMeta,
                const std::vector<float>& aValues, uint32_t aFrames, uint32_t aBlock, ICompressor& aComp, bool aTrailer)
{
    Built b;
    WriteHeader(b.bytes, h, aIndex);
    std::vector<uint8_t> raw, scratch;
    const uint32_t metaCount = MetaCount(aStream);
    for (uint32_t at = 0; at < aFrames; at += aBlock)
    {
        const uint32_t n = std::min(aBlock, aFrames - at);
        EncodeBlockPayload(aStream, n, aColumns, aMeta.data() + size_t(at) * metaCount, aValues.data() + size_t(at) * aColumns, raw);
        AppendDataBlock(b.bytes, aStream, n, raw, aComp, scratch);
        b.blockEnds.push_back(b.bytes.size());
    }
    if (aTrailer)
    {
        Trailer t;
        t.frames[aStream] = aFrames;
        t.dropped[aStream] = 7;
        AppendTrailer(b.bytes, t);
    }
    return b;
}

void ZigZag()
{
    bool ok = true;
    for (uint32_t v = 0; v < 65536; ++v)
        ok = ok && UnZigZag16(ZigZag16(static_cast<int16_t>(v))) == static_cast<int16_t>(v);
    const int64_t edge[] = {0, 1, -1, 123456789012345ll, -123456789012345ll, INT64_MAX, INT64_MIN};
    for (int64_t v : edge)
        ok = ok && UnZigZag64(ZigZag64(v)) == v;
    Check(ok, "A: zigzag 16-bit (all 65536 values) and 64-bit round-trip");
}

void Synthetic()
{
    std::mt19937_64 rng(7);
    std::array<int32_t, kMaxBones> index;
    index.fill(-1);
    uint32_t mapped = 0;
    for (uint32_t n = 0; n < kMaxBones; n += 3)
        index[n] = static_cast<int32_t>(mapped++);
    const FileHeader h = MakeHeader(index, true, kAlgorithmStored);
    const uint32_t C = h.Columns(kStreamLegacy), N = 130;
    std::vector<float> values(size_t(N) * C);
    std::vector<int64_t> meta(size_t(N) * 4);
    int64_t ticks = 6017298379950ll;
    for (uint32_t c = 0; c < C; ++c)
    {
        float v = static_cast<float>(static_cast<int>(rng() % 2000) - 1000) * 0.01f;
        const float speed = (c % 7 == 0) ? 0.0f : (c % 5 == 0 ? 1e-7f : 0.05f);
        for (uint32_t f = 0; f < N; ++f)
        {
            v += speed * (static_cast<float>(rng() % 2001) / 1000.0f - 1.0f);
            float w = v;
            if (c % 11 == 0)
                w = 1.0f;                                              // constant column
            if (c % 13 == 0 && f % 17 == 5)
                w = std::numeric_limits<float>::quiet_NaN();           // nan in the middle
            if (c % 17 == 0 && f % 29 == 3)
                w = std::numeric_limits<float>::infinity();            // inf: not representable, comes back as nan
            if (c == 3)
                w = std::numeric_limits<float>::quiet_NaN();           // all-nan column
            values[size_t(f) * C + c] = w;
        }
    }
    for (uint32_t f = 0; f < N; ++f)
    {
        ticks += 164000 + static_cast<int64_t>(rng() % 5000) - (f == 40 ? 400000 : 0); // one step backwards
        meta[size_t(f) * 4 + 0] = ticks;
        meta[size_t(f) * 4 + 1] = static_cast<int64_t>(0xFFFFFFF000000000ull + f % 3); // desc above 2^63 (unsigned in the CSV)
        meta[size_t(f) * 4 + 2] = static_cast<int64_t>(rng() & 0xFFFFFu);
        meta[size_t(f) * 4 + 3] = f < 64 ? 16 : 17;
    }
    StoreCompressor store;
    const Built b = BuildFile(h, index, kStreamLegacy, C, meta, values, N, 50, store, true);
    const DecodedFile d = DecodeFile(b.bytes.data(), b.bytes.size(), store);
    Check(d.headerOk && !d.truncated && !d.corrupt, "B: synthetic file parses (header, blocks, trailer)");
    Check(d.blocks.size() == 3 && d.FramesOf(0) == N, F("B: 3 blocks (50+50+30), %llu frames", static_cast<unsigned long long>(d.FramesOf(0))));
    Check(d.trailer.present && d.trailer.frames[0] == N && d.trailer.dropped[0] == 7, "B: trailer frames / dropped");
    bool metaExact = true;
    size_t at = 0;
    for (const auto& blk : d.blocks)
    {
        for (uint32_t f = 0; f < blk.frames; ++f, ++at)
            for (uint32_t m = 0; m < 4; ++m)
                metaExact = metaExact && blk.meta[size_t(f) * 4 + m] == meta[at * 4 + m];
    }
    Check(metaExact, "B: ticks / desc / thread / generation exact (negative step, values above 2^63)");
    double worst = 0.0;
    size_t nanMis = 0, exactBreak = 0, off = 0;
    for (const auto& blk : d.blocks)
    {
        const Compare r = CompareBlock(values, blk, blk.frames, C, off);
        worst = std::max(worst, r.worst);
        nanMis += r.nanMismatch;
        exactBreak += r.exactBreak;
        off += blk.frames;
    }
    Check(worst <= 1.0, F("B: quantization error %.3f of range / 65535 (limit 1.0)", worst));
    Check(nanMis == 0, F("B: nan / inf come back as nan (%zu mismatches)", nanMis));
    Check(exactBreak == 0, F("B: constant columns are exact (%zu broken)", exactBreak));
    // header round trip
    Check(d.header.version == 1 && d.header.algorithm == 0 && d.header.segmentIndex == 3 && d.header.maxFrames == 200 && d.header.qpcHz == 10000000 &&
              d.header.armTicks == 6017298347926ll && d.header.utc[0] == 2026 && d.header.utc[6] == 694 && d.header.rig == 0x9876543210ull &&
              d.header.rigCount == 645 && d.header.hasStream[0] && d.header.hasStream[1],
          "B: header fields round-trip (clock, utc, rig, streams)");
    Check(d.header.names.size() == kMaxBones && d.header.names[0] == "Spine3" && d.header.names[184] == "reference_joint" && d.header.gameIndex[3] == 1 &&
              d.header.gameIndex[1] == -1 && d.header.hashes.size() == 645 && d.header.hashes[2] == 0xCA7CFE2BEF51B2A5ull + 2 * 977 && d.header.parents[2] == 0 &&
              d.header.parents[0] == -1,
          "B: header carries the bone names, the index map and the rig topology");
    // one frame in a block: the key frame alone
    {
        const Built one = BuildFile(h, index, kStreamLegacy, C, meta, values, 1, 50, store, false);
        const DecodedFile o = DecodeFile(one.bytes.data(), one.bytes.size(), store);
        Check(o.headerOk && o.blocks.size() == 1 && o.blocks[0].frames == 1 && !o.trailer.present, "B: a one-frame block decodes (key frame only), file without trailer");
    }
    // post-helper stream: 8 meta fields, 12 values per bone
    {
        const uint32_t PC = h.Columns(kStreamPostHelper);
        std::vector<float> pv(size_t(5) * PC);
        std::vector<int64_t> pm(size_t(5) * 8);
        for (size_t i = 0; i < pv.size(); ++i)
            pv[i] = static_cast<float>(i % 97) * 0.125f;
        for (size_t i = 0; i < pm.size(); ++i)
            pm[i] = static_cast<int64_t>(1000 + i * 3);
        const Built pb = BuildFile(h, index, kStreamPostHelper, PC, pm, pv, 5, 5, store, false);
        const DecodedFile pd = DecodeFile(pb.bytes.data(), pb.bytes.size(), store);
        bool ok = pd.blocks.size() == 1 && pd.blocks[0].stream == 1 && pd.blocks[0].meta.size() == pm.size();
        for (size_t i = 0; ok && i < pm.size(); ++i)
            ok = pd.blocks[0].meta[i] == pm[i];
        Check(ok && PC == mapped * 12, "B: post-helper block (8 meta fields, 12 values per bone)");
    }
}

void Slice(const std::string& aPath)
{
    const auto cap = posefixture::ReadCsv(aPath, 0);
    if (!cap.ok)
    {
        Check(false, "C: cannot read the slice " + aPath);
        return;
    }
    const uint32_t N = static_cast<uint32_t>(cap.frames.size());
    uint32_t mapped = 0;
    for (uint32_t n = 0; n < kMaxBones; ++n)
        mapped += cap.map.index[n] >= 0;
    FileHeader h;
    h.qpcHz = cap.qpcHz;
    h.armTicks = cap.armTicks;
    h.rig = cap.map.rig;
    h.rigCount = cap.map.rigCount;
    h.hashes = cap.hashes;
    h.parents = cap.parents;
    h.gameIndex.assign(cap.map.index.begin(), cap.map.index.end());
    const uint32_t C = h.Columns(kStreamLegacy);
    std::vector<float> values(size_t(N) * C);
    std::vector<int64_t> meta(size_t(N) * 4);
    for (uint32_t f = 0; f < N; ++f)
    {
        const auto& fr = cap.frames[f];
        size_t at = size_t(f) * C;
        for (uint32_t n = 0; n < kMaxBones; ++n)
            if (cap.map.index[n] >= 0)
                for (uint32_t s = 0; s < 2; ++s)
                    for (uint32_t v = 0; v < 12; ++v)
                        values[at++] = fr.stage[s][n][v];
        meta[size_t(f) * 4 + 0] = fr.ticks;
        meta[size_t(f) * 4 + 1] = static_cast<int64_t>(fr.desc);
        meta[size_t(f) * 4 + 2] = fr.thread;
        meta[size_t(f) * 4 + 3] = static_cast<int64_t>(fr.generation);
    }
    RleCompressor rle;
    StoreCompressor store;
    for (ICompressor* comp : {static_cast<ICompressor*>(&store), static_cast<ICompressor*>(&rle)})
    {
        h.algorithm = comp->Algorithm();
        const Built b = BuildFile(h, cap.map.index, kStreamLegacy, C, meta, values, N, 50, *comp, true);
        const DecodedFile d = DecodeFile(b.bytes.data(), b.bytes.size(), *comp);
        const std::string tag = comp == &store ? "stored" : "RLE";
        Check(d.headerOk && !d.corrupt && !d.truncated && d.FramesOf(0) == N, F("C: slice (%s): %u frames, %u bones, %u columns, %zu bytes -> decoded %llu frames", tag.c_str(), N, mapped, C,
                                                                                   b.bytes.size(), static_cast<unsigned long long>(d.FramesOf(0))));
        if (!d.headerOk)
            continue;
        bool ticksExact = true;
        size_t at = 0;
        double worst = 0.0;
        size_t nanMis = 0, off = 0;
        for (const auto& blk : d.blocks)
        {
            for (uint32_t f = 0; f < blk.frames; ++f, ++at)
                for (uint32_t m = 0; m < 4; ++m)
                    ticksExact = ticksExact && blk.meta[size_t(f) * 4 + m] == meta[at * 4 + m];
            const Compare r = CompareBlock(values, blk, blk.frames, C, off);
            worst = std::max(worst, r.worst);
            nanMis += r.nanMismatch;
            off += blk.frames;
        }
        Check(ticksExact, "C: slice (" + tag + "): frame timestamps and ids exact");
        Check(worst <= 1.0 && nanMis == 0, F("C: slice (%s): error %.3f of range / 65535, %zu nan mismatches", tag.c_str(), worst, nanMis));
        Check(d.header.hashes == cap.hashes && d.header.parents == cap.parents && d.header.rigCount == cap.map.rigCount, "C: slice: topology in the header equals the CSV `# rig_bone` lines");
    }
}

// A compressor is only an interface: a block that shrinks goes through Compress / Decompress (flag 1), one that does not stays stored (flag 0).
void CompressedPath()
{
    std::array<int32_t, kMaxBones> index;
    index.fill(-1);
    for (uint32_t n = 0; n < 6; ++n)
        index[n] = static_cast<int32_t>(n);
    const FileHeader h = MakeHeader(index, false, 99);
    const uint32_t C = h.Columns(kStreamLegacy), N = 40;
    std::vector<float> still(size_t(N) * C, 0.25f); // nothing moves: the planes are runs of zeros
    std::vector<int64_t> meta(size_t(N) * 4);
    for (uint32_t f = 0; f < N; ++f)
        meta[size_t(f) * 4] = 5000 + f * 100;
    RleCompressor rle;
    const Built b = BuildFile(h, index, kStreamLegacy, C, meta, still, N, 40, rle, false);
    const DecodedFile d = DecodeFile(b.bytes.data(), b.bytes.size(), rle);
    bool same = d.headerOk && d.blocks.size() == 1;
    for (size_t i = 0; same && i < still.size(); ++i)
        same = d.blocks[0].values[i] == still[i];
    Check(same && rle.decompressCalls == 1, F("D: a shrinking block goes through the compressor interface and decodes exactly (%d decompress call)", rle.decompressCalls));
    std::vector<float> noisy(size_t(N) * C);
    std::mt19937 rng(3);
    for (float& v : noisy)
        v = static_cast<float>(rng() % 100000) * 0.001f;
    const Built nb = BuildFile(h, index, kStreamLegacy, C, meta, noisy, N, 40, rle, false);
    rle.decompressCalls = 0;
    const DecodedFile nd = DecodeFile(nb.bytes.data(), nb.bytes.size(), rle);
    Check(nd.headerOk && nd.blocks.size() == 1 && rle.decompressCalls == 0, "D: a block that would grow is stored as is (flag 0, the compressor is not asked to decode it)");
}

void Truncation()
{
    std::array<int32_t, kMaxBones> index;
    index.fill(-1);
    for (uint32_t n = 0; n < 10; ++n)
        index[n * 5] = static_cast<int32_t>(n);
    FileHeader h = MakeHeader(index, false, kAlgorithmStored);
    const uint32_t C = h.Columns(kStreamLegacy), N = 60;
    std::vector<float> values(size_t(N) * C, 0.5f);
    for (size_t i = 0; i < values.size(); ++i)
        values[i] += static_cast<float>(i % 13) * 0.01f;
    std::vector<int64_t> meta(size_t(N) * 4);
    for (uint32_t f = 0; f < N; ++f)
        meta[size_t(f) * 4] = 1000 + f * 164000;
    StoreCompressor store;
    const Built b = BuildFile(h, index, kStreamLegacy, C, meta, values, N, 20, store, true);
    // cut inside the last block: two complete blocks survive
    std::vector<uint8_t> cut(b.bytes.begin(), b.bytes.begin() + static_cast<std::ptrdiff_t>(b.blockEnds[2] - 10));
    const DecodedFile d = DecodeFile(cut.data(), cut.size(), store);
    Check(d.headerOk && d.truncated && d.blocks.size() == 2 && d.FramesOf(0) == 40 && !d.trailer.present, "E: cut inside a block: the earlier blocks decode, truncated flagged, no trailer");
    // cut exactly after a block (no trailer: a crash between blocks)
    const DecodedFile e = DecodeFile(b.bytes.data(), b.blockEnds[1], store);
    Check(e.headerOk && !e.truncated && e.blocks.size() == 2 && !e.trailer.present, "E: file ending after a block without trailer reads cleanly");
    // cut inside the header
    const DecodedFile hh = DecodeFile(b.bytes.data(), 30, store);
    Check(!hh.headerOk, "E: a cut header is refused");
    // wrong version
    std::vector<uint8_t> bad = b.bytes;
    bad[8] = 2;
    Check(!DecodeFile(bad.data(), bad.size(), store).headerOk, "E: an unknown format version is refused");
    bad = b.bytes;
    bad[0] = 'X';
    Check(!DecodeFile(bad.data(), bad.size(), store).headerOk, "E: a wrong magic is refused");
}
} // namespace

int main(int argc, char** argv)
{
    ZigZag();
    Synthetic();
    if (argc > 1)
        Slice(argv[1]);
    else
        Check(false, "C: slice path argument missing");
    CompressedPath();
    Truncation();
    std::printf("%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
