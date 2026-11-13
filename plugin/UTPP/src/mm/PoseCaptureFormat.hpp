#pragma once
// Compact capture of the helper-bone pose (all kMaxBones = 185 bones, both stages, every frame): binary file format, block encoder and
// decoder. Pure C++ (no game types, no Win32): the compression is an injected interface (ICompressor), the production implementation
// (Windows Compression API) lives in src/PoseCaptureRuntime.cpp; the session layer (segments, size cap, retention) is in
// mm/PoseCaptureSession.hpp. tools/analysis/pose_capture_to_csv.py reads this format and writes the text CSV of the continuous capture.
//
// FILE LAYOUT (all integers little-endian; one file = one rig topology; the extension is .utpppose)
//   header
//     char[8]  magic "UTPPPOSE"
//     u32      formatVersion (1)
//     u32      headerBytes   total size of this header including the magic (a reader may skip to the first block with it)
//     u32      flags (0)
//     u32      algorithm     block compression: 0 = stored, 4 = Windows XPRESS with Huffman, 5 = Windows LZMS (COMPRESS_ALGORITHM_* values,
//                            buffer mode, no COMPRESS_RAW)
//     u32      segmentIndex  0-based number of this file within one capture session (files of one session sort by it)
//     u32      maxFrames     the capture budget printed as max_frames in the CSV header
//     i64      qpcHz         QueryPerformanceFrequency (ticks per second of every frame timestamp)
//     i64      armTicks      QueryPerformanceCounter when the file was opened
//     u16[7]   utc           start time: year, month, day, hour, minute, second, millisecond; then u16 0
//     u64      rig           rig identity of every frame (the `rig` column)
//     u32      rigCount      bones of the rig (the `rigCount` column)
//     u8       streamCount   the phases that follow
//       per stream: u8 id (0 = before_mm / after_mm_and_aim, 1 = post_helper_apply_after_commit), u8 stageCount, u8 metaCount,
//                   per stage: u8 length + name
//     u32      boneCount (kMaxBones); per bone: u8 length + name, i32 gameIndex (-1 = not present in the rig: no columns, values are nan)
//     u32      rigBones;  per rig bone: u64 hash, i16 parent   (the `# rig_bone` lines of the CSV)
//   blocks until the end of the file
//     u8 kind
//     kind 1, data block:  u8 stream, u32 frameCount, u32 rawSize, u32 storedSize, u8 compressed (0/1), storedSize bytes
//     kind 2, trailer:     u32 flags (bit 0 = a write failed), u64 frames[2], u64 dropped[2]    (closes the file; absent after a crash)
//   A reader stops cleanly at a truncated block (the frames before it stay valid).
//
// BLOCK PAYLOAD (uncompressed, `rawSize` bytes). A block holds N consecutive frames of one stream and is self-contained: the quantization
// ranges and the first frame (the key frame, stored absolutely) are inside, no state is carried between blocks.
//   C = mappedBones * stageCount * 12 columns; mapped bones in ascending catalogue order (gameIndex >= 0), then stage, then value 0..11
//   u32       metaBytes
//   metaBytes per-frame fields, field-major: for field in 0..metaCount-1: for frame in 0..N-1: varint(zigzag(field[f] - field[f-1])), with
//             field[-1] = 0. Stream 0 fields: ticks, desc, thread, generation. Stream 1: ticks, desc, thread, generation, receiptGeneration,
//             receiptCycle, receiptStartedTicks, receiptLastTicks. Timestamps are exact (QPC ticks).
//   f32[C]    column minimum        (minimum / maximum over the finite values of the column inside this block; 0 / 0 when there are none)
//   f32[C]    column maximum
//   u8[C*N]   low bytes of the codes, column-major (column c occupies [c*N, c*N+N), frame order)
//   u8[C*N]   high bytes, same order
//   Code of a value: finite v -> round((v - min) / step) in 0..65534 with step = (max - min) / 65534 (step 0: code 0); non-finite -> 65535.
//   Stored per column: frame 0 = the code itself; frame f >= 1 = zigzag(int16(code[f] - code[f-1])) (the difference taken modulo 2^16).
//   Decoded value = float32(min + code * step); code 65535 = nan. Error of a finite value <= step / 2 = (max - min) / 131068 plus the float32
//   rounding of the result, i.e. below 1/65535 of the block range of the column. Columns that never change are exact.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "BodyDeformerTrace.hpp"

namespace utpp::mm::posecapture
{
using bodytrace::kMaxBones;
using bodytrace::kNames;

inline constexpr uint32_t kFormatVersion = 1;
inline constexpr char kMagic[8] = {'U', 'T', 'P', 'P', 'P', 'O', 'S', 'E'};
inline constexpr uint8_t kStreamLegacy = 0, kStreamPostHelper = 1;
inline constexpr uint32_t kStreamCount = 2;
inline constexpr uint32_t kMaxMetaFields = 8;
inline constexpr uint16_t kNanCode = 65535;
inline constexpr uint16_t kMaxCode = 65534;
inline constexpr uint32_t kAlgorithmStored = 0, kAlgorithmXpress = 4, kAlgorithmLzms = 5;

inline constexpr uint32_t StageCount(uint32_t aStream) { return aStream == kStreamLegacy ? 2u : 1u; }
inline constexpr uint32_t MetaCount(uint32_t aStream) { return aStream == kStreamLegacy ? 4u : 8u; }
inline const char* StageName(uint32_t aStream, uint32_t aStage)
{
    if (aStream == kStreamPostHelper)
        return "post_helper_apply_after_commit";
    return aStage == 0 ? "before_mm" : "after_mm_and_aim";
}

// ---- injected block compression ----------------------------------------------------------------------------------------------------
class ICompressor
{
public:
    virtual ~ICompressor() = default;
    virtual uint32_t Algorithm() const = 0; // value written to the file header
    virtual bool Compress(const uint8_t* aSrc, size_t aSize, std::vector<uint8_t>& aOut) = 0;
    virtual bool Decompress(const uint8_t* aSrc, size_t aStored, size_t aRawSize, std::vector<uint8_t>& aOut) = 0;
};
class StoreCompressor final : public ICompressor
{
public:
    uint32_t Algorithm() const override { return kAlgorithmStored; }
    bool Compress(const uint8_t* aSrc, size_t aSize, std::vector<uint8_t>& aOut) override { aOut.assign(aSrc, aSrc + aSize); return true; }
    bool Decompress(const uint8_t* aSrc, size_t aStored, size_t aRawSize, std::vector<uint8_t>& aOut) override
    {
        if (aStored != aRawSize)
            return false;
        aOut.assign(aSrc, aSrc + aStored);
        return true;
    }
};

// ---- little-endian byte helpers ----------------------------------------------------------------------------------------------------
class ByteWriter
{
public:
    explicit ByteWriter(std::vector<uint8_t>& aOut) : m_out(aOut) {}
    void U8(uint8_t v) { m_out.push_back(v); }
    void U16(uint16_t v) { Raw(&v, 2); }
    void U32(uint32_t v) { Raw(&v, 4); }
    void U64(uint64_t v) { Raw(&v, 8); }
    void I16(int16_t v) { Raw(&v, 2); }
    void I32(int32_t v) { Raw(&v, 4); }
    void I64(int64_t v) { Raw(&v, 8); }
    void F32(float v) { Raw(&v, 4); }
    void Str(const std::string& s) { U8(static_cast<uint8_t>(std::min<size_t>(s.size(), 255))); Raw(s.data(), std::min<size_t>(s.size(), 255)); }
    void Varint(uint64_t v)
    {
        while (v >= 0x80)
        {
            m_out.push_back(static_cast<uint8_t>(v | 0x80));
            v >>= 7;
        }
        m_out.push_back(static_cast<uint8_t>(v));
    }
    void Raw(const void* aData, size_t aSize)
    {
        const auto* p = static_cast<const uint8_t*>(aData);
        m_out.insert(m_out.end(), p, p + aSize);
    }
    size_t Size() const { return m_out.size(); }

private:
    std::vector<uint8_t>& m_out;
};

class ByteReader
{
public:
    ByteReader(const uint8_t* aData, size_t aSize) : m_d(aData), m_n(aSize) {}
    bool Ok() const { return m_ok; }
    size_t Pos() const { return m_p; }
    size_t Left() const { return m_n - m_p; }
    const uint8_t* Cursor() const { return m_d + m_p; }
    bool Skip(size_t aN)
    {
        if (aN > Left()) { m_ok = false; return false; }
        m_p += aN;
        return true;
    }
    uint8_t U8() { uint8_t v = 0; Raw(&v, 1); return v; }
    uint16_t U16() { uint16_t v = 0; Raw(&v, 2); return v; }
    uint32_t U32() { uint32_t v = 0; Raw(&v, 4); return v; }
    uint64_t U64() { uint64_t v = 0; Raw(&v, 8); return v; }
    int16_t I16() { int16_t v = 0; Raw(&v, 2); return v; }
    int32_t I32() { int32_t v = 0; Raw(&v, 4); return v; }
    int64_t I64() { int64_t v = 0; Raw(&v, 8); return v; }
    float F32() { float v = 0; Raw(&v, 4); return v; }
    std::string Str()
    {
        const size_t len = U8();
        std::string s;
        if (len <= Left())
            s.assign(reinterpret_cast<const char*>(m_d + m_p), len);
        else
            m_ok = false;
        Skip(len);
        return s;
    }
    uint64_t Varint()
    {
        uint64_t v = 0;
        for (int shift = 0; shift < 70; shift += 7)
        {
            const uint8_t b = U8();
            if (!m_ok)
                return 0;
            v |= static_cast<uint64_t>(b & 0x7F) << shift;
            if (!(b & 0x80))
                return v;
        }
        m_ok = false;
        return 0;
    }
    bool Raw(void* aOut, size_t aSize)
    {
        if (aSize > Left())
        {
            m_ok = false;
            std::memset(aOut, 0, aSize);
            return false;
        }
        std::memcpy(aOut, m_d + m_p, aSize);
        m_p += aSize;
        return true;
    }

private:
    const uint8_t* m_d;
    size_t m_n, m_p = 0;
    bool m_ok = true;
};

inline uint64_t ZigZag64(int64_t v) { return (static_cast<uint64_t>(v) << 1) ^ static_cast<uint64_t>(v >> 63); }
inline int64_t UnZigZag64(uint64_t v) { return static_cast<int64_t>(v >> 1) ^ -static_cast<int64_t>(v & 1); }
inline uint16_t ZigZag16(int16_t v) { return static_cast<uint16_t>((static_cast<uint16_t>(v) << 1) ^ static_cast<uint16_t>(v >> 15)); }
inline int16_t UnZigZag16(uint16_t v) { return static_cast<int16_t>(static_cast<uint16_t>((v >> 1) ^ static_cast<uint16_t>(-static_cast<int16_t>(v & 1)))); }

// ---- file header --------------------------------------------------------------------------------------------------------------------
struct FileHeader
{
    uint32_t version = kFormatVersion, flags = 0, algorithm = kAlgorithmStored, segmentIndex = 0, maxFrames = 0, rigCount = 0;
    int64_t qpcHz = 0, armTicks = 0;
    uint16_t utc[7] = {0, 0, 0, 0, 0, 0, 0};
    uint64_t rig = 0;
    bool hasStream[kStreamCount] = {true, false};
    std::vector<std::string> names;        // boneCount catalogue names (read side; the write side uses kNames)
    std::vector<int32_t> gameIndex;        // boneCount entries
    std::vector<uint64_t> hashes;          // rig topology
    std::vector<int16_t> parents;

    uint32_t MappedBones() const
    {
        uint32_t n = 0;
        for (int32_t i : gameIndex)
            n += i >= 0;
        return n;
    }
    uint32_t Columns(uint32_t aStream) const { return MappedBones() * StageCount(aStream) * 12u; }
};

inline void AppendFileHeader(std::vector<uint8_t>& aOut, const FileHeader& h, const std::array<int32_t, kMaxBones>& aIndex)
{
    const size_t start = aOut.size();
    ByteWriter w(aOut);
    w.Raw(kMagic, sizeof(kMagic));
    w.U32(kFormatVersion);
    const size_t sizeAt = aOut.size();
    w.U32(0); // headerBytes, patched below
    w.U32(h.flags);
    w.U32(h.algorithm);
    w.U32(h.segmentIndex);
    w.U32(h.maxFrames);
    w.I64(h.qpcHz);
    w.I64(h.armTicks);
    for (uint16_t v : h.utc)
        w.U16(v);
    w.U16(0);
    w.U64(h.rig);
    w.U32(h.rigCount);
    uint8_t streams = 0;
    for (uint32_t s = 0; s < kStreamCount; ++s)
        streams = static_cast<uint8_t>(streams + (h.hasStream[s] ? 1 : 0));
    w.U8(streams);
    for (uint32_t s = 0; s < kStreamCount; ++s)
    {
        if (!h.hasStream[s])
            continue;
        w.U8(static_cast<uint8_t>(s));
        w.U8(static_cast<uint8_t>(StageCount(s)));
        w.U8(static_cast<uint8_t>(MetaCount(s)));
        for (uint32_t st = 0; st < StageCount(s); ++st)
            w.Str(StageName(s, st));
    }
    w.U32(static_cast<uint32_t>(kMaxBones));
    for (uint32_t n = 0; n < kMaxBones; ++n)
    {
        w.Str(std::string(kNames[n]));
        w.I32(aIndex[n]);
    }
    w.U32(static_cast<uint32_t>(h.hashes.size()));
    for (size_t i = 0; i < h.hashes.size(); ++i)
    {
        w.U64(h.hashes[i]);
        w.I16(i < h.parents.size() ? h.parents[i] : static_cast<int16_t>(-1));
    }
    const uint32_t total = static_cast<uint32_t>(aOut.size() - start);
    std::memcpy(aOut.data() + sizeAt, &total, 4);
}

inline bool ParseFileHeader(ByteReader& r, FileHeader& h)
{
    char magic[8] = {};
    r.Raw(magic, 8);
    if (!r.Ok() || std::memcmp(magic, kMagic, 8) != 0)
        return false;
    h = FileHeader{};
    h.version = r.U32();
    const size_t headerStart = r.Pos() - 12;
    const uint32_t headerBytes = r.U32();
    if (!r.Ok() || h.version != kFormatVersion)
        return false;
    h.flags = r.U32();
    h.algorithm = r.U32();
    h.segmentIndex = r.U32();
    h.maxFrames = r.U32();
    h.qpcHz = r.I64();
    h.armTicks = r.I64();
    for (uint16_t& v : h.utc)
        v = r.U16();
    r.U16();
    h.rig = r.U64();
    h.rigCount = r.U32();
    const uint32_t streams = r.U8();
    h.hasStream[0] = h.hasStream[1] = false;
    for (uint32_t i = 0; i < streams && r.Ok(); ++i)
    {
        const uint32_t id = r.U8(), stages = r.U8(), metas = r.U8();
        if (id >= kStreamCount || stages != StageCount(id) || metas != MetaCount(id))
            return false;
        h.hasStream[id] = true;
        for (uint32_t st = 0; st < stages; ++st)
            r.Str();
    }
    const uint32_t bones = r.U32();
    if (!r.Ok() || bones != kMaxBones)
        return false;
    for (uint32_t n = 0; n < bones && r.Ok(); ++n)
    {
        h.names.push_back(r.Str());
        h.gameIndex.push_back(r.I32());
    }
    const uint32_t rigBones = r.U32();
    if (!r.Ok() || rigBones > 65535)
        return false;
    for (uint32_t i = 0; i < rigBones && r.Ok(); ++i)
    {
        h.hashes.push_back(r.U64());
        h.parents.push_back(r.I16());
    }
    return r.Ok() && r.Pos() - headerStart == headerBytes;
}

// ---- block encoder / decoder ----------------------------------------------------------------------------------------------------------
// aMeta: N * MetaCount(stream) values, frame-major. aValues: N * columns floats, frame-major (frame f, column c at f * columns + c).
inline void EncodeBlockPayload(uint32_t aStream, uint32_t aFrames, uint32_t aColumns, const int64_t* aMeta, const float* aValues,
                               std::vector<uint8_t>& aOut)
{
    aOut.clear();
    ByteWriter w(aOut);
    const uint32_t metaCount = MetaCount(aStream), N = aFrames, C = aColumns;
    std::vector<uint8_t> meta;
    {
        ByteWriter mw(meta);
        for (uint32_t m = 0; m < metaCount; ++m)
        {
            int64_t prev = 0;
            for (uint32_t f = 0; f < N; ++f)
            {
                const int64_t v = aMeta[size_t(f) * metaCount + m];
                mw.Varint(ZigZag64(static_cast<int64_t>(static_cast<uint64_t>(v) - static_cast<uint64_t>(prev))));
                prev = v;
            }
        }
    }
    w.U32(static_cast<uint32_t>(meta.size()));
    w.Raw(meta.data(), meta.size());
    // ranges over the finite values of each column
    std::vector<float> mn(C, std::numeric_limits<float>::infinity()), mx(C, -std::numeric_limits<float>::infinity());
    for (uint32_t f = 0; f < N; ++f)
    {
        const float* row = aValues + size_t(f) * C;
        for (uint32_t c = 0; c < C; ++c)
        {
            const float v = row[c];
            if (std::isfinite(v))
            {
                mn[c] = std::min(mn[c], v);
                mx[c] = std::max(mx[c], v);
            }
        }
    }
    for (uint32_t c = 0; c < C; ++c)
        if (mn[c] > mx[c]) // no finite value
            mn[c] = mx[c] = 0.0f;
    for (uint32_t c = 0; c < C; ++c)
        w.F32(mn[c]);
    for (uint32_t c = 0; c < C; ++c)
        w.F32(mx[c]);
    // codes, then the two byte planes in column-major order
    std::vector<double> inv(C), base(C);
    for (uint32_t c = 0; c < C; ++c)
    {
        const double range = static_cast<double>(mx[c]) - static_cast<double>(mn[c]);
        base[c] = mn[c];
        inv[c] = range > 0.0 ? static_cast<double>(kMaxCode) / range : 0.0;
    }
    const size_t planeBytes = size_t(C) * N;
    const size_t lowAt = aOut.size();
    aOut.resize(lowAt + 2 * planeBytes);
    uint8_t* low = aOut.data() + lowAt;
    uint8_t* high = low + planeBytes;
    std::vector<uint16_t> prev(C, 0);
    for (uint32_t f = 0; f < N; ++f)
    {
        const float* row = aValues + size_t(f) * C;
        for (uint32_t c = 0; c < C; ++c)
        {
            const float v = row[c];
            uint16_t code;
            if (!std::isfinite(v))
                code = kNanCode;
            else
            {
                double q = std::floor((static_cast<double>(v) - base[c]) * inv[c] + 0.5);
                q = q < 0.0 ? 0.0 : (q > kMaxCode ? static_cast<double>(kMaxCode) : q);
#if defined(UTPP_PCF_MUTATION) && UTPP_PCF_MUTATION == 1 // mutation 1: 8-bit quantization (an error of range / 255)
                q = std::floor(q / 257.0) * 257.0;
#endif
                code = static_cast<uint16_t>(q);
            }
            const uint16_t stored = f == 0 ? code : ZigZag16(static_cast<int16_t>(static_cast<uint16_t>(code - prev[c])));
            prev[c] = code;
            const size_t at = size_t(c) * N + f;
            low[at] = static_cast<uint8_t>(stored & 0xFF);
            high[at] = static_cast<uint8_t>(stored >> 8);
        }
    }
}

struct DecodedBlock
{
    uint8_t stream = 0;
    uint32_t frames = 0, columns = 0;
    std::vector<int64_t> meta;  // frames * MetaCount(stream), frame-major
    std::vector<float> values;  // frames * columns, frame-major
};

inline bool DecodeBlockPayload(uint32_t aStream, uint32_t aFrames, uint32_t aColumns, const uint8_t* aRaw, size_t aRawSize, DecodedBlock& aOut)
{
    ByteReader r(aRaw, aRawSize);
    const uint32_t metaCount = MetaCount(aStream), N = aFrames, C = aColumns;
    const uint32_t metaBytes = r.U32();
    if (!r.Ok() || metaBytes > r.Left())
        return false;
    ByteReader mr(r.Cursor(), metaBytes);
    r.Skip(metaBytes);
    aOut.stream = static_cast<uint8_t>(aStream);
    aOut.frames = N;
    aOut.columns = C;
    aOut.meta.assign(size_t(N) * metaCount, 0);
    for (uint32_t m = 0; m < metaCount; ++m)
    {
        int64_t prev = 0;
        for (uint32_t f = 0; f < N; ++f)
        {
            const int64_t d = UnZigZag64(mr.Varint());
            prev = static_cast<int64_t>(static_cast<uint64_t>(prev) + static_cast<uint64_t>(d));
            aOut.meta[size_t(f) * metaCount + m] = prev;
        }
    }
    if (!mr.Ok())
        return false;
    std::vector<float> mn(C), mx(C);
    for (float& v : mn)
        v = r.F32();
    for (float& v : mx)
        v = r.F32();
    const size_t planeBytes = size_t(C) * N;
    if (!r.Ok() || r.Left() != 2 * planeBytes)
        return false;
    const uint8_t* low = r.Cursor();
    const uint8_t* high = low + planeBytes;
    aOut.values.assign(planeBytes, 0.0f);
    for (uint32_t c = 0; c < C; ++c)
    {
        const double step = (static_cast<double>(mx[c]) - static_cast<double>(mn[c])) / static_cast<double>(kMaxCode);
        const double base = mn[c];
        uint16_t code = 0;
        for (uint32_t f = 0; f < N; ++f)
        {
            const size_t at = size_t(c) * N + f;
            const uint16_t stored = static_cast<uint16_t>(low[at] | (static_cast<uint16_t>(high[at]) << 8));
            code = f == 0 ? stored : static_cast<uint16_t>(code + static_cast<uint16_t>(UnZigZag16(stored)));
            aOut.values[size_t(f) * C + c] =
                code == kNanCode ? std::numeric_limits<float>::quiet_NaN() : static_cast<float>(base + static_cast<double>(code) * step);
        }
    }
    return true;
}

// ---- block framing ----------------------------------------------------------------------------------------------------------------------
inline bool AppendDataBlock(std::vector<uint8_t>& aOut, uint32_t aStream, uint32_t aFrames, const std::vector<uint8_t>& aRaw, ICompressor& aComp,
                            std::vector<uint8_t>& aScratch)
{
    bool compressed = aComp.Algorithm() != kAlgorithmStored && aComp.Compress(aRaw.data(), aRaw.size(), aScratch) && aScratch.size() < aRaw.size();
    const std::vector<uint8_t>& body = compressed ? aScratch : aRaw;
    if (body.size() > 0xFFFFFFFFull || aRaw.size() > 0xFFFFFFFFull)
        return false;
    ByteWriter w(aOut);
    w.U8(1);
    w.U8(static_cast<uint8_t>(aStream));
    w.U32(aFrames);
    w.U32(static_cast<uint32_t>(aRaw.size()));
    w.U32(static_cast<uint32_t>(body.size()));
    w.U8(compressed ? 1 : 0);
    w.Raw(body.data(), body.size());
    return true;
}

struct Trailer
{
    bool present = false, failed = false;
    uint64_t frames[2] = {0, 0}, dropped[2] = {0, 0};
};

inline void AppendTrailer(std::vector<uint8_t>& aOut, const Trailer& t)
{
    ByteWriter w(aOut);
    w.U8(2);
    w.U32(t.failed ? 1u : 0u);
    w.U64(t.frames[0]); w.U64(t.frames[1]);
    w.U64(t.dropped[0]); w.U64(t.dropped[1]);
}

// Whole-file reader (tests, the measurement tool): header, every complete block decoded, trailer when present.
struct DecodedFile
{
    FileHeader header;
    std::vector<DecodedBlock> blocks;
    Trailer trailer;
    bool headerOk = false, truncated = false, corrupt = false;
    uint64_t FramesOf(uint32_t aStream) const
    {
        uint64_t n = 0;
        for (const auto& b : blocks)
            n += b.stream == aStream ? b.frames : 0;
        return n;
    }
};

inline DecodedFile DecodeFile(const uint8_t* aData, size_t aSize, ICompressor& aComp)
{
    DecodedFile out;
    ByteReader r(aData, aSize);
    if (!ParseFileHeader(r, out.header))
        return out;
    out.headerOk = true;
    std::vector<uint8_t> raw;
    while (r.Left() > 0)
    {
        const uint8_t kind = r.U8();
        if (kind == 1)
        {
            const uint32_t stream = r.U8(), frames = r.U32(), rawSize = r.U32(), stored = r.U32();
            const uint8_t compressed = r.U8();
            if (!r.Ok() || stream >= kStreamCount || stored > r.Left())
            {
                out.truncated = !r.Ok() || stored > r.Left();
                out.corrupt = !out.truncated;
                break;
            }
            const uint8_t* body = r.Cursor();
            r.Skip(stored);
            DecodedBlock b;
            const bool got = compressed ? aComp.Decompress(body, stored, rawSize, raw) : (raw.assign(body, body + stored), true);
            if (!got || raw.size() != rawSize || !DecodeBlockPayload(stream, frames, out.header.Columns(stream), raw.data(), raw.size(), b))
            {
                out.corrupt = true;
                break;
            }
            out.blocks.push_back(std::move(b));
        }
        else if (kind == 2)
        {
            Trailer t;
            t.failed = (r.U32() & 1u) != 0;
            t.frames[0] = r.U64(); t.frames[1] = r.U64();
            t.dropped[0] = r.U64(); t.dropped[1] = r.U64();
            if (!r.Ok())
            {
                out.truncated = true;
                break;
            }
            t.present = true;
            out.trailer = t;
        }
        else
        {
            out.corrupt = r.Ok();
            out.truncated = !r.Ok();
            break;
        }
    }
    return out;
}
} // namespace utpp::mm::posecapture
