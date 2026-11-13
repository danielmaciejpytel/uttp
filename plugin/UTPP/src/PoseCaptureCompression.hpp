#pragma once
// Block compression of the compact pose capture through the Compression API built into Windows (compressapi.h, Cabinet.dll): XPRESS with
// Huffman coding or LZMS, buffer mode (no COMPRESS_RAW), so a reader needs only the matching Decompress call (the converter tool uses
// ctypes on cabinet.dll). One instance per thread: the handles are not shared.
#include <Windows.h>
#include <compressapi.h>

#include <vector>

#include "mm/PoseCaptureFormat.hpp"

namespace utpp::posecapture
{
class WindowsCompressor final : public mm::posecapture::ICompressor
{
public:
    // aAlgorithm: mm::posecapture::kAlgorithmXpress (4) or kAlgorithmLzms (5) = the COMPRESS_ALGORITHM_* values.
    explicit WindowsCompressor(uint32_t aAlgorithm) : m_algorithm(aAlgorithm)
    {
        if (!CreateCompressor(aAlgorithm, nullptr, &m_compressor))
            m_compressor = nullptr;
        if (!CreateDecompressor(aAlgorithm, nullptr, &m_decompressor))
            m_decompressor = nullptr;
    }
    ~WindowsCompressor() override
    {
        if (m_compressor)
            CloseCompressor(m_compressor);
        if (m_decompressor)
            CloseDecompressor(m_decompressor);
    }
    WindowsCompressor(const WindowsCompressor&) = delete;
    WindowsCompressor& operator=(const WindowsCompressor&) = delete;

    bool Valid() const { return m_compressor != nullptr && m_decompressor != nullptr; }
    uint32_t Algorithm() const override { return m_algorithm; }
    bool Compress(const uint8_t* aSrc, size_t aSize, std::vector<uint8_t>& aOut) override
    {
        if (!m_compressor)
            return false;
        SIZE_T need = 0;
        if (!::Compress(m_compressor, aSrc, aSize, nullptr, 0, &need) && GetLastError() != ERROR_INSUFFICIENT_BUFFER)
            return false;
        aOut.resize(need);
        SIZE_T got = 0;
        if (!::Compress(m_compressor, aSrc, aSize, aOut.data(), aOut.size(), &got))
            return false;
        aOut.resize(got);
        return true;
    }
    bool Decompress(const uint8_t* aSrc, size_t aStored, size_t aRawSize, std::vector<uint8_t>& aOut) override
    {
        if (!m_decompressor)
            return false;
        aOut.resize(aRawSize);
        SIZE_T got = 0;
        if (!::Decompress(m_decompressor, aSrc, aStored, aOut.data(), aOut.size(), &got) || got != aRawSize)
            return false;
        return true;
    }

private:
    uint32_t m_algorithm;
    COMPRESSOR_HANDLE m_compressor = nullptr;
    DECOMPRESSOR_HANDLE m_decompressor = nullptr;
};
} // namespace utpp::posecapture
