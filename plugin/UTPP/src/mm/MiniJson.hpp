// UTPP - minimal JSON reader (RFC 8259 subset sufficient for UTPP data files). Header-only.
//
// Parses UTF-8 text into a small DOM. Numbers are converted with std::from_chars (locale independent, correctly
// rounded), so a double written by Python repr() round-trips bit-exactly (including subnormals). Objects keep member
// order; duplicate keys keep the first one for Find(). Nesting depth is limited (kMaxDepth) and every error reports
// the byte offset. Allocates; meant for load time only (never per frame).
#pragma once

#include <charconv>
#include <cstdint>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace utpp::json
{
enum class Type : uint8_t
{
    Null,
    Bool,
    Number,
    String,
    Array,
    Object
};

struct Value
{
    Type type = Type::Null;
    bool b = false;
    double num = 0.0;
    bool isInteger = false; // the literal had no fraction / exponent
    std::string str;
    std::vector<Value> arr;
    std::vector<std::pair<std::string, Value>> obj;

    bool IsNull() const { return type == Type::Null; }
    bool IsBool() const { return type == Type::Bool; }
    bool IsNumber() const { return type == Type::Number; }
    bool IsString() const { return type == Type::String; }
    bool IsArray() const { return type == Type::Array; }
    bool IsObject() const { return type == Type::Object; }

    const Value* Find(std::string_view aKey) const
    {
        if (type != Type::Object)
            return nullptr;
        for (const auto& kv : obj)
            if (kv.first == aKey)
                return &kv.second;
        return nullptr;
    }
};

class Parser
{
public:
    static constexpr int kMaxDepth = 64;

    // Returns false and fills aError ("offset N: message") on malformed input.
    static bool Parse(std::string_view aText, Value& aOut, std::string& aError)
    {
        aOut = Value{};
        Parser p(aText);
        // optional UTF-8 BOM
        if (p.m_text.size() >= 3 && static_cast<unsigned char>(p.m_text[0]) == 0xEF &&
            static_cast<unsigned char>(p.m_text[1]) == 0xBB && static_cast<unsigned char>(p.m_text[2]) == 0xBF)
            p.m_pos = 3;
        p.SkipWs();
        if (!p.ParseValue(aOut, 0))
        {
            aError = p.m_error;
            return false;
        }
        p.SkipWs();
        if (p.m_pos != p.m_text.size())
        {
            p.Fail("trailing characters");
            aError = p.m_error;
            return false;
        }
        return true;
    }

private:
    explicit Parser(std::string_view aText) : m_text(aText) {}

    bool Fail(const char* aMsg)
    {
        if (m_error.empty())
            m_error = "offset " + std::to_string(m_pos) + ": " + aMsg;
        return false;
    }

    void SkipWs()
    {
        while (m_pos < m_text.size())
        {
            const char c = m_text[m_pos];
            if (c == ' ' || c == '\t' || c == '\n' || c == '\r')
                ++m_pos;
            else
                break;
        }
    }

    bool Literal(std::string_view aWord)
    {
        if (m_text.substr(m_pos, aWord.size()) != aWord)
            return Fail("invalid literal");
        m_pos += aWord.size();
        return true;
    }

    bool ParseValue(Value& aOut, int aDepth)
    {
        if (aDepth > kMaxDepth)
            return Fail("nesting too deep");
        if (m_pos >= m_text.size())
            return Fail("unexpected end of input");
        const char c = m_text[m_pos];
        switch (c)
        {
        case 'n':
            aOut.type = Type::Null;
            return Literal("null");
        case 't':
            aOut.type = Type::Bool;
            aOut.b = true;
            return Literal("true");
        case 'f':
            aOut.type = Type::Bool;
            aOut.b = false;
            return Literal("false");
        case '"':
            aOut.type = Type::String;
            return ParseString(aOut.str);
        case '[':
            return ParseArray(aOut, aDepth);
        case '{':
            return ParseObject(aOut, aDepth);
        default:
            if (c == '-' || (c >= '0' && c <= '9'))
                return ParseNumber(aOut);
            return Fail("unexpected character");
        }
    }

    bool ParseNumber(Value& aOut)
    {
        const size_t start = m_pos;
        size_t i = m_pos;
        const size_t n = m_text.size();
        auto digit = [&](size_t k) { return k < n && m_text[k] >= '0' && m_text[k] <= '9'; };
        if (i < n && m_text[i] == '-')
            ++i;
        if (!digit(i))
            return Fail("bad number");
        if (m_text[i] == '0')
            ++i;
        else
            while (digit(i))
                ++i;
        bool integer = true;
        if (i < n && m_text[i] == '.')
        {
            integer = false;
            ++i;
            if (!digit(i))
                return Fail("bad number fraction");
            while (digit(i))
                ++i;
        }
        if (i < n && (m_text[i] == 'e' || m_text[i] == 'E'))
        {
            integer = false;
            ++i;
            if (i < n && (m_text[i] == '+' || m_text[i] == '-'))
                ++i;
            if (!digit(i))
                return Fail("bad number exponent");
            while (digit(i))
                ++i;
        }
        double v = 0.0;
        const char* first = m_text.data() + start;
        const char* last = m_text.data() + i;
        const auto r = std::from_chars(first, last, v, std::chars_format::general);
        // result_out_of_range: from_chars leaves v unchanged; subnormals are converted (MSVC returns them in range).
        if (r.ec != std::errc{} || r.ptr != last)
            return Fail("number out of range");
        aOut.type = Type::Number;
        aOut.num = v;
        aOut.isInteger = integer;
        m_pos = i;
        return true;
    }

    static void AppendUtf8(std::string& s, uint32_t cp)
    {
        if (cp < 0x80)
            s.push_back(static_cast<char>(cp));
        else if (cp < 0x800)
        {
            s.push_back(static_cast<char>(0xC0 | (cp >> 6)));
            s.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
        }
        else if (cp < 0x10000)
        {
            s.push_back(static_cast<char>(0xE0 | (cp >> 12)));
            s.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
            s.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
        }
        else
        {
            s.push_back(static_cast<char>(0xF0 | (cp >> 18)));
            s.push_back(static_cast<char>(0x80 | ((cp >> 12) & 0x3F)));
            s.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
            s.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
        }
    }

    bool Hex4(uint32_t& aOut)
    {
        if (m_pos + 4 > m_text.size())
            return Fail("short \\u escape");
        uint32_t v = 0;
        for (int k = 0; k < 4; ++k)
        {
            const char c = m_text[m_pos++];
            v <<= 4;
            if (c >= '0' && c <= '9')
                v |= static_cast<uint32_t>(c - '0');
            else if (c >= 'a' && c <= 'f')
                v |= static_cast<uint32_t>(c - 'a' + 10);
            else if (c >= 'A' && c <= 'F')
                v |= static_cast<uint32_t>(c - 'A' + 10);
            else
                return Fail("bad \\u escape");
        }
        aOut = v;
        return true;
    }

    bool ParseString(std::string& aOut)
    {
        ++m_pos; // opening quote
        aOut.clear();
        while (true)
        {
            if (m_pos >= m_text.size())
                return Fail("unterminated string");
            const char c = m_text[m_pos++];
            if (c == '"')
                return true;
            if (static_cast<unsigned char>(c) < 0x20)
                return Fail("control character in string");
            if (c != '\\')
            {
                aOut.push_back(c);
                continue;
            }
            if (m_pos >= m_text.size())
                return Fail("unterminated escape");
            const char e = m_text[m_pos++];
            switch (e)
            {
            case '"': aOut.push_back('"'); break;
            case '\\': aOut.push_back('\\'); break;
            case '/': aOut.push_back('/'); break;
            case 'b': aOut.push_back('\b'); break;
            case 'f': aOut.push_back('\f'); break;
            case 'n': aOut.push_back('\n'); break;
            case 'r': aOut.push_back('\r'); break;
            case 't': aOut.push_back('\t'); break;
            case 'u':
            {
                uint32_t cp = 0;
                if (!Hex4(cp))
                    return false;
                if (cp >= 0xD800 && cp <= 0xDBFF)
                {
                    uint32_t lo = 0;
                    if (m_pos + 2 > m_text.size() || m_text[m_pos] != '\\' || m_text[m_pos + 1] != 'u')
                        return Fail("unpaired surrogate");
                    m_pos += 2;
                    if (!Hex4(lo))
                        return false;
                    if (lo < 0xDC00 || lo > 0xDFFF)
                        return Fail("bad low surrogate");
                    cp = 0x10000 + ((cp - 0xD800) << 10) + (lo - 0xDC00);
                }
                else if (cp >= 0xDC00 && cp <= 0xDFFF)
                    return Fail("unpaired surrogate");
                AppendUtf8(aOut, cp);
                break;
            }
            default:
                return Fail("bad escape");
            }
        }
    }

    bool ParseArray(Value& aOut, int aDepth)
    {
        aOut.type = Type::Array;
        ++m_pos;
        SkipWs();
        if (m_pos < m_text.size() && m_text[m_pos] == ']')
        {
            ++m_pos;
            return true;
        }
        while (true)
        {
            SkipWs();
            aOut.arr.emplace_back();
            if (!ParseValue(aOut.arr.back(), aDepth + 1))
                return false;
            SkipWs();
            if (m_pos >= m_text.size())
                return Fail("unterminated array");
            const char c = m_text[m_pos++];
            if (c == ']')
                return true;
            if (c != ',')
                return Fail("expected , or ]");
        }
    }

    bool ParseObject(Value& aOut, int aDepth)
    {
        aOut.type = Type::Object;
        ++m_pos;
        SkipWs();
        if (m_pos < m_text.size() && m_text[m_pos] == '}')
        {
            ++m_pos;
            return true;
        }
        while (true)
        {
            SkipWs();
            if (m_pos >= m_text.size() || m_text[m_pos] != '"')
                return Fail("expected object key");
            std::string key;
            if (!ParseString(key))
                return false;
            SkipWs();
            if (m_pos >= m_text.size() || m_text[m_pos] != ':')
                return Fail("expected :");
            ++m_pos;
            SkipWs();
            aOut.obj.emplace_back(std::move(key), Value{});
            if (!ParseValue(aOut.obj.back().second, aDepth + 1))
                return false;
            SkipWs();
            if (m_pos >= m_text.size())
                return Fail("unterminated object");
            const char c = m_text[m_pos++];
            if (c == '}')
                return true;
            if (c != ',')
                return Fail("expected , or }");
        }
    }

    std::string_view m_text;
    size_t m_pos = 0;
    std::string m_error;
};

inline bool Parse(std::string_view aText, Value& aOut, std::string& aError) { return Parser::Parse(aText, aOut, aError); }

} // namespace utpp::json
