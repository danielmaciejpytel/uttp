// Mutation-test early exit (tests only, never part of the DLL).
// ctest sets UTPP_TEST_STOP_ON=<the test's PASS_REGULAR_EXPRESSION> (or "FAIL" for WILL_FAIL tests) on every mutation test
// (tests/labels.cmake). A test's Check()/Expect() calls UtppFailFastNote(line) for every FAIL line it prints: the FAIL lines seen so far
// are joined with a space and searched with the expected regex; on a match the process flushes and exits with 1. ctest then sees
// the same expected FAIL text it would have seen at the end of the full run (a mutant only has to show it), just sooner.
// No variable / no match / a regex std::regex cannot compile = no effect: the test runs to the end exactly as before.
#pragma once
#include <cstdio>
#include <cstdlib>
#include <regex>
#include <string>

inline void UtppFailFastNote(const std::string& aFailLine)
{
    static const std::string s_expr = [] {
        char* v = nullptr;
        size_t n = 0;
        std::string s;
        if (_dupenv_s(&v, &n, "UTPP_TEST_STOP_ON") == 0 && v)
            s = v;
        std::free(v);
        return s;
    }();
    if (s_expr.empty())
        return;
    static std::string s_seen;
    static bool s_bad = false;
    static std::regex s_rx = [] {
        try { return std::regex(s_expr); } catch (...) { s_bad = true; return std::regex(); }
    }();
    if (s_bad)
        return;
    if (!s_seen.empty())
        s_seen += ' ';
    s_seen += aFailLine;
    if (std::regex_search(s_seen, s_rx))
    {
        std::printf("UTPP_TEST_STOP_ON matched: stopping the mutation test early\n");
        std::fflush(stdout);
        std::fflush(stderr);
        std::_Exit(1);
    }
}
