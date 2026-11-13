// Mutation selection of the offline mutation tests (tests only; production code only carries the compile-time constant 0).
//
// Production builds (family macro 0 / undefined): the expression forms vanish and the statement forms are "if constexpr (false)", so the code is that
// of the former "#if" blocks. A compile-time mutant (UTPP_<FAM>_MUTATION=k) compares the macro with k. A mutation test executable compiled with UTPP_RUNTIME_MUTATION_<FAM> (tests/*.cmake)
// reads the mutation number from the environment variable UTPP_MUTATION_<FAM> instead (cached at the first read; unset / empty / not a
// number = 0 = production behaviour), so ONE executable serves every run-time mutant of the family and ctest runs it once per mutation.
// Mutants whose site cannot be a run-time branch (declarations, types, constants in constant contexts, code that does not compile both
// ways) stay compile-time executables with a plain "#if" on UTPP_<FAM>_MUTATION; UTPP_RUNTIME_MUTATION_<FAM> leaves that macro undefined.
//
//   UTPP_MUT(FAM, k)             expression: true when mutation k of the family is selected
//   UTPP_IF_MUT(FAM, k) / UTPP_IF_NOT_MUT(FAM, k)   statement: "if constexpr (...)" in production, a plain "if (...)" in a run-time executable of that family
//   UTPP_MUT_IF (cond)           the same switch for the "if constexpr (kMutation ...)" sites of VehicleTransition.hpp
//   UTPP_MUT_CONSTEXPR_VEHEXIT   "constexpr" except in the run-time VEHEXIT executable (functions with a UTPP_MUT(VEHEXIT, k) site)
//   utpp::mutsel::RuntimeValue   stands in for a family's "constexpr int kMutation" in a run-time executable (approach, vehtrans)
// Families: APPROACH (src/input/ApproachCore.hpp), VEHTRANS (src/mm/VehicleTransition.hpp), JUMPPICK (src/mm/ChooserMatcher.hpp,
// MotionAdapter.hpp, MotionDriver.hpp), VEHEXIT (src/MotionHost.cpp, src/mm/GameIntegration.hpp, VehicleTransition.hpp).
#pragma once

#if defined(UTPP_RUNTIME_MUTATION_APPROACH) || defined(UTPP_RUNTIME_MUTATION_VEHTRANS) || defined(UTPP_RUNTIME_MUTATION_JUMPPICK) || \
    defined(UTPP_RUNTIME_MUTATION_VEHEXIT)
#include <cstdlib>

namespace utpp::mutsel
{
// environment variable -> mutation number (0 when unset / empty / not a number)
inline int ReadEnv(const char* aVar)
{
    char* v = nullptr;
    size_t n = 0;
    int r = 0;
    if (_dupenv_s(&v, &n, aVar) == 0 && v)
    {
        char* end = nullptr;
        const long x = std::strtol(v, &end, 10);
        if (end != v)
            r = static_cast<int>(x);
    }
    std::free(v);
    return r;
}
// one cached value per family (function-local static: no static-initialisation-order dependence)
template <int Family>
inline int Cached(const char* aVar)
{
    static const int s_v = ReadEnv(aVar);
    return s_v;
}
// stands in for the constexpr int kMutation of a family: non-constexpr conversion, so "kMutation == 5" is evaluated at run time
template <int Family>
struct RuntimeValue
{
    const char* var;
    operator int() const { return Cached<Family>(var); }
};
} // namespace utpp::mutsel
#endif

// ---- per family: the selected mutation ----
#if defined(UTPP_RUNTIME_MUTATION_APPROACH)
#define UTPP_MUT_APPROACH(k) (::utpp::mutsel::Cached<1>("UTPP_MUTATION_APPROACH") == (k))
#define UTPP_MUT_IF_APPROACH if
#define UTPP_AN_APPROACH(k) !UTPP_MUT_APPROACH(k) &&
#define UTPP_OR_APPROACH(k) UTPP_MUT_APPROACH(k) ||
#define UTPP_TN_APPROACH(k, a, b) (UTPP_MUT_APPROACH(k) ? (a) : (b))
#elif defined(UTPP_APPROACH_MUTATION) && (UTPP_APPROACH_MUTATION) != 0
#define UTPP_MUT_APPROACH(k) ((UTPP_APPROACH_MUTATION) == (k))
#define UTPP_MUT_IF_APPROACH if constexpr
#define UTPP_AN_APPROACH(k) !UTPP_MUT_APPROACH(k) &&
#define UTPP_OR_APPROACH(k) UTPP_MUT_APPROACH(k) ||
#define UTPP_TN_APPROACH(k, a, b) (UTPP_MUT_APPROACH(k) ? (a) : (b))
#else // production (mutation 0): the expression forms vanish
#define UTPP_MUT_APPROACH(k) false
#define UTPP_MUT_IF_APPROACH if constexpr
#define UTPP_AN_APPROACH(k)
#define UTPP_OR_APPROACH(k)
#define UTPP_TN_APPROACH(k, a, b) (b)
#endif

#if defined(UTPP_RUNTIME_MUTATION_VEHTRANS)
#define UTPP_MUT_VEHTRANS(k) (::utpp::mutsel::Cached<2>("UTPP_MUTATION_VEHTRANS") == (k))
#define UTPP_MUT_IF_VEHTRANS if
#define UTPP_AN_VEHTRANS(k) !UTPP_MUT_VEHTRANS(k) &&
#define UTPP_OR_VEHTRANS(k) UTPP_MUT_VEHTRANS(k) ||
#define UTPP_TN_VEHTRANS(k, a, b) (UTPP_MUT_VEHTRANS(k) ? (a) : (b))
#elif defined(UTPP_VEHTRANS_MUTATION) && (UTPP_VEHTRANS_MUTATION) != 0
#define UTPP_MUT_VEHTRANS(k) ((UTPP_VEHTRANS_MUTATION) == (k))
#define UTPP_MUT_IF_VEHTRANS if constexpr
#define UTPP_AN_VEHTRANS(k) !UTPP_MUT_VEHTRANS(k) &&
#define UTPP_OR_VEHTRANS(k) UTPP_MUT_VEHTRANS(k) ||
#define UTPP_TN_VEHTRANS(k, a, b) (UTPP_MUT_VEHTRANS(k) ? (a) : (b))
#else // production (mutation 0): the expression forms vanish
#define UTPP_MUT_VEHTRANS(k) false
#define UTPP_MUT_IF_VEHTRANS if constexpr
#define UTPP_AN_VEHTRANS(k)
#define UTPP_OR_VEHTRANS(k)
#define UTPP_TN_VEHTRANS(k, a, b) (b)
#endif

#if defined(UTPP_RUNTIME_MUTATION_JUMPPICK)
#define UTPP_MUT_JUMPPICK(k) (::utpp::mutsel::Cached<3>("UTPP_MUTATION_JUMPPICK") == (k))
#define UTPP_MUT_IF_JUMPPICK if
#define UTPP_AN_JUMPPICK(k) !UTPP_MUT_JUMPPICK(k) &&
#define UTPP_OR_JUMPPICK(k) UTPP_MUT_JUMPPICK(k) ||
#define UTPP_TN_JUMPPICK(k, a, b) (UTPP_MUT_JUMPPICK(k) ? (a) : (b))
#elif defined(UTPP_JUMPPICK_MUTATION) && (UTPP_JUMPPICK_MUTATION) != 0
#define UTPP_MUT_JUMPPICK(k) ((UTPP_JUMPPICK_MUTATION) == (k))
#define UTPP_MUT_IF_JUMPPICK if constexpr
#define UTPP_AN_JUMPPICK(k) !UTPP_MUT_JUMPPICK(k) &&
#define UTPP_OR_JUMPPICK(k) UTPP_MUT_JUMPPICK(k) ||
#define UTPP_TN_JUMPPICK(k, a, b) (UTPP_MUT_JUMPPICK(k) ? (a) : (b))
#else // production (mutation 0): the expression forms vanish
#define UTPP_MUT_JUMPPICK(k) false
#define UTPP_MUT_IF_JUMPPICK if constexpr
#define UTPP_AN_JUMPPICK(k)
#define UTPP_OR_JUMPPICK(k)
#define UTPP_TN_JUMPPICK(k, a, b) (b)
#endif

#if defined(UTPP_RUNTIME_MUTATION_VEHEXIT)
#define UTPP_MUT_VEHEXIT(k) (::utpp::mutsel::Cached<4>("UTPP_MUTATION_VEHEXIT") == (k))
#define UTPP_MUT_IF_VEHEXIT if
#define UTPP_AN_VEHEXIT(k) !UTPP_MUT_VEHEXIT(k) &&
#define UTPP_OR_VEHEXIT(k) UTPP_MUT_VEHEXIT(k) ||
#define UTPP_TN_VEHEXIT(k, a, b) (UTPP_MUT_VEHEXIT(k) ? (a) : (b))
#elif defined(UTPP_VEHEXIT_MUTATION) && (UTPP_VEHEXIT_MUTATION) != 0
#define UTPP_MUT_VEHEXIT(k) ((UTPP_VEHEXIT_MUTATION) == (k))
#define UTPP_MUT_IF_VEHEXIT if constexpr
#define UTPP_AN_VEHEXIT(k) !UTPP_MUT_VEHEXIT(k) &&
#define UTPP_OR_VEHEXIT(k) UTPP_MUT_VEHEXIT(k) ||
#define UTPP_TN_VEHEXIT(k, a, b) (UTPP_MUT_VEHEXIT(k) ? (a) : (b))
#else // production (mutation 0): the expression forms vanish
#define UTPP_MUT_VEHEXIT(k) false
#define UTPP_MUT_IF_VEHEXIT if constexpr
#define UTPP_AN_VEHEXIT(k)
#define UTPP_OR_VEHEXIT(k)
#define UTPP_TN_VEHEXIT(k, a, b) (b)
#endif

#define UTPP_MUT(FAM, k) UTPP_MUT_##FAM(k)

// Statement forms: "if constexpr" (the guarded statement is discarded at compile time) in production and compile-time mutant builds, a plain
// "if" in a run-time executable of that family. UTPP_IF_MUT(FAM, k) = "if (mutation k)", UTPP_IF_NOT_MUT(FAM, k) = "if (!mutation k)"; both also
// work after an "else". UTPP_MUT_IF (cond) is the same switch for the "if constexpr (kMutation ...)" sites of VehicleTransition.hpp.
#define UTPP_IF_MUT(FAM, k) UTPP_MUT_IF_##FAM (UTPP_MUT(FAM, k))
#define UTPP_IF_NOT_MUT(FAM, k) UTPP_MUT_IF_##FAM (!UTPP_MUT(FAM, k))
#define UTPP_MUT_IF UTPP_MUT_IF_VEHTRANS

// Expression forms. In production (family macro 0 / undefined) they expand to NOTHING: the token stream of the expression is the original one, so the
// generated code is that of the former "#if" version. Otherwise they test the mutation:
//   UTPP_AND_NOT(FAM, k) x   "!mutation && x"       (the mutation switches the operand x off)
//   UTPP_OR(FAM, k) x        "mutation || x"        (the mutation forces the disjunction true)
//   UTPP_TERN(FAM, k, a, b)  "(mutation ? a : b)"   (production: "(b)")
#define UTPP_AND_NOT(FAM, k) UTPP_AN_##FAM(k)
#define UTPP_OR(FAM, k) UTPP_OR_##FAM(k)
#define UTPP_TERN(FAM, k, a, b) UTPP_TN_##FAM(k, a, b)

#if defined(UTPP_RUNTIME_MUTATION_VEHEXIT)
#define UTPP_MUT_CONSTEXPR_VEHEXIT
#else
#define UTPP_MUT_CONSTEXPR_VEHEXIT constexpr
#endif
