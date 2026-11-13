#pragma once
// Authored recoil profiles: one pair of weapon-local sampler files (hip shot = "shoulder", aimed shot = "ironsight") per weapon
// family. The redscript glue (scripts/utpp_motion.reds, UTPPMotionRecoilProfileOfFamily) resolves the equipped record to an id and sends it
// with UTPPSetMMRecoilWeapon; MotionDriver loads <db dir>\recoil\<file> for every id on the worker and falls back to the plain
// authored aim pose (no recoil kick) for an id whose files are missing or invalid, and for id 0.
//
// Keep three places in step: this table, the family -> id lines of utpp_motion.reds (checked by RecoilProfilesTest) and PROFILE_SOURCES of
// tools/analysis/export_nue_native_recoil_curves.py (writes the files; the stem is the lower-case family tag). Ids are the index.
#include "AuthoredRecoil.hpp"
#include <array>
#include <cstddef>
#include <cstdint>
namespace utpp::mm::recoil_profile
{
inline constexpr int32_t kNone = 0;
inline constexpr int32_t kNue = 1; // Items.Preset_Nue_Default: the original profile (add_recoil_single_shot_*), also the only authored reload

struct Entry
{
    const char* family;    // lower-case weapon family tag; nullptr for the unused id 0 and for a retired id
    const char* shoulder;  // file name inside the recoil folder
    const char* ironsight;
    bool longGun;          // the family uses the Rifle overlay (rifles, SMGs); false = a pistol overlay (test selection only)
};

// Ids are never renumbered: a retired id keeps its slot (family nullptr), is not Valid(), is not loaded and is not returned by the glue.
// Retired: 8 nova, 9 overture, 10 burya. Their single_shot clips are not a recoil kick but a revolver hammer-cock: the weapon swings
// 7-10 cm sideways and rolls about the barrel a few tenths of a second after the shot. The hand IK follows 3-30% of it (shootScale
// 0.03 - 0.36 against 1.0 for every other profile), flickering the scale from frame to frame - the bisection thrash of the review. Even
// at 5 cm / 0.3 rad the scale stayed under 0.65. These revolvers keep the plain authored aim pose (fallback A).
inline constexpr std::array<Entry, 28> kTable = {{
    {nullptr, nullptr, nullptr, false},
    {"nue", "add_recoil_single_shot_shoulder.utpprecoil", "add_recoil_single_shot_ironsight.utpprecoil", false},
    {"liberty", "liberty_single_shot_shoulder.utpprecoil", "liberty_single_shot_ironsight.utpprecoil", false},
    {"unity", "unity_single_shot_shoulder.utpprecoil", "unity_single_shot_ironsight.utpprecoil", false},
    {"yukimura", "yukimura_single_shot_shoulder.utpprecoil", "yukimura_single_shot_ironsight.utpprecoil", false},
    {"chao", "chao_single_shot_shoulder.utpprecoil", "chao_single_shot_ironsight.utpprecoil", false},
    {"lexington", "lexington_single_shot_shoulder.utpprecoil", "lexington_single_shot_ironsight.utpprecoil", false},
    {"slaughtomatic", "slaughtomatic_single_shot_shoulder.utpprecoil", "slaughtomatic_single_shot_ironsight.utpprecoil", false},
    {nullptr, nullptr, nullptr, false}, // 8 nova (retired)
    {nullptr, nullptr, nullptr, false}, // 9 overture (retired)
    {nullptr, nullptr, nullptr, false}, // 10 burya (retired)
    {"quasar", "quasar_single_shot_shoulder.utpprecoil", "quasar_single_shot_ironsight.utpprecoil", false},
    {"ajax", "ajax_single_shot_shoulder.utpprecoil", "ajax_single_shot_ironsight.utpprecoil", true},
    {"copperhead", "copperhead_single_shot_shoulder.utpprecoil", "copperhead_single_shot_ironsight.utpprecoil", true},
    {"sidewinder", "sidewinder_single_shot_shoulder.utpprecoil", "sidewinder_single_shot_ironsight.utpprecoil", true},
    {"masamune", "masamune_single_shot_shoulder.utpprecoil", "masamune_single_shot_ironsight.utpprecoil", true},
    {"shingen", "shingen_single_shot_shoulder.utpprecoil", "shingen_single_shot_ironsight.utpprecoil", true},
    {"pulsar", "pulsar_single_shot_shoulder.utpprecoil", "pulsar_single_shot_ironsight.utpprecoil", true},
    {"saratoga", "saratoga_single_shot_shoulder.utpprecoil", "saratoga_single_shot_ironsight.utpprecoil", true},
    {"senkoh", "senkoh_single_shot_shoulder.utpprecoil", "senkoh_single_shot_ironsight.utpprecoil", true},
    // 2.5 additions (recoil-missing-20261109), each from its own single-shot clips of the 2.5 animset (same exporter settings).
    // Kenshin, Omaha, Ticon aim with add_recoil_charged_ironsight (no single_shot_ironsight in their animsets); Kenshin ADS
    // and Omaha use a per-family budget of 0.5 x their peak (hand-IK reach). Borg4a and Hercules records carry no family tag:
    // reached only through utpp_motion.reds UTPPMotionRecoilFamilyOfAnimset (drop ids 26-27 if that bridge is not taken).
    {"kenshin", "kenshin_single_shot_shoulder.utpprecoil", "kenshin_single_shot_ironsight.utpprecoil", false}, // 20 Kenshin
    {"omaha", "omaha_single_shot_shoulder.utpprecoil", "omaha_single_shot_ironsight.utpprecoil", false}, // 21 Omaha
    {"ticon", "ticon_single_shot_shoulder.utpprecoil", "ticon_single_shot_ironsight.utpprecoil", false}, // 22 Ticon
    {"defender", "defender_single_shot_shoulder.utpprecoil", "defender_single_shot_ironsight.utpprecoil", true}, // 23 Defender
    {"dian", "dian_single_shot_shoulder.utpprecoil", "dian_single_shot_ironsight.utpprecoil", true}, // 24 Dian
    {"warden", "warden_single_shot_shoulder.utpprecoil", "warden_single_shot_ironsight.utpprecoil", true}, // 25 Warden
    {"borg4a", "borg4a_single_shot_shoulder.utpprecoil", "borg4a_single_shot_ironsight.utpprecoil", true}, // 26 Borg4a
    {"hercules", "hercules_single_shot_shoulder.utpprecoil", "hercules_single_shot_ironsight.utpprecoil", true}, // 27 Hercules
}};
inline constexpr int32_t kSlots = static_cast<int32_t>(kTable.size()); // ids 0 .. kSlots - 1

inline constexpr bool Valid(int32_t aId) { return aId > 0 && aId < kSlots && kTable[static_cast<size_t>(aId)].family != nullptr; }

// Series-safe mode of the Player for every profile but the Nue (the Nue keeps the legacy accumulation, bit for bit).
// Per-shot budget of the exported curves (tools/analysis/export_nue_native_recoil_curves.py): 8 cm / 0.50 rad, ramp 40 ms. The Nue
// was validated in game at 5 cm / 0.655 rad; 8 cm is the top of the 5-8 cm range and 0.50 rad leaves 0.20 rad of the 0.70 rad
// clamp for the offset that is still carried.
//  series 0.12 m / 0.60 rad: carry + new kick stay under it (a burst stacks up to it, the clamp is 0.25 m / 0.70 rad);
//  speed 0.5 m/s / 1.5 rad/s: at most 1.7 cm and 2.9 degrees per frame at 30 fps (0.7 cm / 0.6 degrees at 144 fps).
inline authored_recoil::Limits LimitsOf(int32_t aId)
{
#if defined(UTPP_RECOIL_SERIES_MUTATION) && UTPP_RECOIL_SERIES_MUTATION == 1
    (void)aId;
    return {}; // mutation 1: every profile accumulates like the Nue (the pre-limit build)
#else
    if (aId == kNue || !Valid(aId)) return {};
    return {0.12f, 0.60f, 0.5f, 1.5f};
#endif
}
} // namespace utpp::mm::recoil_profile
