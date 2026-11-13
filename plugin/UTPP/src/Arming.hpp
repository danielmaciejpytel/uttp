#pragma once
#include <cstdint>
#include <vector>

namespace utpp
{
struct ArmInfo
{
    void* entity = nullptr;
    void* animObj = nullptr;
    void* metaRig = nullptr;
    void* playerDesc = nullptr; // *(animObj+0x18)
    uint64_t trackA = 0;
    uint64_t trackB = 0;
    uint32_t boneCount = 0;
    int32_t boneIndex = -1; // -1 not found, -2 exception while reading bone names
};

// Arming result codes (UTPPArm return value). >= 0 is a bitmask: 1 = trackA, 2 = trackB,
// 4 = target bone resolved.
enum ArmCode : int32_t
{
    kArmBusy = -1,
    kArmNoPlayer = -2,
    kArmNoAnimSystem = -3,
    kArmNoRootAnimObj = -4,
    kArmReadFault = -5,
    kArmNoTracks = -6,
};

// Game thread only. Returns the player entity pointer or nullptr (uses GetPlayer;GameInstance).
void* GetPlayerEntityRaw();

// Game thread only. Fills aInfo; returns an ArmCode (< 0) or the bitmask (>= 0).
int32_t ResolvePlayer(uint64_t aBoneNameHash, ArmInfo& aInfo);

// Game thread only. Resolve a bone name hash in a metaRig (SEH-guarded).
// Returns index, -1 not found, -2 fault.
int32_t ResolveBoneIndex(void* aMetaRig, uint64_t aBoneNameHash, uint32_t& aOutCount);

// Game thread only. Copies the metaRig bone name hashes and parent indices (SEH-guarded). aOutParents is left
// empty if the parent array size differs from the name count. Returns false on fault / implausible rig.
bool ReadRigNamesAndParents(void* aMetaRig, std::vector<uint64_t>& aOutHashes, std::vector<int16_t>& aOutParents);
} // namespace utpp
