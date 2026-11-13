// Player arming (game thread only).
//
// Logic and offsets adapted from cyberpunk-vr-port (MIT License, Copyright (c) 2026 dariulone):
//  - src/Natives/AnimLookup.cpp: FindPlayerEntity (GetPlayer;GameInstance), world animation system
//    lookup (script interface, fallback runtimeScene+0x8 -> +0x38), and
//    FindPlayerAnimatedObjectByComponentName (bucket walk, component name "root").
//  - src/Natives/VrikArming.cpp: trackA = *(*(animObj+0x8)+0x40), trackB = *(*(animObj+0x18)+0x18),
//    bone index resolution through metaRig->boneNames.
// Every raw dereference below runs inside an SEH frame in a function without C++ unwinding.

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>

#include <RED4ext/RED4ext.hpp>
#include <RED4ext/Containers/StaticArray.hpp> // required before worldAnimationSystem.hpp (not self-contained)
#include <RED4ext/Scripting/Natives/Generated/ent/AnimatedComponent.hpp>
#include <RED4ext/Scripting/Natives/ScriptGameInstance.hpp>
#include <RED4ext/Scripting/Natives/animAnimatedObject.hpp>
#include <RED4ext/Scripting/Natives/animMetaRig.hpp>
#include <RED4ext/Scripting/Natives/entEntity.hpp>
#include <RED4ext/Scripting/Natives/worldAnimationSystem.hpp>
#include <RED4ext/Scripting/Natives/worldAnimationSystemScriptInterface.hpp>

#include "Arming.hpp"

namespace utpp
{
namespace
{
constexpr uint64_t kRootHash = RED4ext::CName("root").hash;
constexpr uint64_t kAnimSystemTypeHash = RED4ext::CName("worldAnimationSystem").hash;

// ---- SEH-only helpers (no objects with destructors) ----

RED4ext::world::AnimationSystem* SehAnimSystemFromScriptInterface(RED4ext::GameInstance* aGame, RED4ext::CClass* aCls)
{
    __try
    {
        auto* iface = reinterpret_cast<RED4ext::world::AnimationSystemScriptInterface*>(aGame->GetSystem(aCls));
        return iface ? iface->animationSystem : nullptr;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return nullptr;
    }
}

RED4ext::world::AnimationSystem* SehAnimSystemFromScene(RED4ext::ent::Entity* aEntity)
{
    __try
    {
        auto* scene = reinterpret_cast<uint8_t*>(aEntity->runtimeScene);
        if (!scene)
            return nullptr;
        auto* parent = *reinterpret_cast<uint8_t**>(scene + 0x8);
        if (!parent)
            return nullptr;
        auto* cand = *reinterpret_cast<RED4ext::IScriptable**>(parent + 0x38);
        if (!cand)
            return nullptr;
        RED4ext::CClass* type = cand->GetType();
        if (type && type->name.hash == kAnimSystemTypeHash)
            return reinterpret_cast<RED4ext::world::AnimationSystem*>(cand);
        return nullptr;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return nullptr;
    }
}

// Returns 1 found, 0 not found, -1 fault.
int SehFindRootAnimObj(RED4ext::world::AnimationSystem* aSys, void* aEntity, RED4ext::anim::AnimatedObject** aOut)
{
    __try
    {
        for (uint32_t b = 0; b < RED4ext::world::AnimationSystem::BucketCount; ++b)
        {
            auto& bucket = aSys->entitityBuckets[b];
            const uint32_t n = static_cast<uint32_t>(bucket.entities.size());
            if (n > RED4ext::world::AnimatedEntitiesBucket::MaxEntries)
                continue;
            for (uint32_t i = 0; i < n; ++i)
            {
                if (bucket.entities[i].instance != aEntity)
                    continue;
                auto* comp = bucket.animatedComponents[i];
                if (comp && comp->name.hash == kRootHash)
                {
                    *aOut = bucket.animatedObjects[i];
                    return *aOut ? 1 : 0;
                }
            }
        }
        return 0;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return -1;
    }
}

bool SehReadTracks(uint8_t* aAnimObj, uint64_t* aA, uint64_t* aB, void** aMetaRig, void** aDesc)
{
    __try
    {
        uint8_t* ownerA = *reinterpret_cast<uint8_t**>(aAnimObj + 0x8); // == metaRig in the SDK layout
        uint8_t* ownerB = *reinterpret_cast<uint8_t**>(aAnimObj + 0x18);
        *aMetaRig = ownerA;
        *aDesc = ownerB;
        *aA = ownerA ? *reinterpret_cast<uint64_t*>(ownerA + 0x40) : 0;
        *aB = ownerB ? *reinterpret_cast<uint64_t*>(ownerB + 0x18) : 0;
        return true;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return false;
    }
}

int32_t SehResolveBone(RED4ext::anim::MetaRig* aRig, uint64_t aHash, uint32_t* aCount)
{
    __try
    {
        const uint32_t n = static_cast<uint32_t>(aRig->boneNames.size());
        *aCount = n;
        if (n == 0 || n > 8192 || !aRig->boneNames.Data())
            return -1;
        for (uint32_t i = 0; i < n; ++i)
        {
            if (aRig->boneNames.Data()[i].hash == aHash)
                return static_cast<int32_t>(i);
        }
        return -1;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return -2;
    }
}
// Returns name count (0 on fault / implausible) and parent count via aParents.
uint32_t SehRigCounts(RED4ext::anim::MetaRig* aRig, uint32_t* aParents)
{
    __try
    {
        const uint32_t n = static_cast<uint32_t>(aRig->boneNames.size());
        *aParents = static_cast<uint32_t>(aRig->parentIndeces.size());
        if (n == 0 || n > 8192 || !aRig->boneNames.Data())
            return 0;
        if (*aParents != n || !aRig->parentIndeces.Data())
            *aParents = 0;
        return n;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return 0;
    }
}

bool SehRigCopy(RED4ext::anim::MetaRig* aRig, uint64_t* aHashes, int16_t* aParents, uint32_t aN, uint32_t aNumParents)
{
    __try
    {
        for (uint32_t i = 0; i < aN; ++i)
            aHashes[i] = aRig->boneNames.Data()[i].hash;
        for (uint32_t i = 0; i < aNumParents; ++i)
            aParents[i] = aRig->parentIndeces.Data()[i];
        return true;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return false;
    }
}
} // namespace

bool ReadRigNamesAndParents(void* aMetaRig, std::vector<uint64_t>& aOutHashes, std::vector<int16_t>& aOutParents)
{
    aOutHashes.clear();
    aOutParents.clear();
    if (!aMetaRig)
        return false;
    auto* rig = static_cast<RED4ext::anim::MetaRig*>(aMetaRig);
    uint32_t np = 0;
    const uint32_t n = SehRigCounts(rig, &np);
    if (n == 0)
        return false;
    aOutHashes.resize(n);
    aOutParents.resize(np);
    if (!SehRigCopy(rig, aOutHashes.data(), aOutParents.data(), n, np))
    {
        aOutHashes.clear();
        aOutParents.clear();
        return false;
    }
    return true;
}

void* GetPlayerEntityRaw()
{
    RED4ext::ScriptGameInstance gameInstance;
    if (!gameInstance.instance)
        return nullptr;
    RED4ext::Handle<RED4ext::IScriptable> player;
    if (!RED4ext::ExecuteGlobalFunction("GetPlayer;GameInstance", &player, gameInstance))
        return nullptr;
    // The game keeps its own reference to the player; we only keep the raw pointer for identity.
    return player ? player.instance : nullptr;
}

int32_t ResolveBoneIndex(void* aMetaRig, uint64_t aBoneNameHash, uint32_t& aOutCount)
{
    aOutCount = 0;
    if (!aMetaRig || aBoneNameHash == 0)
        return -1;
    return SehResolveBone(static_cast<RED4ext::anim::MetaRig*>(aMetaRig), aBoneNameHash, &aOutCount);
}

int32_t ResolvePlayer(uint64_t aBoneNameHash, ArmInfo& aInfo)
{
    aInfo = ArmInfo{};
    void* entity = GetPlayerEntityRaw();
    if (!entity)
        return kArmNoPlayer;
    aInfo.entity = entity;

    RED4ext::world::AnimationSystem* sys = nullptr;
    auto* engine = RED4ext::CGameEngine::Get();
    auto* framework = engine ? engine->framework : nullptr;
    auto* game = framework ? framework->gameInstance : nullptr;
    auto* rtti = RED4ext::CRTTISystem::Get();
    auto* cls = rtti ? rtti->GetClass("worldAnimationSystemScriptInterface") : nullptr;
    if (game && cls)
        sys = SehAnimSystemFromScriptInterface(game, cls);
    if (!sys)
        sys = SehAnimSystemFromScene(static_cast<RED4ext::ent::Entity*>(entity));
    if (!sys)
        return kArmNoAnimSystem;

    RED4ext::anim::AnimatedObject* animObj = nullptr;
    const int found = SehFindRootAnimObj(sys, entity, &animObj);
    if (found < 0)
        return kArmReadFault;
    if (found == 0 || !animObj)
        return kArmNoRootAnimObj;
    aInfo.animObj = animObj;

    if (!SehReadTracks(reinterpret_cast<uint8_t*>(animObj), &aInfo.trackA, &aInfo.trackB, &aInfo.metaRig, &aInfo.playerDesc))
        return kArmReadFault;
    if (aInfo.trackA == 0 && aInfo.trackB == 0)
        return kArmNoTracks;

    aInfo.boneIndex = ResolveBoneIndex(aInfo.metaRig, aBoneNameHash, aInfo.boneCount);

    int32_t mask = 0;
    if (aInfo.trackA)
        mask |= 1;
    if (aInfo.trackB)
        mask |= 2;
    if (aInfo.boneIndex >= 0)
        mask |= 4;
    return mask;
}
} // namespace utpp
