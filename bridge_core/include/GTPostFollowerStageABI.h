#pragma once

#include <cstdint>

#if defined(_WIN32)
#    define GT_POST_FOLLOWER_CALL __cdecl
#else
#    define GT_POST_FOLLOWER_CALL
#endif

inline constexpr std::uint32_t GT_POST_FOLLOWER_STAGE_ABI_V1 = 1U;
inline constexpr char GT_POST_FOLLOWER_REGISTER_EXPORT_V1[] =
    "GTSoftbody_RegisterPostFollowerUpdateV1";

using GTPostFollowerUpdateCallbackV1 =
    void(GT_POST_FOLLOWER_CALL*)(void* context) noexcept;

enum class GTPostFollowerRegisterStatusV1 : std::uint32_t
{
    success = 0U,
    alreadyRegistered = 1U,
    incompatibleAbi = 2U,
    invalidCallback = 3U,
    callbackConflict = 4U,
};

using GTRegisterPostFollowerUpdateV1 = GTPostFollowerRegisterStatusV1(
    GT_POST_FOLLOWER_CALL*)(
        std::uint32_t abiVersion,
        GTPostFollowerUpdateCallbackV1 callback,
        void* context) noexcept;
