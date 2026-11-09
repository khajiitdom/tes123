#ifndef GT_CONTACT_BRIDGE_ABI_H
#define GT_CONTACT_BRIDGE_ABI_H

#include <stddef.h>
#include <stdint.h>

#if defined(_WIN32)
#define GT_CONTACT_CALL __cdecl
#if defined(GT_CONTACT_BRIDGE_EXPORTS)
#define GT_CONTACT_API __declspec(dllexport)
#elif defined(GT_CONTACT_BRIDGE_IMPORTS)
#define GT_CONTACT_API __declspec(dllimport)
#else
#define GT_CONTACT_API
#endif
#else
#define GT_CONTACT_CALL
#define GT_CONTACT_API
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* ABI v1.1 adds an explicit target-generation handshake while retaining the
 * v1 export name and fixed structure sizes.  Exact version/size validation
 * prevents an older caller from invoking the changed setTargetSkeletons
 * signature. */
#define GT_CONTACT_ABI_V1 UINT32_C(0x00010001)
#define GT_CONTACT_ABI_MAJOR(version) ((uint32_t)(version) >> 16U)
#define GT_CONTACT_ABI_MINOR(version) ((uint32_t)(version) & UINT32_C(0x0000ffff))

typedef uint32_t GTContactStatusV1;

#define GT_CONTACT_STATUS_OK UINT32_C(0)
#define GT_CONTACT_STATUS_INVALID_ARGUMENT UINT32_C(1)
#define GT_CONTACT_STATUS_INCOMPATIBLE_ABI UINT32_C(2)
#define GT_CONTACT_STATUS_STRUCT_SIZE_MISMATCH UINT32_C(3)
#define GT_CONTACT_STATUS_RESERVED_NONZERO UINT32_C(4)
#define GT_CONTACT_STATUS_NO_FRAME UINT32_C(5)
#define GT_CONTACT_STATUS_BUFFER_TOO_SMALL UINT32_C(6)
#define GT_CONTACT_STATUS_GENERATION_EXHAUSTED UINT32_C(7)
#define GT_CONTACT_STATUS_TARGET_GENERATION_MISMATCH UINT32_C(8)

#define GT_CONTACT_ENDPOINT_DYNAMIC UINT32_C(0x00000001)
#define GT_CONTACT_ENDPOINT_KINEMATIC UINT32_C(0x00000002)
#define GT_CONTACT_ENDPOINT_BREAST UINT32_C(0x00000004)
#define GT_CONTACT_ENDPOINT_EXTERNAL_COLLIDER UINT32_C(0x00000008)
#define GT_CONTACT_ENDPOINT_STATIC UINT32_C(0x00000010)
#define GT_CONTACT_ENDPOINT_KNOWN_FLAGS UINT32_C(0x0000001f)

#define GT_CONTACT_SAMPLE_CANONICAL_ENDPOINT_ORDER UINT32_C(0x00000001)
#define GT_CONTACT_SAMPLE_BREAST_RELEVANT UINT32_C(0x00000002)
#define GT_CONTACT_SAMPLE_KNOWN_FLAGS UINT32_C(0x00000003)

#define GT_CONTACT_FRAME_DEDUPLICATED UINT32_C(0x00000001)
#define GT_CONTACT_FRAME_INVALID_INPUT_DROPPED UINT32_C(0x00000002)
#define GT_CONTACT_FRAME_INPUT_CAP_REACHED UINT32_C(0x00000004)
#define GT_CONTACT_FRAME_PUBLISH_CAP_REACHED UINT32_C(0x00000008)
#define GT_CONTACT_FRAME_KNOWN_FLAGS UINT32_C(0x0000000f)

#define GT_CONTACT_BONE_NAME_CAPACITY UINT32_C(64)
#define GT_CONTACT_MAX_TARGET_SKELETONS UINT32_C(64)

/*
 * DLL-boundary rules for ABI v1:
 * - Every structure is POD and versioned by structSize.
 * - Every reserved field must be zero on input.
 * - All output storage belongs to the caller; the producer never returns an
 *   engine object, STL container, allocation, or ownership-bearing pointer.
 * - Tokens are opaque integer identities. They must never be dereferenced by
 *   the consumer.
 * - Coordinates are FSMP local-world coordinates. originOffset in the frame
 *   converts them to Skyrim world coordinates when required.
 */

typedef struct GTContactEndpointV1 {
    uint32_t structSize;
    uint32_t flags;
    uint64_t skeletonToken;
    uint64_t rigidBodyToken;
    uint64_t boneNameHash;
    char boneName[GT_CONTACT_BONE_NAME_CAPACITY];
    uint64_t reserved[4];
} GTContactEndpointV1;

typedef struct GTContactSampleV1 {
    uint32_t structSize;
    uint32_t flags;
    uint64_t frameSequence;
    uint32_t substepIndex;
    uint32_t reserved0;
    float timeStep;
    float penetrationDepth;
    float normalImpulse;
    float reservedScalar;
    GTContactEndpointV1 endpointA;
    GTContactEndpointV1 endpointB;
    float pointOnA[3];
    float pointOnB[3];
    float normalOnB[3];
    float relativeVelocityAminusB[3];
    uint64_t reserved[4];
} GTContactSampleV1;

typedef struct GTContactFrameV1 {
    uint32_t structSize;
    uint32_t abiVersion;
    uint32_t sampleStructSize;
    uint32_t flags;
    uint64_t frameSequence;
    float frameTimeStep;
    float originOffset[3];
    uint32_t substepCount;
    uint32_t availableSampleCount;
    uint32_t totalObservedSampleCount;
    uint32_t validObservedSampleCount;
    uint32_t retainedCandidateCount;
    uint32_t publishedSampleCount;
    uint32_t droppedSampleCount;
    uint32_t invalidSampleCount;
    uint32_t duplicateSampleCount;
    uint32_t inputOverflowSampleCount;
    uint32_t reserved0;
    /* Input/output handshake for copyLatestFrame.  The caller supplies the
     * non-zero generation returned by its latest successful
     * setTargetSkeletons call.  If that expectation no longer owns the
     * producer's current target generation, the producer returns
     * TARGET_GENERATION_MISMATCH without modifying frame/sample outputs.  If
     * it still owns the current generation but no frame for it has been
     * published yet, the producer returns NO_FRAME. */
    uint64_t targetGeneration;
    uint64_t reserved[3];
} GTContactFrameV1;

typedef GTContactStatusV1(GT_CONTACT_CALL* GTContactCopyLatestFrameV1)(
    GTContactFrameV1* outFrame,
    GTContactSampleV1* outSamples,
    uint32_t sampleCapacity,
    uint32_t* outCopiedSampleCount);

/* Capture is producer-side and disabled by default.  A validated consumer
 * explicitly enables it, so an unused bridge build preserves stock FSMP
 * behavior and avoids manifold-copy overhead. */
typedef GTContactStatusV1(GT_CONTACT_CALL* GTContactSetCaptureEnabledV1)(
    uint32_t enabled);

/* Replaces the complete producer-side allowlist atomically and returns a new,
 * process-lifetime, monotonically increasing non-zero generation.  A zero
 * count clears the allowlist and causes the producer to publish empty frames
 * until a consumer supplies active NPC-root identity tokens.  Every
 * successful call advances the generation, including an identical target
 * list; this makes reconnect a clean epoch boundary. */
typedef GTContactStatusV1(GT_CONTACT_CALL* GTContactSetTargetSkeletonsV1)(
    const uint64_t* skeletonTokens,
    uint32_t skeletonTokenCount,
    uint64_t* outTargetGeneration);

typedef struct GTContactBridgeV1 {
    uint32_t structSize;
    uint32_t abiVersion;
    uint32_t frameStructSize;
    uint32_t sampleStructSize;
    uint32_t endpointStructSize;
    uint32_t flags;
    GTContactCopyLatestFrameV1 copyLatestFrame;
    GTContactSetCaptureEnabledV1 setCaptureEnabled;
    GTContactSetTargetSkeletonsV1 setTargetSkeletons;
    uint64_t reserved[4];
} GTContactBridgeV1;

/* Implemented by a future patched FSMP producer DLL, not by bridge_core. */
GT_CONTACT_API const GTContactBridgeV1* GT_CONTACT_CALL GTFSMP_GetContactBridgeV1(
    uint32_t requestedAbiVersion,
    uint32_t callerBridgeStructSize);

#ifdef __cplusplus
}

#include <type_traits>

static_assert(std::is_standard_layout_v<GTContactEndpointV1>);
static_assert(std::is_trivial_v<GTContactEndpointV1>);
static_assert(std::is_standard_layout_v<GTContactSampleV1>);
static_assert(std::is_trivial_v<GTContactSampleV1>);
static_assert(std::is_standard_layout_v<GTContactFrameV1>);
static_assert(std::is_trivial_v<GTContactFrameV1>);
static_assert(std::is_standard_layout_v<GTContactBridgeV1>);
static_assert(std::is_trivial_v<GTContactBridgeV1>);

#if UINTPTR_MAX == UINT64_MAX
static_assert(sizeof(GTContactEndpointV1) == 128);
static_assert(sizeof(GTContactSampleV1) == 376);
static_assert(sizeof(GTContactFrameV1) == 120);
static_assert(sizeof(GTContactBridgeV1) == 80);
#endif
#endif

#endif
