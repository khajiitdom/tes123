#include "GTContactFrameCollector.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <utility>

namespace gt::contact {
namespace {

constexpr float kPointQuantization = 64.0F;
constexpr float kNormalQuantization = 4096.0F;

bool IsZero(const std::uint64_t* values, const std::size_t count) noexcept
{
    for (std::size_t index = 0; index < count; ++index) {
        if (values[index] != 0U) {
            return false;
        }
    }
    return true;
}

bool IsFinite3(const float values[3]) noexcept
{
    return std::isfinite(values[0]) && std::isfinite(values[1]) && std::isfinite(values[2]);
}

float CleanZero(const float value) noexcept
{
    return value == 0.0F ? 0.0F : value;
}

bool IsCanonicalName(const char name[GT_CONTACT_BONE_NAME_CAPACITY]) noexcept
{
    bool foundTerminator = false;
    for (std::size_t index = 0; index < GT_CONTACT_BONE_NAME_CAPACITY; ++index) {
        if (!foundTerminator) {
            if (name[index] == '\0') {
                if (index == 0U) {
                    return false;
                }
                foundTerminator = true;
            }
        } else if (name[index] != '\0') {
            return false;
        }
    }
    return foundTerminator;
}

bool ValidateEndpoint(const GTContactEndpointV1& endpoint) noexcept
{
    return endpoint.structSize == sizeof(GTContactEndpointV1) &&
           (endpoint.flags & ~GT_CONTACT_ENDPOINT_KNOWN_FLAGS) == 0U &&
           endpoint.skeletonToken != 0U && endpoint.rigidBodyToken != 0U &&
           endpoint.boneNameHash != 0U && IsCanonicalName(endpoint.boneName) &&
           IsZero(endpoint.reserved, std::size(endpoint.reserved));
}

int CompareEndpoint(const GTContactEndpointV1& lhs, const GTContactEndpointV1& rhs) noexcept
{
    if (lhs.skeletonToken != rhs.skeletonToken) {
        return lhs.skeletonToken < rhs.skeletonToken ? -1 : 1;
    }
    if (lhs.boneNameHash != rhs.boneNameHash) {
        return lhs.boneNameHash < rhs.boneNameHash ? -1 : 1;
    }
    const int nameResult = std::memcmp(lhs.boneName, rhs.boneName, GT_CONTACT_BONE_NAME_CAPACITY);
    if (nameResult != 0) {
        return nameResult < 0 ? -1 : 1;
    }
    if (lhs.rigidBodyToken != rhs.rigidBodyToken) {
        return lhs.rigidBodyToken < rhs.rigidBodyToken ? -1 : 1;
    }
    if (lhs.flags != rhs.flags) {
        return lhs.flags < rhs.flags ? -1 : 1;
    }
    return 0;
}

template <class T, std::size_t N>
int CompareArray(const std::array<T, N>& lhs, const std::array<T, N>& rhs) noexcept
{
    for (std::size_t index = 0; index < N; ++index) {
        if (lhs[index] != rhs[index]) {
            return lhs[index] < rhs[index] ? -1 : 1;
        }
    }
    return 0;
}

int CompareFloat(const float lhs, const float rhs) noexcept
{
    if (lhs == rhs) {
        return 0;
    }
    return lhs < rhs ? -1 : 1;
}

float ApproachSpeed(const GTContactSampleV1& sample) noexcept
{
    const float projected = sample.relativeVelocityAminusB[0] * sample.normalOnB[0] +
                            sample.relativeVelocityAminusB[1] * sample.normalOnB[1] +
                            sample.relativeVelocityAminusB[2] * sample.normalOnB[2];
    return std::max(0.0F, -projected);
}

GTContactStatusV1 ValidateFrameDestination(const GTContactFrameV1* frame) noexcept
{
    if (frame == nullptr) {
        return GT_CONTACT_STATUS_INVALID_ARGUMENT;
    }
    if (frame->structSize != sizeof(GTContactFrameV1) ||
        frame->sampleStructSize != sizeof(GTContactSampleV1)) {
        return GT_CONTACT_STATUS_STRUCT_SIZE_MISMATCH;
    }
    if (frame->abiVersion != GT_CONTACT_ABI_V1) {
        return GT_CONTACT_STATUS_INCOMPATIBLE_ABI;
    }
    if (frame->reserved0 != 0U || !IsZero(frame->reserved, std::size(frame->reserved))) {
        return GT_CONTACT_STATUS_RESERVED_NONZERO;
    }
    if (frame->targetGeneration == 0U) {
        return GT_CONTACT_STATUS_INVALID_ARGUMENT;
    }
    if ((frame->flags & ~GT_CONTACT_FRAME_KNOWN_FLAGS) != 0U) {
        return GT_CONTACT_STATUS_INVALID_ARGUMENT;
    }
    return GT_CONTACT_STATUS_OK;
}

}  // namespace

GTContactStatusV1 ValidateBridgeDescriptorV1(
    const GTContactBridgeV1* bridge,
    const std::uint32_t callerBridgeStructSize) noexcept
{
    if (bridge == nullptr) {
        return GT_CONTACT_STATUS_INVALID_ARGUMENT;
    }
    if (callerBridgeStructSize != sizeof(GTContactBridgeV1) ||
        bridge->structSize != sizeof(GTContactBridgeV1) ||
        bridge->frameStructSize != sizeof(GTContactFrameV1) ||
        bridge->sampleStructSize != sizeof(GTContactSampleV1) ||
        bridge->endpointStructSize != sizeof(GTContactEndpointV1)) {
        return GT_CONTACT_STATUS_STRUCT_SIZE_MISMATCH;
    }
    if (bridge->abiVersion != GT_CONTACT_ABI_V1) {
        return GT_CONTACT_STATUS_INCOMPATIBLE_ABI;
    }
    if (bridge->flags != 0U) {
        return GT_CONTACT_STATUS_INVALID_ARGUMENT;
    }
    if (!IsZero(bridge->reserved, std::size(bridge->reserved))) {
        return GT_CONTACT_STATUS_RESERVED_NONZERO;
    }
    if (bridge->copyLatestFrame == nullptr || bridge->setCaptureEnabled == nullptr ||
        bridge->setTargetSkeletons == nullptr) {
        return GT_CONTACT_STATUS_INVALID_ARGUMENT;
    }
    return GT_CONTACT_STATUS_OK;
}

GTContactStatusV1 AdvanceTargetGeneration(
    const std::uint64_t currentGeneration,
    std::uint64_t* outNextGeneration) noexcept
{
    if (outNextGeneration == nullptr) {
        return GT_CONTACT_STATUS_INVALID_ARGUMENT;
    }
    *outNextGeneration = 0U;
    if (currentGeneration == std::numeric_limits<std::uint64_t>::max()) {
        return GT_CONTACT_STATUS_GENERATION_EXHAUSTED;
    }
    *outNextGeneration = currentGeneration + 1U;
    return GT_CONTACT_STATUS_OK;
}

bool ContactFrameCollector::BeginFrame(
    const std::uint64_t frameSequence,
    const float frameTimeStep,
    const std::array<float, 3>& originOffset,
    const std::uint32_t substepCount,
    const std::uint64_t targetGeneration) noexcept
{
    std::scoped_lock lock(mutex_);
    if (collecting_ || frameSequence == 0U || !std::isfinite(frameTimeStep) || frameTimeStep < 0.0F ||
        !std::isfinite(originOffset[0]) || !std::isfinite(originOffset[1]) ||
        !std::isfinite(originOffset[2]) || targetGeneration == 0U ||
        (hasPublishedFrame_ && frameSequence <= publishedFrame_.frameSequence)) {
        return false;
    }

    buildingFrame_ = {};
    buildingFrame_.structSize = sizeof(GTContactFrameV1);
    buildingFrame_.abiVersion = GT_CONTACT_ABI_V1;
    buildingFrame_.sampleStructSize = sizeof(GTContactSampleV1);
    buildingFrame_.frameSequence = frameSequence;
    buildingFrame_.frameTimeStep = CleanZero(frameTimeStep);
    for (std::size_t axis = 0; axis < originOffset.size(); ++axis) {
        buildingFrame_.originOffset[axis] = CleanZero(originOffset[axis]);
    }
    buildingFrame_.substepCount = substepCount;
    buildingFrame_.targetGeneration = targetGeneration;
    candidateCount_ = 0U;
    collecting_ = true;
    return true;
}

AddResult ContactFrameCollector::Add(const GTContactSampleV1& source) noexcept
{
    std::scoped_lock lock(mutex_);
    if (!collecting_) {
        return AddResult::noOpenFrame;
    }

    ++buildingFrame_.totalObservedSampleCount;
    if (!ValidateSample(source) || source.substepIndex >= buildingFrame_.substepCount) {
        ++buildingFrame_.invalidSampleCount;
        return AddResult::invalidSample;
    }

    ++buildingFrame_.validObservedSampleCount;
    Candidate incoming{};
    incoming.sample = source;
    incoming.sample.frameSequence = buildingFrame_.frameSequence;
    Canonicalize(incoming.sample);
    incoming.key = MakeKey(incoming.sample);

    for (std::size_t index = 0; index < candidateCount_; ++index) {
        if (SameContact(candidates_[index], incoming)) {
            ++buildingFrame_.duplicateSampleCount;
            if (BetterForRetention(incoming, candidates_[index])) {
                candidates_[index] = incoming;
            }
            return AddResult::duplicateMerged;
        }
    }

    if (candidateCount_ < kObservationCapacity) {
        candidates_[candidateCount_++] = incoming;
        return AddResult::accepted;
    }

    ++buildingFrame_.inputOverflowSampleCount;
    std::size_t worst = 0U;
    for (std::size_t index = 1; index < candidateCount_; ++index) {
        if (BetterForRetention(candidates_[worst], candidates_[index])) {
            worst = index;
        }
    }
    if (BetterForRetention(incoming, candidates_[worst])) {
        candidates_[worst] = incoming;
        return AddResult::capacityRetained;
    }
    return AddResult::capacityDropped;
}

bool ContactFrameCollector::EndFrame() noexcept
{
    std::scoped_lock lock(mutex_);
    if (!collecting_) {
        return false;
    }

    std::sort(
        candidates_.begin(),
        candidates_.begin() + static_cast<std::ptrdiff_t>(candidateCount_),
        BetterForRetention);

    const std::size_t publishCount = std::min(candidateCount_, kPublishedCapacity);
    for (std::size_t index = 0; index < publishCount; ++index) {
        publishedSamples_[index] = candidates_[index].sample;
    }
    std::sort(
        publishedSamples_.begin(),
        publishedSamples_.begin() + static_cast<std::ptrdiff_t>(publishCount),
        [](const GTContactSampleV1& lhs, const GTContactSampleV1& rhs) {
            Candidate left{};
            left.sample = lhs;
            left.key = MakeKey(lhs);
            Candidate right{};
            right.sample = rhs;
            right.key = MakeKey(rhs);
            return CanonicalLess(left, right);
        });

    buildingFrame_.retainedCandidateCount = static_cast<std::uint32_t>(candidateCount_);
    buildingFrame_.availableSampleCount = static_cast<std::uint32_t>(publishCount);
    buildingFrame_.publishedSampleCount = static_cast<std::uint32_t>(publishCount);
    buildingFrame_.droppedSampleCount =
        buildingFrame_.totalObservedSampleCount - buildingFrame_.publishedSampleCount;
    if (buildingFrame_.duplicateSampleCount != 0U) {
        buildingFrame_.flags |= GT_CONTACT_FRAME_DEDUPLICATED;
    }
    if (buildingFrame_.invalidSampleCount != 0U) {
        buildingFrame_.flags |= GT_CONTACT_FRAME_INVALID_INPUT_DROPPED;
    }
    if (buildingFrame_.inputOverflowSampleCount != 0U) {
        buildingFrame_.flags |= GT_CONTACT_FRAME_INPUT_CAP_REACHED;
    }
    if (candidateCount_ > kPublishedCapacity) {
        buildingFrame_.flags |= GT_CONTACT_FRAME_PUBLISH_CAP_REACHED;
    }

    publishedFrame_ = buildingFrame_;
    hasPublishedFrame_ = true;
    collecting_ = false;
    return true;
}

GTContactStatusV1 ContactFrameCollector::CopyLatest(
    GTContactFrameV1* outFrame,
    GTContactSampleV1* outSamples,
    const std::uint32_t sampleCapacity,
    std::uint32_t* outCopiedSampleCount,
    const std::uint64_t currentTargetGeneration) const noexcept
{
    if (outCopiedSampleCount == nullptr || (sampleCapacity != 0U && outSamples == nullptr)) {
        return GT_CONTACT_STATUS_INVALID_ARGUMENT;
    }
    *outCopiedSampleCount = 0U;
    const GTContactStatusV1 destinationStatus = ValidateFrameDestination(outFrame);
    if (destinationStatus != GT_CONTACT_STATUS_OK) {
        return destinationStatus;
    }

    const std::uint64_t expectedTargetGeneration = outFrame->targetGeneration;
    if (currentTargetGeneration == 0U) {
        return GT_CONTACT_STATUS_INVALID_ARGUMENT;
    }
    if (expectedTargetGeneration != currentTargetGeneration) {
        return GT_CONTACT_STATUS_TARGET_GENERATION_MISMATCH;
    }
    std::scoped_lock lock(mutex_);
    if (!hasPublishedFrame_ ||
        publishedFrame_.targetGeneration != expectedTargetGeneration) {
        // The caller owns the current target epoch, but a frame for that epoch
        // has not been published yet.  This is a normal transition, not an
        // ownership mismatch that should trigger another target replacement.
        return GT_CONTACT_STATUS_NO_FRAME;
    }

    const std::uint32_t copyCount = std::min(sampleCapacity, publishedFrame_.publishedSampleCount);
    if (copyCount != 0U) {
        std::copy_n(publishedSamples_.begin(), copyCount, outSamples);
    }
    *outCopiedSampleCount = copyCount;
    *outFrame = publishedFrame_;
    return copyCount == publishedFrame_.publishedSampleCount ? GT_CONTACT_STATUS_OK
                                                             : GT_CONTACT_STATUS_BUFFER_TOO_SMALL;
}

std::uint64_t ContactFrameCollector::HashBoneName(const char* name) noexcept
{
    if (name == nullptr) {
        return 0U;
    }
    std::uint64_t hash = UINT64_C(14695981039346656037);
    while (*name != '\0') {
        hash ^= static_cast<std::uint8_t>(*name++);
        hash *= UINT64_C(1099511628211);
    }
    return hash;
}

bool ContactFrameCollector::ValidateSample(const GTContactSampleV1& sample) noexcept
{
    if (sample.structSize != sizeof(GTContactSampleV1) ||
        (sample.flags & ~GT_CONTACT_SAMPLE_KNOWN_FLAGS) != 0U || sample.reserved0 != 0U ||
        sample.reservedScalar != 0.0F || !IsZero(sample.reserved, std::size(sample.reserved)) ||
        !ValidateEndpoint(sample.endpointA) || !ValidateEndpoint(sample.endpointB) ||
        !std::isfinite(sample.timeStep) || sample.timeStep < 0.0F ||
        !std::isfinite(sample.penetrationDepth) || sample.penetrationDepth < 0.0F ||
        !std::isfinite(sample.normalImpulse) || sample.normalImpulse < 0.0F ||
        !IsFinite3(sample.pointOnA) || !IsFinite3(sample.pointOnB) || !IsFinite3(sample.normalOnB) ||
        !IsFinite3(sample.relativeVelocityAminusB)) {
        return false;
    }
    const float lengthSquared = sample.normalOnB[0] * sample.normalOnB[0] +
                                sample.normalOnB[1] * sample.normalOnB[1] +
                                sample.normalOnB[2] * sample.normalOnB[2];
    return std::isfinite(lengthSquared) && lengthSquared > std::numeric_limits<float>::epsilon();
}

void ContactFrameCollector::Canonicalize(GTContactSampleV1& sample) noexcept
{
    for (float& value : sample.pointOnA) {
        value = CleanZero(value);
    }
    for (float& value : sample.pointOnB) {
        value = CleanZero(value);
    }
    for (float& value : sample.relativeVelocityAminusB) {
        value = CleanZero(value);
    }
    sample.timeStep = CleanZero(sample.timeStep);
    sample.penetrationDepth = CleanZero(sample.penetrationDepth);
    sample.normalImpulse = CleanZero(sample.normalImpulse);

    const float length = std::sqrt(sample.normalOnB[0] * sample.normalOnB[0] +
                                   sample.normalOnB[1] * sample.normalOnB[1] +
                                   sample.normalOnB[2] * sample.normalOnB[2]);
    for (float& value : sample.normalOnB) {
        value = CleanZero(value / length);
    }

    if (CompareEndpoint(sample.endpointB, sample.endpointA) < 0) {
        std::swap(sample.endpointA, sample.endpointB);
        for (std::size_t axis = 0; axis < 3U; ++axis) {
            std::swap(sample.pointOnA[axis], sample.pointOnB[axis]);
            sample.normalOnB[axis] = CleanZero(-sample.normalOnB[axis]);
            sample.relativeVelocityAminusB[axis] = CleanZero(-sample.relativeVelocityAminusB[axis]);
        }
    }
    sample.flags |= GT_CONTACT_SAMPLE_CANONICAL_ENDPOINT_ORDER;
}

ContactFrameCollector::ContactKey ContactFrameCollector::MakeKey(
    const GTContactSampleV1& sample) noexcept
{
    ContactKey key{};
    for (std::size_t axis = 0; axis < 3U; ++axis) {
        key.pointBins[axis] = static_cast<std::int64_t>(
            std::llround(static_cast<double>(sample.pointOnA[axis]) * kPointQuantization));
        key.pointBins[axis + 3U] = static_cast<std::int64_t>(
            std::llround(static_cast<double>(sample.pointOnB[axis]) * kPointQuantization));
        key.normalBins[axis] = static_cast<std::int32_t>(
            std::lround(sample.normalOnB[axis] * kNormalQuantization));
    }
    return key;
}

bool ContactFrameCollector::SameContact(const Candidate& lhs, const Candidate& rhs) noexcept
{
    return CompareEndpoint(lhs.sample.endpointA, rhs.sample.endpointA) == 0 &&
           CompareEndpoint(lhs.sample.endpointB, rhs.sample.endpointB) == 0 &&
           lhs.key.pointBins == rhs.key.pointBins && lhs.key.normalBins == rhs.key.normalBins;
}

bool ContactFrameCollector::CanonicalLess(const Candidate& lhs, const Candidate& rhs) noexcept
{
    const int endpointA = CompareEndpoint(lhs.sample.endpointA, rhs.sample.endpointA);
    if (endpointA != 0) {
        return endpointA < 0;
    }
    const int endpointB = CompareEndpoint(lhs.sample.endpointB, rhs.sample.endpointB);
    if (endpointB != 0) {
        return endpointB < 0;
    }
    const int points = CompareArray(lhs.key.pointBins, rhs.key.pointBins);
    if (points != 0) {
        return points < 0;
    }
    const int normal = CompareArray(lhs.key.normalBins, rhs.key.normalBins);
    if (normal != 0) {
        return normal < 0;
    }
    if (lhs.sample.substepIndex != rhs.sample.substepIndex) {
        return lhs.sample.substepIndex < rhs.sample.substepIndex;
    }
    const int depth = CompareFloat(lhs.sample.penetrationDepth, rhs.sample.penetrationDepth);
    if (depth != 0) {
        return depth < 0;
    }
    const int impulse = CompareFloat(lhs.sample.normalImpulse, rhs.sample.normalImpulse);
    if (impulse != 0) {
        return impulse < 0;
    }
    const int approach = CompareFloat(ApproachSpeed(lhs.sample), ApproachSpeed(rhs.sample));
    if (approach != 0) {
        return approach < 0;
    }
    return lhs.sample.flags < rhs.sample.flags;
}

bool ContactFrameCollector::BetterForRetention(const Candidate& lhs, const Candidate& rhs) noexcept
{
    if (lhs.sample.penetrationDepth != rhs.sample.penetrationDepth) {
        return lhs.sample.penetrationDepth > rhs.sample.penetrationDepth;
    }
    if (lhs.sample.normalImpulse != rhs.sample.normalImpulse) {
        return lhs.sample.normalImpulse > rhs.sample.normalImpulse;
    }
    const float lhsApproach = ApproachSpeed(lhs.sample);
    const float rhsApproach = ApproachSpeed(rhs.sample);
    if (lhsApproach != rhsApproach) {
        return lhsApproach > rhsApproach;
    }
    return CanonicalLess(lhs, rhs);
}

}  // namespace gt::contact
