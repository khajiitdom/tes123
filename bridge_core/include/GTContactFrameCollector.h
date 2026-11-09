#ifndef GT_CONTACT_FRAME_COLLECTOR_H
#define GT_CONTACT_FRAME_COLLECTOR_H

#include "GTContactBridgeABI.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <mutex>

namespace gt::contact {

enum class AddResult : std::uint8_t {
    accepted,
    duplicateMerged,
    invalidSample,
    noOpenFrame,
    capacityRetained,
    capacityDropped,
};

GTContactStatusV1 ValidateBridgeDescriptorV1(
    const GTContactBridgeV1* bridge,
    std::uint32_t callerBridgeStructSize) noexcept;

GTContactStatusV1 AdvanceTargetGeneration(
    std::uint64_t currentGeneration,
    std::uint64_t* outNextGeneration) noexcept;

class ContactFrameCollector final {
public:
    static constexpr std::size_t kObservationCapacity = 2048;
    static constexpr std::size_t kPublishedCapacity = 512;

    ContactFrameCollector() noexcept = default;
    ContactFrameCollector(const ContactFrameCollector&) = delete;
    ContactFrameCollector& operator=(const ContactFrameCollector&) = delete;

    bool BeginFrame(
        std::uint64_t frameSequence,
        float frameTimeStep,
        const std::array<float, 3>& originOffset,
        std::uint32_t substepCount,
        std::uint64_t targetGeneration) noexcept;

    AddResult Add(const GTContactSampleV1& sample) noexcept;
    bool EndFrame() noexcept;

    GTContactStatusV1 CopyLatest(
        GTContactFrameV1* outFrame,
        GTContactSampleV1* outSamples,
        std::uint32_t sampleCapacity,
        std::uint32_t* outCopiedSampleCount,
        std::uint64_t currentTargetGeneration) const noexcept;

    static std::uint64_t HashBoneName(const char* name) noexcept;

private:
    struct ContactKey {
        std::array<std::int64_t, 6> pointBins{};
        std::array<std::int32_t, 3> normalBins{};
    };

    struct Candidate {
        GTContactSampleV1 sample{};
        ContactKey key{};
    };

    static bool ValidateSample(const GTContactSampleV1& sample) noexcept;
    static void Canonicalize(GTContactSampleV1& sample) noexcept;
    static ContactKey MakeKey(const GTContactSampleV1& sample) noexcept;
    static bool SameContact(const Candidate& lhs, const Candidate& rhs) noexcept;
    static bool CanonicalLess(const Candidate& lhs, const Candidate& rhs) noexcept;
    static bool BetterForRetention(const Candidate& lhs, const Candidate& rhs) noexcept;

    mutable std::mutex mutex_;
    std::array<Candidate, kObservationCapacity> candidates_{};
    std::array<GTContactSampleV1, kPublishedCapacity> publishedSamples_{};
    GTContactFrameV1 buildingFrame_{};
    GTContactFrameV1 publishedFrame_{};
    std::size_t candidateCount_{0};
    bool collecting_{false};
    bool hasPublishedFrame_{false};
};

}  // namespace gt::contact

#endif
