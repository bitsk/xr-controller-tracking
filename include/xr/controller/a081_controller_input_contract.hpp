#ifndef A081_CONTROLLER_INPUT_CONTRACT_HPP
#define A081_CONTROLLER_INPUT_CONTRACT_HPP

#include <xr/device/a081_controller_abi.h>
#include <xr/controller/a081_controller_image_abi.hpp>
#include <xr/device/a081_controller_reference.hpp>

#include <array>
#include <cstdint>
#include <type_traits>

// Field packing recovered from this A081 build, not a running sensor adapter.
// No device access, SDK calls, camera synthesis or time-sync negotiation here.
// See input-contract.md for callsites, SHA boundaries and missing prerequisites.
namespace a081_controller::input_contract {

using PushImagesConfirmed = int (*)(
    void *self, BufferVector tracking_pixels, BufferVector tracking_release_tokens,
    unsigned view_width, unsigned view_height, unsigned stride_bytes,
    unsigned long long ctrl_sof_boottime_ns, int tracking_frame_id, int hand,
    unsigned char channels, ReleaseFrame release_tracking,
    BufferVector ctrl_pixels, BufferVector ctrl_release_tokens,
    unsigned ctrl_frame_id, ReleaseFrame release_ctrl);

static_assert(std::is_same_v<PushImagesConfirmed, PushImages>);

inline constexpr unsigned char mono8_channels = 1;

struct ImageGeometry {
    unsigned view_width;
    unsigned view_height;
    unsigned stride_bytes;
};

// JNI does not read frame.stride or AHardwareBuffer_Desc.stride here. A caller
// must verify/repack the real buffers to this layout before using these values.
constexpr ImageGeometry stock_geometry(unsigned ctrl_frame_width,
                                      unsigned ctrl_frame_height) noexcept
{
    const unsigned width = ctrl_frame_width >> 1;
    return {width, ctrl_frame_height, width << 1};
}

// Frame PHASE 0 selects HAND 1; these are deliberately different numbers.
// A nonnegative result does not establish the original state==5/enable gates.
constexpr int hand_for_ctrl_frame(unsigned frame_id) noexcept
{
    return frame_id % 3 == 0 ? A081_HAND_RIGHT :
           frame_id % 3 == 1 ? A081_HAND_LEFT : -1;
}

enum class ClockEncoding { legacy_30500ns, microseconds };

// The third byte of the Pattern ID version array is payload[22].
// It is NOT the separate SDK FusionImpl algorithm selector (default 20).
constexpr ClockEncoding clock_encoding(std::uint8_t pattern_version_third) noexcept
{
    return pattern_version_third < 25 ? ClockEncoding::legacy_30500ns
                                    : ClockEncoding::microseconds;
}

// This only reproduces ParseImuPreIntPackageImpl's scale. A clock value is
// aligned to Linux BOOTTIME only after the original synchronization protocol.
// Do not use receive time as the timestamp or add an unobserved affine fit here.
constexpr std::uint64_t scale_type7_clock(std::uint64_t wire_clock_48,
                                        ClockEncoding encoding) noexcept
{
    return wire_clock_48 * (encoding == ClockEncoding::legacy_30500ns
                               ? UINT64_C(30500) : UINT64_C(1000));
}

inline a081_controller_imu_input pack_type7_fields(
    const a081::controller::PreintegratedPacket &packet,
    ClockEncoding encoding) noexcept
{
    a081_controller_imu_input result{};
    result.values[0] = packet.orientation.w;
    result.values[1] = packet.orientation.x;
    result.values[2] = packet.orientation.y;
    result.values[3] = packet.orientation.z;
    for (unsigned i = 0; i < 3; ++i) {
        result.values[i + 4] = packet.vector[i];
        result.sideband[i] = packet.sideband[i];
    }
    result.aligned_time_ns = scale_type7_clock(packet.clock_value_48, encoding);
    return result;
}

// Bytes constructed by SerialPortIOImpl::WriteTimeStampCmd BEFORE transport.
// This is a 12-byte command body, not the 36-byte SPI receive transfer format.
// caller passes a confirmed LEFT/RIGHT hand and real ctrl-tracking SOF time.
// The floating conversions reproduce the observed JNI/serial sequence.
inline std::array<std::uint8_t, 12> timestamp_command(
    a081_controller_hand hand, std::uint64_t ctrl_sof_boottime_ns,
    ClockEncoding encoding) noexcept
{
    auto value = static_cast<std::uint64_t>(
        static_cast<double>(ctrl_sof_boottime_ns) * 0.001);
    if (encoding == ClockEncoding::legacy_30500ns)
        value = static_cast<std::uint64_t>(static_cast<double>(value) / 30.5);

    std::array<std::uint8_t, 12> bytes{};
    bytes[0] = 0xa5;
    bytes[1] = 12;
    bytes[2] = hand == A081_HAND_RIGHT ? 0x12 : 0x11;
    unsigned checksum = bytes[1] + bytes[2];
    for (unsigned i = 0; i < 8; ++i) {
        bytes[i + 3] = static_cast<std::uint8_t>(value >> (i * 8));
        checksum += bytes[i + 3];
    }
    bytes[11] = static_cast<std::uint8_t>(checksum);
    return bytes;
}

} // namespace a081_controller::input_contract

#endif
