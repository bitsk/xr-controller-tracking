#ifndef A081_CONTROLLER_IMAGE_ABI_HPP
#define A081_CONTROLLER_IMAGE_ABI_HPP

#include <cstdint>
#include <functional>
#include <vector>

#if !defined(__ANDROID__) || !defined(__arm__)
#error "This is the Android ARM32 libc++ ABI, not a host/glibc interface."
#endif

namespace a081_controller {

using BufferVector = std::vector<void *>;
using ReleaseFrame = std::function<void(int, BufferVector, BufferVector)>;

static_assert(sizeof(BufferVector) == 12, "Pinned ARM32 vector ABI");
static_assert(sizeof(ReleaseFrame) == 24, "Pinned ARM32 libc++ function ABI");

// Recovered from the setIMGRawData lambda RTTI plus callsites/stack loads.
// This is a signature reference, not an image adapter ready to run:
// - Each pair of vector roles (mapped memory vs ION/release metadata) still
//   needs the QVR map routine decoded; pointers cannot be sent across IPC.
// - geometry_a/b/c, layout and stream ordering need actual camera mapping.
// - All vector/function parameters are BY VALUE (nontrivial indirect AAPCS).
//   Do not replace them with vector* or with libc++ from a glibc process.
// - Each frame's backing buffers must outlive the asynchronous release callback.
// - Original JNI submits one wrapper for each hand (+0x48 and +0x60); the common
//   method at +0x1c takes the hand explicitly.
using PushImages = int (*)(
    void *self, BufferVector buffers_a, BufferVector metadata_a,
    unsigned geometry_a, unsigned geometry_b, unsigned geometry_c,
    unsigned long long timestamp_ns, int frame_id_a, int hand,
    unsigned char layout, ReleaseFrame release_a,
    BufferVector buffers_b, BufferVector metadata_b,
    unsigned frame_id_b, ReleaseFrame release_b);

} // namespace a081_controller

#endif
