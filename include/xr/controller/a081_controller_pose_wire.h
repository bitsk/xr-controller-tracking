// SPDX-License-Identifier: BSL-1.0
#pragma once
#include <stddef.h>
#include <stdint.h>

/* Little-endian, IEEE754 binary32, no pointers or native libc++ objects.
 * One SOCK_SEQPACKET message is exactly one hand measurement. This contract
 * does not turn SDK confidence into proof of optical position tracking.
 */
#define A081_CONTROLLER_POSE_MAGIC UINT32_C(0x43503841)
#define A081_CONTROLLER_POSE_VERSION 1u
#define A081_CONTROLLER_POSE_MAX_AGE_NS UINT64_C(100000000)
#define A081_CONTROLLER_CONNECTED (1u << 0)
#define A081_CONTROLLER_ORIENTATION_VALID (1u << 1)
#define A081_CONTROLLER_POSITION_VALID (1u << 2)
#define A081_CONTROLLER_CLOCK_VALID (1u << 3)
#define A081_CONTROLLER_SDK_POSE (1u << 4)
/* The pinned SDK reports confidence1 for its recent visual update window.
 * This diagnostic is optional for an initialized SDK's valid predicted pose;
 * its confidence0.1 floor and submitted camera frames are not optical proof.
 */
#define A081_CONTROLLER_OPTICAL_VALID (1u << 5)
#define A081_CONTROLLER_POSE_REQUIRED_FLAGS (A081_CONTROLLER_CONNECTED | \
    A081_CONTROLLER_ORIENTATION_VALID | A081_CONTROLLER_POSITION_VALID | \
    A081_CONTROLLER_CLOCK_VALID | A081_CONTROLLER_SDK_POSE)
#define A081_CONTROLLER_POSE_KNOWN_FLAGS 63u
#define A081_CONTROLLER_PROFILE_UNVERIFIED 0u
/* Meters and xyzw orientation already transformed to the SAME raw OpenVR
 * origin/basis used by the running QVR HMD, with a verified SDK transform.
 * This is an output contract, not a guessed SDK coordinate conversion.
 */
#define A081_CONTROLLER_PROFILE_OPENVR_HEAD_RAW 1u

struct a081_controller_pose_packet {
    uint32_t magic, version, bytes, flags;
    uint64_t session_id, sequence, sample_monotonic_ns, send_monotonic_ns;
    uint32_t hand; /* 0 left, 1 right */
    uint32_t coordinate_profile;
    float confidence;
    uint32_t reserved0;
    float position[3];
    float orientation_xyzw[4];
    uint32_t reserved1;
};
#ifdef __cplusplus
static_assert(sizeof(a081_controller_pose_packet) == 96, "controller pose wire size");
static_assert(offsetof(a081_controller_pose_packet, position) == 64, "controller pose offsets");
#else
_Static_assert(sizeof(struct a081_controller_pose_packet) == 96, "controller pose wire size");
_Static_assert(offsetof(struct a081_controller_pose_packet, position) == 64, "controller pose offsets");
#endif
