// SPDX-License-Identifier: BSL-1.0
#pragma once
#include <xr/controller/a081_controller_controls.h>
#include <xr/controller/production-v2/a081_controller_pose_wire.h>

/* Publish receipt-aged radio controls independently of SDK pose validity.
 * Recompute the multiplexed field mask for every packet: producing new poses
 * must never refresh an old held button or joystick value.
 */
static inline void
a081_controller_pose_set_controls(struct a081_controller_pose_packet *packet,
                                  const struct a081_controller_controls *controls,
                                  uint64_t now_monotonic_ns)
{
    packet->flags &= ~A081_CONTROLLER_CONTROLS_VALID;
    packet->controls = packet->joystick = 0;
    if (!(packet->flags & A081_CONTROLLER_CONNECTED))
        return;
    const uint32_t valid = a081_controller_controls_valid_mask(
        controls, now_monotonic_ns, A081_CONTROLLER_POSE_MAX_AGE_NS);
    if (!valid)
        return;
    packet->flags |= A081_CONTROLLER_CONTROLS_VALID;
    packet->controls = valid << 24;
    if ((valid & A081_CONTROL_MODE1) == A081_CONTROL_MODE1) {
        packet->controls |= (controls->buttons_raw & 0xf0u) |
                            ((controls->touch_raw & 15u) << 8) |
                            ((controls->index_finger_raw & 15u) << 12) |
                            ((controls->middle_finger_raw & 15u) << 16);
    }
    if (valid & A081_CONTROL_JOYSTICK)
        packet->joystick = (uint8_t)controls->joystick_raw[0] |
                           ((uint32_t)(uint8_t)controls->joystick_raw[1] << 8);
}
