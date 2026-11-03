#ifndef A081_CONTROLLER_CONTROLS_H
#define A081_CONTROLLER_CONTROLS_H

#include <xr/device/a081_controller_wire.h>

/* Input fields from the original serial callback registrations, before JNI
 * hand-specific button/touch/axis remapping. See controls-contract.json.
 * These are vendor values, not OpenXR paths or a pose/clock-validity flag.
 */
enum a081_controller_control_fields {
    A081_CONTROL_BUTTONS = 1u << 0,
    A081_CONTROL_TOUCH = 1u << 1,
    A081_CONTROL_INDEX_FINGER = 1u << 2,
    A081_CONTROL_MIDDLE_FINGER = 1u << 3,
    A081_CONTROL_JOYSTICK = 1u << 4,
    A081_CONTROL_MODE1 = A081_CONTROL_BUTTONS | A081_CONTROL_TOUCH |
                         A081_CONTROL_INDEX_FINGER | A081_CONTROL_MIDDLE_FINGER
};

struct a081_controller_controls {
    uint8_t pattern_known, head_version[3];
    uint8_t head_power_state, gzll_raw_state, gzll_state_known;
    uint8_t stock_enable;
    uint8_t buttons_raw, touch_raw, index_finger_raw, middle_finger_raw;
    int16_t joystick_raw[2];
    uint32_t seen_mask;
    uint64_t mode1_receipt_monotonic_ns, joystick_receipt_monotonic_ns;
    uint64_t mode1_sequence, joystick_sequence;
};

static inline void
a081_controller_controls_release(struct a081_controller_controls *s)
{
    s->seen_mask = 0;
    s->mode1_receipt_monotonic_ns = s->joystick_receipt_monotonic_ns = 0;
    s->mode1_sequence = s->joystick_sequence = 0;
    s->buttons_raw = s->touch_raw = s->index_finger_raw = s->middle_finger_raw = 0;
    s->joystick_raw[0] = s->joystick_raw[1] = 0;
}

/* packet must already have passed META decoding, strict header and sum8.
 * Receipt time is from the real MONOTONIC transport, never converted to an
 * invented radio event time. Each multiplexed field keeps its own receipt.
 */
static inline uint32_t
a081_controller_controls_update(struct a081_controller_controls *s,
                                const struct a081_controller_wire_packet *packet,
                                uint64_t receipt_monotonic_ns, uint64_t sequence)
{
    const uint8_t *p = packet->payload;
    if (packet->type == 2) {
        uint8_t version[3] = {p[24], p[23], p[22]};
        uint8_t power = version[2] >= 25 ? p[25] & 15 : p[25];
        uint8_t enable = power == 3; /* Original head_pwr callback, not GZLL. */
        if (!s->pattern_known || memcmp(s->head_version, version, sizeof(version)) ||
            s->stock_enable != enable || !enable)
            a081_controller_controls_release(s);
        memcpy(s->head_version, version, sizeof(version));
        s->pattern_known = 1;
        s->head_power_state = power;
        s->gzll_state_known = version[2] >= 25;
        s->gzll_raw_state = s->gzll_state_known ? p[25] >> 4 : 0;
        s->stock_enable = enable;
        return 0;
    }
    if (packet->type != 7 || !receipt_monotonic_ns || !sequence)
        return 0;

    uint32_t updated = 0;
    switch (p[2] & 15) {
    case 1:
        s->buttons_raw = p[27] & 0xf0;
        s->touch_raw = p[27] & 15;
        s->index_finger_raw = p[28] & 15;
        s->middle_finger_raw = p[28] >> 4;
        s->mode1_receipt_monotonic_ns = receipt_monotonic_ns;
        s->mode1_sequence = sequence;
        updated = A081_CONTROL_MODE1;
        break;
    case 2:
        s->joystick_raw[0] = p[28] < 128 ? p[28] : (int)p[28] - 256;
        s->joystick_raw[1] = p[27] < 128 ? p[27] : (int)p[27] - 256;
        s->joystick_receipt_monotonic_ns = receipt_monotonic_ns;
        s->joystick_sequence = sequence;
        updated = A081_CONTROL_JOYSTICK;
        break;
    default:
        break; /* Other modes must not refresh earlier control values. */
    }
    s->seen_mask |= updated;
    return updated;
}

static inline int
a081_controller_control_fresh(uint64_t receipt, uint64_t now, uint64_t age_limit_ns)
{
    return receipt && age_limit_ns && now >= receipt && now - receipt < age_limit_ns;
}

/* Call this again at consumption time, even if no further packets arrive.
 * age_limit_ns is a consumer safety policy, not a vendor wireless timeout.
 * Zero/unknown enable, expired samples, and source EOF require release. A
 * transport being present or a GZLL raw value alone is not active input.
 */
static inline uint32_t
a081_controller_controls_valid_mask(const struct a081_controller_controls *s,
                                    uint64_t now_monotonic_ns, uint64_t age_limit_ns)
{
    if (!s->pattern_known || !s->stock_enable)
        return 0;
    uint32_t valid = 0;
    if (a081_controller_control_fresh(s->mode1_receipt_monotonic_ns,
                                     now_monotonic_ns, age_limit_ns))
        valid |= s->seen_mask & A081_CONTROL_MODE1;
    if (a081_controller_control_fresh(s->joystick_receipt_monotonic_ns,
                                     now_monotonic_ns, age_limit_ns))
        valid |= s->seen_mask & A081_CONTROL_JOYSTICK;
    return valid;
}

#endif
