#define _GNU_SOURCE
#include <xr/device/a081_controller_wire.h>
#include <xr/controller/a081_controller_controls.h>

#ifndef A081_CONTROLLER_CONTROLS_ONLY
#include <dlfcn.h>
#include <link.h>
#include <math.h>
#endif
#include <errno.h>
#include <inttypes.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#if defined(A081_CONTROLLER_CONTROLS_ONLY)
#if !defined(__linux__) || !defined(__aarch64__)
#error "Build the native controls-only adapter for Linux ARM64."
#endif
#else
#if !defined(__ANDROID__) || !defined(__arm__)
#error "Build for Android ARM32; run only on the actual compatible A081."
#endif
_Static_assert(sizeof(void *) == 4, "The stock SDK requires ARM32 pointers");
#endif
_Static_assert(sizeof(double) == 8, "The stock SDK requires binary64 arrays");

/* This is an input adapter, not an alternative tracking algorithm. It reads
 * only real nordic-stream records. No sample generation or radio TX lives here.
 * This build's original parser passes the scaled type7 timestamp directly to
 * the SDK. Its TimeCalibrator::Avaliable/LR::Available return constant 1; an
 * external "ready" assertion would not reproduce an actual stock handshake.
 * Check the real BOOTTIME receive window for every sample instead. See
 * radio-clock-contract.json. These gates do not establish optical tracking.
 */
#define MAX_LINE 4096
#define MAX_SAMPLE_AGE_NS UINT64_C(33000000)
#define MAX_ANCHOR_UNCERTAINTY_NS UINT64_C(100000)
#define MAX_ANCHOR_CHANGE_NS INT64_C(1000000)
/* A local control freshness policy, not a reconstructed vendor disconnect
 * timeout. Consumers must also enforce this if the stream stops producing. */
#define CONTROL_MAX_AGE_NS UINT64_C(100000000)

#ifndef A081_CONTROLLER_CONTROLS_ONLY
struct sdk {
    void *object;
    a081_controller_push_imu_fn imu;
    a081_controller_get_pose_fn pose;
};
#endif

struct record {
    uint64_t irq_ns, completion_ns, sequence;
    uint8_t raw[36], raw72[72];
    uint64_t offset;
};

#ifndef A081_CONTROLLER_CONTROLS_ONLY
struct anchor {
    uint64_t monotonic_ns, boottime_ns, uncertainty_ns;
    int64_t offset_ns;
};
#endif

struct hand_state {
    int version_known;
    uint8_t version[3];
    uint64_t last_time_ns;
    struct a081_controller_controls controls;
    uint32_t emitted_controls_valid;
};

static void quoted(const char *s)
{
    putchar('"');
    for (const unsigned char *p = (const unsigned char *)s; *p; ++p) {
        if (*p == '"' || *p == '\\') {
            putchar('\\');
            putchar(*p);
        } else if (*p < 0x20) {
            printf("\\u%04x", *p);
        } else {
            putchar(*p);
        }
    }
    putchar('"');
}

static void event(const char *stage, const char *status, const char *detail)
{
    printf("{\"kind\":\"controller_imu_stage\",\"stage\":");
    quoted(stage);
    printf(",\"status\":");
    quoted(status);
    printf(",\"detail\":");
    quoted(detail ? detail : "");
    puts("}");
}

static void fail(int code, const char *stage, const char *detail)
{
    event(stage, "error", detail);
    _exit(code);
}

static void expired(int signal_number)
{
    static const char message[] =
        "{\"kind\":\"controller_imu_stage\",\"stage\":\"deadline\","
        "\"status\":\"timeout\"}\n";
    (void)signal_number;
    ssize_t written = write(STDERR_FILENO, message, sizeof(message) - 1);
    (void)written;
    _exit(124);
}

#ifndef A081_CONTROLLER_CONTROLS_ONLY
struct build_id_request { uintptr_t base; char hex[65]; };

static int find_build_id(struct dl_phdr_info *info, size_t size, void *opaque)
{
    struct build_id_request *r = opaque;
    (void)size;
    if ((uintptr_t)info->dlpi_addr != r->base)
        return 0;
    for (ElfW(Half) i = 0; i < info->dlpi_phnum; ++i) {
        const ElfW(Phdr) *ph = &info->dlpi_phdr[i];
        if (ph->p_type != PT_NOTE)
            continue;
        const unsigned char *p = (const unsigned char *)(r->base + ph->p_vaddr);
        const unsigned char *end = p + ph->p_memsz;
        while ((size_t)(end - p) >= sizeof(ElfW(Nhdr))) {
            ElfW(Nhdr) note;
            memcpy(&note, p, sizeof(note));
            p += sizeof(note);
            size_t namesz = ((size_t)note.n_namesz + 3) & ~(size_t)3;
            size_t descsz = ((size_t)note.n_descsz + 3) & ~(size_t)3;
            if (namesz > (size_t)(end - p) || descsz > (size_t)(end - p) - namesz)
                break;
            if (note.n_type == NT_GNU_BUILD_ID && note.n_namesz == 4 &&
                note.n_descsz <= 32 && memcmp(p, "GNU", 4) == 0) {
                for (uint32_t j = 0; j < note.n_descsz; ++j)
                    snprintf(r->hex + j * 2, 3, "%02x", p[namesz + j]);
                return 1;
            }
            p += namesz + descsz;
        }
    }
    return 1;
}

static struct sdk open_sdk(const char *path)
{
    event("dlopen", "begin", path);
    void *library = dlopen(path, RTLD_NOW | RTLD_LOCAL);
    if (!library)
        fail(65, "dlopen", dlerror());
    dlerror();
    void *symbol = dlsym(library, A081_CONTROLLER_FACTORY_SYMBOL);
    const char *error = dlerror();
    if (!symbol || error)
        fail(66, "factory", error);
    Dl_info info;
    if (!dladdr(symbol, &info))
        fail(66, "identity", "dladdr failed");
    const uintptr_t base = (uintptr_t)info.dli_fbase;
    struct build_id_request id = {.base = base, .hex = {0}};
    dl_iterate_phdr(find_build_id, &id);
    if (strcmp(id.hex, A081_CONTROLLER_SDK_BUILD_ID) ||
        (((uintptr_t)symbol & ~(uintptr_t)1) - base) != A081_CONTROLLER_FACTORY_OFFSET ||
        !((uintptr_t)symbol & 1))
        fail(67, "identity", "SDK BuildID/factory entry differs from reference");
    event("identity", "ok", id.hex);
    a081_controller_factory_fn factory;
    _Static_assert(sizeof(factory) == sizeof(symbol), "Function pointer ABI");
    memcpy(&factory, &symbol, sizeof(factory));
    struct sdk sdk = {.object = factory(A081_HAND_BOTH, 2)};
    if (!sdk.object)
        fail(68, "factory", "getInstance(2,2) returned NULL");
    const uintptr_t *vtable;
    memcpy(&vtable, sdk.object, sizeof(vtable));
    if ((uintptr_t)vtable != base + A081_CONTROLLER_VTABLE_OFFSET)
        fail(69, "vtable", "SDK object differs from reference");
    static const struct { unsigned slot; uintptr_t elf; } entries[] = {
        {A081_SLOT_INITIAL_MODE, 0x38451c},
        {A081_SLOT_IMU_INPUT, 0x384df8},
        {A081_SLOT_GET_POSE, 0x3865b0}
    };
    for (size_t i = 0; i < sizeof(entries) / sizeof(entries[0]); ++i) {
        const uintptr_t pointer = vtable[entries[i].slot / sizeof(uintptr_t)];
        if (!(pointer & 1) || (pointer & ~(uintptr_t)1) - base != entries[i].elf)
            fail(69, "vtable", "SDK method differs from reference");
    }
    a081_controller_initial_mode_fn initial;
    memcpy(&initial, vtable + A081_SLOT_INITIAL_MODE / sizeof(uintptr_t), sizeof(initial));
    memcpy(&sdk.imu, vtable + A081_SLOT_IMU_INPUT / sizeof(uintptr_t), sizeof(sdk.imu));
    memcpy(&sdk.pose, vtable + A081_SLOT_GET_POSE / sizeof(uintptr_t), sizeof(sdk.pose));
    const int mode_result = initial(sdk.object, 0);
    printf("{\"kind\":\"controller_imu_stage\",\"stage\":\"initial_mode\","
           "\"mode\":0,\"sdk_return\":%d}\n", mode_result);
    if (mode_result != 0)
        fail(71, "initial_mode", "Unexpected return from initial_mode(0)");
    return sdk;
}
#endif

static uint64_t clock_ns(clockid_t id)
{
    struct timespec t;
    if (clock_gettime(id, &t) || t.tv_sec < 0)
        fail(72, "clock", "clock_gettime failed");
    return (uint64_t)t.tv_sec * UINT64_C(1000000000) + (uint64_t)t.tv_nsec;
}

#ifndef A081_CONTROLLER_CONTROLS_ONLY
static struct anchor measure_anchor(void)
{
    uint64_t before = clock_ns(CLOCK_MONOTONIC);
    uint64_t boot = clock_ns(CLOCK_BOOTTIME);
    uint64_t after = clock_ns(CLOCK_MONOTONIC);
    if (after < before || after > INT64_MAX || boot > INT64_MAX)
        fail(72, "clock", "Invalid system clock sample");
    struct timespec mono_res, boot_res;
    if (clock_getres(CLOCK_MONOTONIC, &mono_res) ||
        clock_getres(CLOCK_BOOTTIME, &boot_res))
        fail(72, "clock", "clock_getres failed");
    uint64_t quantization = (uint64_t)mono_res.tv_sec * UINT64_C(1000000000) +
        (uint64_t)mono_res.tv_nsec + (uint64_t)boot_res.tv_sec * UINT64_C(1000000000) +
        (uint64_t)boot_res.tv_nsec;
    struct anchor a = {
        .monotonic_ns = before + (after - before) / 2,
        .boottime_ns = boot,
        .uncertainty_ns = (after - before + 1) / 2 + quantization
    };
    a.offset_ns = (int64_t)boot - (int64_t)a.monotonic_ns;
    return a;
}
#endif

static int literal(const char **p, const char *s)
{
    size_t n = strlen(s);
    if (strncmp(*p, s, n))
        return 0;
    *p += n;
    return 1;
}

static int unsigned_integer(const char **p, uint64_t *out)
{
    const char *s = *p;
    if (*s < '0' || *s > '9')
        return 0;
    uint64_t value = 0;
    do {
        unsigned digit = (unsigned)(*s++ - '0');
        if (value > (UINT64_MAX - digit) / 10)
            return 0;
        value = value * 10 + digit;
    } while (*s >= '0' && *s <= '9');
    *p = s;
    *out = value;
    return 1;
}

static int hex_digit(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

/* Deliberately the exact prefix written by ../nordic-stream.cpp, not a general
 * JSON API. Derived tail fields are ignored: CRC/hand/type are re-read from rx.
 * 1=record, 0=producer summary, -1=malformed/unknown line.
 */
static int parse_record(const char *line, struct record *r)
{
    const char *p = line;
    if (!strncmp(line,"{\"kind\":\"summary\",",18) ||
        !strncmp(line,"{\"kind\":\"hand_summary\",",23) ||
        !strncmp(line,"{\"kind\":\"transfer_summary\",",27)) return 0;
    p = line;
    if (!literal(&p, "{\"kind\":\"nordic_rx\",\"irq_ns\":") ||
        !unsigned_integer(&p, &r->irq_ns) ||
        !literal(&p, ",\"completion_ns\":") ||
        !unsigned_integer(&p, &r->completion_ns) ||
        !literal(&p, ",\"irq_sequence\":") ||
        !unsigned_integer(&p, &r->sequence) ||
        !literal(&p, ",\"rx\":\""))
        return -1;
    if (strlen(p) < 73)
        return -1;
    for (size_t i = 0; i < sizeof(r->raw); ++i) {
        int high = hex_digit(*p++), low = hex_digit(*p++);
        if (high < 0 || low < 0)
            return -1;
        r->raw[i] = (uint8_t)((high << 4) | low);
    }
    if (!literal(&p, "\",\"spi_bytes\":72,\"rx_offset\":") ||
        !unsigned_integer(&p, &r->offset) || (r->offset != 0 && r->offset != 36) ||
        !literal(&p, ",\"raw72\":\"") || strlen(p) < 145) return -1;
    for (size_t i = 0; i < sizeof(r->raw72); ++i) {
        int high = hex_digit(*p++), low = hex_digit(*p++);
        if (high < 0 || low < 0) return -1;
        r->raw72[i] = (uint8_t)((high << 4) | low);
    }
    if (*p++ != '"' || (*p != ',' && *p != '}') ||
        memcmp(r->raw, r->raw72 + r->offset, sizeof(r->raw)) ||
        !r->irq_ns || r->irq_ns > r->completion_ns || !r->sequence)
        return -1;
    size_t n = strlen(line);
    while (n && (line[n - 1] == '\n' || line[n - 1] == '\r'))
        --n;
    return n && line[n - 1] == '}' ? 1 : -1;
}

#ifndef A081_CONTROLLER_CONTROLS_ONLY
static void number(double value)
{
    if (isfinite(value)) printf("%.17g", value);
    else printf("null");
}

static void array(const double *v, size_t n)
{
    putchar('[');
    for (size_t i = 0; i < n; ++i) {
        if (i) putchar(',');
        number(v[i]);
    }
    putchar(']');
}

static void pose_output(struct sdk *sdk, unsigned hand, uint64_t query_time)
{
    double position[3] = {NAN, NAN, NAN};
    double orientation[4] = {NAN, NAN, NAN, NAN};
    double motion[4][3];
    for (size_t i = 0; i < 4; ++i)
        for (size_t j = 0; j < 3; ++j)
            motion[i][j] = NAN;
    uint64_t returned_time = query_time;
    float confidence = sdk->pose(sdk->object, position, orientation, &returned_time,
                                motion[0], motion[1], motion[2], motion[3], (int)hand);
    printf("{\"kind\":\"controller_sdk_pose\",\"hand\":%u,"
           "\"query_boottime_ns\":%" PRIu64 ",\"sdk_time_ns\":%" PRIu64 ","
           "\"sdk_confidence\":", hand, query_time, returned_time);
    number(confidence);
    printf(",\"position_sdk\":"); array(position, 3);
    printf(",\"quaternion_wxyz_sdk\":"); array(orientation, 4);
    printf(",\"motion_vectors_sdk\":[");
    for (size_t i = 0; i < 4; ++i) {
        if (i) putchar(',');
        array(motion[i], 3);
    }
    puts("],\"orientation_valid\":false,\"orientation_tracked\":false,"
         "\"position_valid\":false,\"position_tracked\":false,"
         "\"validity_reason\":\"optical_hmd_clock_lifecycle_not_validated\"}");
}
#endif

static void controls_output(struct hand_state *hand, unsigned side, uint64_t now,
                            uint64_t sequence, uint32_t updated, int replay,
                            const char *reason)
{
    const struct a081_controller_controls *s = &hand->controls;
    uint32_t valid = replay ? 0 :
        a081_controller_controls_valid_mask(s, now, CONTROL_MAX_AGE_NS);
    printf("{\"kind\":\"controller_controls\",\"version\":1,\"hand\":%u,"
           "\"input_mode\":\"%s\",\"reason\":\"%s\",\"source_sequence\":%" PRIu64
           ",\"observation_monotonic_ns\":%" PRIu64 ",\"max_age_ns\":%" PRIu64
           ",\"pattern_known\":%s,\"stock_enable\":%s,\"head_power_state\":",
           side, replay ? "replay" : "live", reason, sequence, now,
           CONTROL_MAX_AGE_NS, s->pattern_known ? "true" : "false",
           s->stock_enable ? "true" : "false");
    if (s->pattern_known) printf("%u", s->head_power_state); else printf("null");
    printf(",\"gzll_raw_state\":");
    if (s->gzll_state_known) printf("%u", s->gzll_raw_state); else printf("null");
    printf(",\"updated_mask\":%u,\"seen_mask\":%u,\"valid_mask\":%u,"
           "\"mode1_receipt_monotonic_ns\":%" PRIu64 ",\"mode1_sequence\":%" PRIu64
           ",\"joystick_receipt_monotonic_ns\":%" PRIu64 ",\"joystick_sequence\":%" PRIu64
           ",\"stock_buttons_raw\":%u,\"stock_touch_raw\":%u,"
           "\"stock_index_finger_raw\":%u,\"stock_middle_finger_raw\":%u,"
           "\"stock_joystick_raw\":[%d,%d],\"input_active\":%s,"
           "\"event_time_domain\":\"transport_monotonic_receipt\","
           "\"openxr_actions_mapped\":false,\"position_tracking\":false}\n",
           updated, s->seen_mask, valid, s->mode1_receipt_monotonic_ns,
           s->mode1_sequence, s->joystick_receipt_monotonic_ns, s->joystick_sequence,
           s->buttons_raw, s->touch_raw, s->index_finger_raw, s->middle_finger_raw,
           s->joystick_raw[0], s->joystick_raw[1], valid ? "true" : "false");
    hand->emitted_controls_valid = valid;
}

int main(int argc, char **argv)
{
    const char *sdk_path = NULL;
    unsigned seconds = 20;
    int replay = 0;
#ifdef A081_CONTROLLER_CONTROLS_ONLY
    int controls_only = 1;
#else
    int controls_only = 0;
#endif
    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "--sdk") && i + 1 < argc) {
            sdk_path = argv[++i];
        } else if (!strcmp(argv[i], "--seconds") && i + 1 < argc) {
            const char *p = argv[++i];
            uint64_t value;
            if (!unsigned_integer(&p, &value) || *p || value < 1 || value > 120)
                return 64;
            seconds = (unsigned)value;
        } else if (!strcmp(argv[i], "--replay")) {
            replay = 1;
        } else if (!strcmp(argv[i], "--controls-only")) {
            controls_only = 1;
        } else {
            fprintf(stderr, "usage: %s --sdk /system/lib/libControllerSDK_EXPORT.so "
                    "[--seconds 1..120]\n"
                    "       %s --replay [--seconds 1..120]\n"
                    "       %s --controls-only [--seconds 1..120]\n"
                    "stdin: exact real nordic-stream JSONL; replay never calls SDK\n",
                    argv[0], argv[0], argv[0]);
            return 64;
        }
    }
    if (((replay || controls_only) && sdk_path) ||
        (!replay && !controls_only && (!sdk_path || sdk_path[0] != '/')))
        return 64;
    setvbuf(stdout, NULL, _IONBF, 0);
    struct sigaction action = {0};
    action.sa_handler = expired;
    sigemptyset(&action.sa_mask);
    if (sigaction(SIGALRM, &action, NULL))
        fail(70, "deadline", strerror(errno));
    alarm(seconds);
    event("exec", "ok", replay ? "real-log replay: no SDK calls" :
          controls_only ? "live control input only: no SDK calls" : "live JSONL input");
#ifndef A081_CONTROLLER_CONTROLS_ONLY
    struct sdk sdk = {0};
    struct anchor initial = {0};
    if (!replay && !controls_only) {
        sdk = open_sdk(sdk_path);
        initial = measure_anchor();
        printf("{\"kind\":\"controller_clock_anchor\",\"monotonic_ns\":%" PRIu64
               ",\"boottime_ns\":%" PRIu64 ",\"boottime_minus_monotonic_ns\":%" PRId64
               ",\"uncertainty_ns\":%" PRIu64
               ",\"clock_policy\":\"stock_type7_boottime_window_per_sample\","
               "\"radio_sync_verified\":false}\n",
               initial.monotonic_ns, initial.boottime_ns, initial.offset_ns,
               initial.uncertainty_ns);
    }
#endif
    struct hand_state hands[2] = {0};
    uint64_t records = 0, headers = 0, checksums = 0, patterns = 0, imu_packets = 0;
    uint64_t submitted = 0, previous_completion = 0, previous_sequence = 0, previous_irq = 0, previous_offset = 36;
    uint8_t previous_raw72[72] = {0};
    char line[MAX_LINE];
    while (fgets(line, sizeof(line), stdin)) {
        if (!strchr(line, '\n') && !feof(stdin))
            fail(73, "input", "Oversize/truncated nordic-stream record");
        struct record record;
        int parsed = parse_record(line, &record);
        if (parsed < 0)
            fail(73, "input", "Expected atomic72 nordic-stream JSONL with verified36-byte slice");
        if (!parsed)
            continue;
        ++records;
        if (record.offset == 0) {
            if (previous_offset != 36 || record.completion_ns <= previous_completion || record.sequence <= previous_sequence)
                fail(73, "input", "New atomic72 transfer order invalid");
            previous_completion = record.completion_ns;
            previous_sequence = record.sequence;
            previous_irq = record.irq_ns;
            memcpy(previous_raw72, record.raw72, sizeof(previous_raw72));
        } else if (previous_offset != 0 || record.completion_ns != previous_completion ||
                   record.sequence != previous_sequence || record.irq_ns != previous_irq ||
                   memcmp(previous_raw72, record.raw72, sizeof(previous_raw72))) {
            fail(73, "input", "Second atomic72 slice boundary mismatch");
        }
        previous_offset = record.offset;
        uint64_t control_now = replay ? record.completion_ns : clock_ns(CLOCK_MONOTONIC);
        for (unsigned side = 0; side < 2; ++side) {
            uint32_t valid = replay ? 0 : a081_controller_controls_valid_mask(
                &hands[side].controls, control_now, CONTROL_MAX_AGE_NS);
            if (valid != hands[side].emitted_controls_valid)
                controls_output(&hands[side], side, control_now, record.sequence,
                                0, replay, "expired");
        }
        struct a081_controller_wire_packet packet;
        enum a081_controller_wire_result decoded = a081_controller_decode_wire(record.raw, &packet);
        if (decoded == A081_WIRE_HEADER) { ++headers; continue; }
        if (decoded == A081_WIRE_CHECKSUM) { ++checksums; continue; }
        if (decoded != A081_WIRE_OK) continue;
        struct hand_state *hand = &hands[packet.hand];
        if (packet.type == 2) {
            ++patterns;
            const uint8_t *p = packet.payload;
            if (hand->version_known && (hand->version[0] != p[24] ||
                hand->version[1] != p[23] || hand->version[2] != p[22]))
                fail(74, "version", "Head PatternID version changed; synchronization must be re-established");
            hand->version[0] = p[24]; hand->version[1] = p[23]; hand->version[2] = p[22];
            hand->version_known = 1;
            a081_controller_controls_update(&hand->controls, &packet,
                                             record.completion_ns, record.sequence);
            controls_output(hand, packet.hand, control_now, record.sequence, 0,
                            replay, "pattern");
            printf("{\"kind\":\"controller_pattern\",\"hand\":%u,\"head_version\":[%u,%u,%u],"
                   "\"head_power_state\":%u,\"gzll_raw_state\":%u,\"source_sequence\":%" PRIu64 "}\n",
                   packet.hand, p[24], p[23], p[22], hand->controls.head_power_state,
                   p[25] >> 4, record.sequence);
            continue;
        }
        if (packet.type != 7)
            continue;
        ++imu_packets;
        uint32_t controls_updated = a081_controller_controls_update(
            &hand->controls, &packet, record.completion_ns, record.sequence);
        controls_output(hand, packet.hand, control_now, record.sequence,
                        controls_updated, replay, "type7");
        if (controls_only)
            continue;
#ifndef A081_CONTROLLER_CONTROLS_ONLY
        const char *rejection = NULL;
        struct a081_controller_imu_input input = {0};
        uint64_t wire_clock = 0, receive_boot_ns = 0;
        struct anchor current = {0};
        /* Quaternion/vector/tick unpacking does not depend on the version.
         * Hide scaled time until a real PatternID supplies its unit. */
        (void)a081_controller_unpack_type7(&packet, hand->version[2], &input, &wire_clock);
        if (!hand->version_known) {
            rejection = "head_pattern_version_unknown";
        } else {
            int nonzero_q = 0;
            for (size_t i = 0; i < 7; ++i) {
                if (!isfinite(input.values[i])) rejection = "nonfinite_sample";
                if (i < 4 && input.values[i] != 0) nonzero_q = 1;
            }
            if (!nonzero_q) rejection = "zero_quaternion";
            if (!input.aligned_time_ns || input.aligned_time_ns <= hand->last_time_ns)
                rejection = "sample_time_not_increasing";
            if (input.aligned_time_ns > hand->last_time_ns)
                hand->last_time_ns = input.aligned_time_ns;
        }
        if (replay) {
            if (!rejection) rejection = "replay_never_submits";
        } else {
            current = measure_anchor();
            if (!rejection && (current.uncertainty_ns > MAX_ANCHOR_UNCERTAINTY_NS ||
                initial.uncertainty_ns > MAX_ANCHOR_UNCERTAINTY_NS))
                rejection = "clock_anchor_uncertainty";
            if (!rejection && (current.offset_ns - initial.offset_ns > MAX_ANCHOR_CHANGE_NS ||
                initial.offset_ns - current.offset_ns > MAX_ANCHOR_CHANGE_NS))
                rejection = "system_clock_offset_changed";
            if (!rejection && (record.completion_ns > current.monotonic_ns ||
                current.monotonic_ns - record.completion_ns >= MAX_SAMPLE_AGE_NS))
                rejection = "transport_not_live";
            if (!rejection) {
                int64_t completion = (int64_t)record.completion_ns;
                if (completion <= 0 || current.offset_ns < -completion ||
                    (current.offset_ns > 0 && completion > INT64_MAX - current.offset_ns)) {
                    rejection = "receive_clock_overflow";
                } else {
                    receive_boot_ns = (uint64_t)(completion + current.offset_ns);
                    /* Stock uses the scaled timestamp as-is and rejects ages
                     * outside (0,33ms). Bound both actual transport receipt
                     * and current SDK dispatch: pipeline delay must not make
                     * a packet that was fresh at SPI receipt stale at input.
                     * No fitted offset or claimed sync flag is substituted.
                     */
                    if (receive_boot_ns <= input.aligned_time_ns ||
                        receive_boot_ns - input.aligned_time_ns <= current.uncertainty_ns ||
                        receive_boot_ns - input.aligned_time_ns >=
                            MAX_SAMPLE_AGE_NS - current.uncertainty_ns)
                        rejection = "sample_outside_boottime_window";
                    else if (current.boottime_ns <= input.aligned_time_ns ||
                        current.boottime_ns - input.aligned_time_ns <= current.uncertainty_ns ||
                        current.boottime_ns - input.aligned_time_ns >=
                            MAX_SAMPLE_AGE_NS - current.uncertainty_ns)
                        rejection = "sample_stale_at_sdk_dispatch";
                }
            }
        }
        printf("{\"kind\":\"controller_imu_input\",\"hand\":%u,\"source_sequence\":%" PRIu64
               ",\"wire_clock48\":%" PRIu64 ",\"scaled_time_ns\":",
               packet.hand, record.sequence, wire_clock);
        if (hand->version_known) printf("%" PRIu64, input.aligned_time_ns);
        else printf("null");
        printf(",\"receive_boottime_ns\":");
        if (receive_boot_ns) printf("%" PRIu64, receive_boot_ns);
        else printf("null");
        printf(",\"sdk_dispatch_boottime_ns\":");
        if (!replay) printf("%" PRIu64, current.boottime_ns);
        else printf("null");
        printf(",\"anchor_uncertainty_ns\":");
        if (!replay) printf("%" PRIu64, current.uncertainty_ns);
        else printf("null");
        printf(",\"values\":");
        array(input.values, 7);
        printf(",\"sideband\":[%d,%d,%d],\"sdk_submit_requested\":%s",
               input.sideband[0], input.sideband[1], input.sideband[2], rejection ? "false" : "true");
        if (rejection) {
            printf(",\"rejection\":"); quoted(rejection); puts("}");
            continue;
        }
        puts("}");
        int status = sdk.imu(sdk.object, input.values, input.sideband,
                             input.aligned_time_ns, packet.hand);
        ++submitted;
        printf("{\"kind\":\"controller_sdk_imu_result\",\"hand\":%u,"
               "\"source_sequence\":%" PRIu64 ",\"sdk_return\":%d,"
               "\"fusion_acceptance\":\"unknown\"}\n", packet.hand, record.sequence, status);
        pose_output(&sdk, packet.hand, current.boottime_ns);
#endif
    }
    if (ferror(stdin))
        fail(73, "input", strerror(errno));
    uint64_t control_end = replay ? previous_completion : clock_ns(CLOCK_MONOTONIC);
    for (unsigned side = 0; side < 2; ++side) {
        a081_controller_controls_release(&hands[side].controls);
        controls_output(&hands[side], side, control_end, 0, 0, replay, "eof");
    }
    if (previous_offset != 36) fail(73, "input", "Truncated atomic72 transfer at EOF");
    printf("{\"kind\":\"controller_imu_summary\",\"input_mode\":\"%s\","
           "\"records\":%" PRIu64 ",\"rejected_headers\":%" PRIu64
           ",\"rejected_checksums\":%" PRIu64 ",\"pattern_packets\":%" PRIu64
           ",\"type7_packets\":%" PRIu64 ",\"sdk_calls\":%" PRIu64
           ",\"no_type7_input\":%s,\"controls_only\":%s,"
           "\"motion_verified\":false,\"position_tracking\":false}\n",
           replay ? "replay" : "live", records, headers, checksums, patterns,
           imu_packets, submitted, imu_packets ? "false" : "true",
           controls_only ? "true" : "false");
    /* Factory workers belong to this bounded process. Keep the library loaded
     * and do not invoke an unverified singleton destructor during process exit.
     */
    _exit(0);
}
