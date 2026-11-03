// SPDX-License-Identifier: GPL-3.0-or-later
// Native ARM32 bridge to the original A081 SDK. Inputs are live root-owned
// camera/radio packets; replay, synthetic images and dead-reckoned positions
// are not input modes. Raw SDK output does not establish OpenVR coordinates.
#include <xr/device/a081_controller_input_wire.h>
#include <xr/controller/a081_controller_image_abi.hpp>
#include <xr/device/a081_camera_timing.hpp>
#define _Static_assert static_assert
#include <xr/controller/a081_controller_controls.h>
#undef _Static_assert
#include <xr/controller/a081_controller_pose_wire.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <fcntl.h>
#include <link.h>
#include <memory>
#include <poll.h>
#include <signal.h>
#include <stdexcept>
#include <string>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/ucontext.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>

#if !defined(__ANDROID__) || !defined(__arm__)
#error "Build with Android ARM32 NDK and the original libc++_shared.so"
#endif
#ifndef F_GET_SEALS
#define F_GET_SEALS 1034
#define F_SEAL_SHRINK 0x0002
#define F_SEAL_GROW 0x0004
#define F_SEAL_WRITE 0x0008
#endif

namespace {
using u64 = uint64_t;
constexpr u64 ms = 1000000, max_age = 100 * ms;
volatile sig_atomic_t stopping;
bool detailed_logging = true;
void stopped(int) { stopping = 1; }

struct sigaction previous_sigsegv;
void fault_write(const char *text, size_t size)
{
    while (size) {
        const ssize_t count = write(STDERR_FILENO, text, size);
        if (count < 0 && errno == EINTR) continue;
        if (count <= 0) return;
        text += count; size -= size_t(count);
    }
}
char *fault_text(char *out, const char *text)
{
    while (*text) *out++ = *text++;
    return out;
}
char *fault_hex(char *out, uint32_t value)
{
    static constexpr char digits[] = "0123456789abcdef";
    *out++ = '0'; *out++ = 'x';
    for (int shift = 28; shift >= 0; shift -= 4) *out++ = digits[(value >> shift) & 15];
    return out;
}
void crashed(int number, siginfo_t *info, void *context)
{
    // No stdio, allocation, unwinding, or SDK calls in a synchronous fault.
    // After restoring the previous action, return to the unchanged faulting
    // instruction so Android's original fatal-signal path receives it again.
    const int saved_errno = errno;
    const auto &r = static_cast<const ucontext_t *>(context)->uc_mcontext;
    const struct { const char *name; uint32_t value; } fields[] = {
        {"signal", uint32_t(number)}, {"si_code", uint32_t(info->si_code)},
        {"si_addr", uint32_t(reinterpret_cast<uintptr_t>(info->si_addr))},
        {"r0", r.arm_r0}, {"r1", r.arm_r1}, {"r2", r.arm_r2}, {"r3", r.arm_r3},
        {"r4", r.arm_r4}, {"r5", r.arm_r5}, {"r6", r.arm_r6}, {"r7", r.arm_r7},
        {"r8", r.arm_r8}, {"r9", r.arm_r9}, {"r10", r.arm_r10},
        {"r11", r.arm_fp}, {"r12", r.arm_ip}, {"sp", r.arm_sp},
        {"lr", r.arm_lr}, {"pc", r.arm_pc}, {"cpsr", r.arm_cpsr},
        {"trap", r.trap_no}, {"error", r.error_code}, {"fault_address", r.fault_address}
    };
    char line[768];
    char *out = fault_text(line, "controller_sdk_fault");
    for (const auto &field : fields) {
        *out++ = ' '; out = fault_text(out, field.name); *out++ = '=';
        out = fault_hex(out, field.value);
    }
    *out++ = '\n'; fault_write(line, size_t(out - line));
    static constexpr char begin[] = "controller_sdk_fault_maps_begin\n";
    static constexpr char end[] = "controller_sdk_fault_maps_end\n";
    fault_write(begin, sizeof(begin) - 1);
    const int maps = open("/proc/self/maps", O_RDONLY | O_CLOEXEC);
    if (maps >= 0) {
        char buffer[4096];
        while (true) {
            const ssize_t count = read(maps, buffer, sizeof(buffer));
            if (count < 0 && errno == EINTR) continue;
            if (count <= 0) break;
            fault_write(buffer, size_t(count));
        }
        close(maps);
    } else {
        const int error = errno;
        out = fault_text(line, "controller_sdk_fault_maps_open_errno=");
        out = fault_hex(out, uint32_t(error)); *out++ = '\n';
        fault_write(line, size_t(out - line));
    }
    fault_write(end, sizeof(end) - 1);
    sigaction(SIGSEGV, &previous_sigsegv, nullptr);
    errno = saved_errno;
}
void install_fault_diagnostic()
{
    struct sigaction action{};
    action.sa_sigaction = crashed;
    action.sa_flags = SA_SIGINFO | SA_ONSTACK;
    sigemptyset(&action.sa_mask);
    if (sigaction(SIGSEGV, &action, &previous_sigsegv))
        std::fprintf(stderr, "controller_sdk_fault_install_errno=%d\n", errno);
}
u64 now(clockid_t id)
{
    timespec ts{};
    if (clock_gettime(id, &ts)) throw std::runtime_error("clock_gettime");
    return u64(ts.tv_sec) * 1000000000ULL + ts.tv_nsec;
}
bool fresh(u64 sample, u64 current, u64 limit = max_age)
{
    return sample && sample <= current && current - sample < limit;
}
void require(bool ok, const char *reason)
{
    if (!ok) throw std::runtime_error(reason);
}
void event(const char *kind, const char *reason, unsigned hand = 2)
{
    std::printf("{\"kind\":\"%s\",\"reason\":\"%s\",\"hand\":%u}\n", kind, reason, hand);
}
template <size_t N> bool finite(const double (&values)[N])
{
    for (auto value : values) if (!std::isfinite(value)) return false;
    return true;
}
bool quaternion(const double *q)
{
    double norm = 0;
    for (unsigned i = 0; i < 4; ++i) norm += q[i] * q[i];
    return std::isfinite(norm) && std::abs(norm - 1) < .02;
}
void array(const double *p, unsigned count)
{
    std::putchar('[');
    for (unsigned i = 0; i < count; ++i) {
        if (i) std::putchar(',');
        if (std::isfinite(p[i])) std::printf("%.17g", p[i]); else std::printf("null");
    }
    std::putchar(']');
}
struct Anchor {
    u64 mono, boot, uncertainty;
    int64_t offset;
};
Anchor anchor()
{
    const auto a = now(CLOCK_MONOTONIC), b = now(CLOCK_BOOTTIME), c = now(CLOCK_MONOTONIC);
    timespec mr{}, br{};
    require(!clock_getres(CLOCK_MONOTONIC, &mr) && !clock_getres(CLOCK_BOOTTIME, &br), "clock resolution");
    require(c >= a && b < INT64_MAX && c < INT64_MAX, "clock range");
    Anchor out{a + (c-a)/2, b, (c-a+1)/2 + u64(mr.tv_sec+br.tv_sec)*1000000000ULL + mr.tv_nsec+br.tv_nsec, 0};
    out.offset = int64_t(b) - int64_t(out.mono);
    return out;
}
struct BuildId { uintptr_t base; char hex[65]{}; };
int build_id(dl_phdr_info *info, size_t, void *opaque)
{
    auto &r = *static_cast<BuildId *>(opaque);
    if (uintptr_t(info->dlpi_addr) != r.base) return 0;
    for (unsigned i = 0; i < info->dlpi_phnum; ++i) {
        auto &ph = info->dlpi_phdr[i];
        if (ph.p_type != PT_NOTE) continue;
        auto *p = reinterpret_cast<const unsigned char *>(r.base + ph.p_vaddr);
        auto *end = p + ph.p_memsz;
        while (size_t(end-p) >= sizeof(ElfW(Nhdr))) {
            ElfW(Nhdr) n{}; std::memcpy(&n, p, sizeof(n)); p += sizeof(n);
            const size_t namesz = (size_t(n.n_namesz)+3)&~size_t(3), descsz = (size_t(n.n_descsz)+3)&~size_t(3);
            if (namesz > size_t(end-p) || descsz > size_t(end-p)-namesz) break;
            if (n.n_type == NT_GNU_BUILD_ID && n.n_namesz == 4 && n.n_descsz <= 32 && !std::memcmp(p,"GNU",4)) {
                for (unsigned j=0;j<n.n_descsz;++j) std::snprintf(r.hex+j*2,3,"%02x",p[namesz+j]);
                return 1;
            }
            p += namesz + descsz;
        }
    }
    return 1;
}
struct Sdk {
    void *object{};
    a081_controller_push_hmd_pose_fn hmd{};
    a081_controller_push_imu_fn imu{};
    a081_controller_get_pose_fn pose{};
    a081_controller::PushImages images{};
    explicit Sdk(const char *path)
    {
        void *lib = dlopen(path, RTLD_NOW|RTLD_LOCAL);
        if (!lib) throw std::runtime_error(dlerror());
        void *symbol = dlsym(lib,A081_CONTROLLER_FACTORY_SYMBOL);
        Dl_info info{};
        require(symbol && dladdr(symbol,&info), "SDK factory missing");
        const auto base=uintptr_t(info.dli_fbase);
        BuildId id{base,{}}; dl_iterate_phdr(build_id,&id);
        const bool old=!std::strcmp(id.hex,A081_CONTROLLER_SDK_BUILD_ID);
        const bool live=!std::strcmp(id.hex,"9e04a7620e90dba0c5c56e67bd1c9bcb740b9256");
        const uintptr_t factory_offset=old ? A081_CONTROLLER_FACTORY_OFFSET:0x380e9c;
        const uintptr_t vtable_offset=old ? A081_CONTROLLER_VTABLE_OFFSET:0x7a63b0;
        require((old || live) && (uintptr_t(symbol)&1) &&
            (uintptr_t(symbol)&~uintptr_t(1))-base==factory_offset,"SDK identity mismatch");
        a081_controller_factory_fn factory; std::memcpy(&factory,&symbol,sizeof(factory));
        object=factory(A081_HAND_BOTH,2); require(object,"SDK factory null");
        auto *vt=*static_cast<const uintptr_t **>(object);
        require(uintptr_t(vt)==base+vtable_offset,"SDK vtable mismatch");
        const std::pair<unsigned,uintptr_t> entries[]={{0,old?0x384460u:0x374130u},{4,old?0x38451cu:0x3741ecu},
            {0x0c,old?0x3845a4u:0x374274u},{0x18,old?0x384df8u:0x374ac8u},
            {0x1c,old?0x3850d8u:0x374da8u},{0x34,old?0x3865b0u:0x376210u}};
        for (auto e:entries) require((vt[e.first/4]&1) &&
            (vt[e.first/4]&~uintptr_t(1))-base==e.second,"SDK method mismatch");
        std::memcpy(&hmd,vt+0x0c/4,sizeof(hmd)); std::memcpy(&imu,vt+0x18/4,sizeof(imu));
        std::memcpy(&images,vt+0x1c/4,sizeof(images)); std::memcpy(&pose,vt+0x34/4,sizeof(pose));
        a081_controller_initial_mode_fn mode,start;
        std::memcpy(&mode,vt+1,sizeof(mode)); std::memcpy(&start,vt,sizeof(start));
        require(mode(object,0)==0 && start(object,0)==0,"SDK mode/start failed");
        event("controller_sdk_stage","original_factory_mode0_start0_complete");
        // The singleton has no recovered destructor. Keep lib/object alive
        // until the bounded worker process exits, as the validated SDK probe.
    }
};
struct Hand {
    a081_controller_controls controls{};
    u64 last_radio{}, last_clock{}, last_imu{}, last_image{}, last_hmd{};
    unsigned state{};
    u64 accepted_imu{}, rejected_imu{}, images{}, image_attempts{}, epoch_images{}, epoch{}, epoch_start_boot{};
    bool stale{true};
};
std::atomic<unsigned> live_leases{0}, callbacks{0};
struct Lease {
    void *memory{MAP_FAILED}; size_t bytes{}; int fd{-1};
    explicit Lease(int source) : fd(source)
    {
        struct stat st{};
        if (fstat(fd,&st) || st.st_size <= 0 || st.st_size > 8*1024*1024) {
            close(fd); throw std::runtime_error("camera memfd size");
        }
        const int seals=fcntl(fd,F_GET_SEALS);
        if (seals<0 || (seals&(F_SEAL_WRITE|F_SEAL_GROW|F_SEAL_SHRINK))!=(F_SEAL_WRITE|F_SEAL_GROW|F_SEAL_SHRINK)) {
            close(fd); throw std::runtime_error("camera memfd not immutable");
        }
        // The stock SDK writes into its image input. Private COW preserves the
        // sealed source frame while providing the SDK's writable buffer contract.
        bytes=size_t(st.st_size); memory=mmap(nullptr,bytes,PROT_READ|PROT_WRITE,MAP_PRIVATE,fd,0);
        if (memory==MAP_FAILED) { close(fd); throw std::runtime_error("camera mmap"); }
        ++live_leases;
    }
    ~Lease() { munmap(memory,bytes); close(fd); --live_leases; }
};
struct Client { int fd{-1}; u64 sequence{}; unsigned type{}; };

class Worker {
public:
    Worker(const char *library,int output,bool candidate_coordinates,const char *snapshot_directory):sdk(library), output_fd(output), initial(anchor()), candidate_profile(candidate_coordinates), snapshot_directory_(snapshot_directory ? snapshot_directory : "")
    {
        require(initial.uncertainty<=100000,"clock anchor uncertainty");
        session=now(CLOCK_MONOTONIC);
    }
    void invalidate(unsigned side,const char *reason)
    {
        Hand &h=hands[side];
        h.last_clock=h.last_imu=h.last_image=h.last_hmd=0;
        h.epoch_images=0;++h.epoch;h.epoch_start_boot=now(CLOCK_BOOTTIME);
        a081_controller_controls_release(&h.controls);
        event("controller_tracking_reset",reason,side);
    }
    void state(unsigned side,unsigned value,const char *reason)
    {
        Hand &h=hands[side];
        if(h.state==value)return;
        const auto before=h.state;
        invalidate(side,reason);h.state=value;
        std::printf("{\"kind\":\"controller_state\",\"hand\":%u,\"from\":%u,\"to\":%u,\"reason\":\"%s\",\"epoch\":%llu}\n",
                    side,before,value,reason,(unsigned long long)h.epoch);
    }
    void source_closed(unsigned type)
    {
        for(unsigned side=0;side<2;++side) {
            invalidate(side,type==A081_CONTROLLER_INPUT_RADIO?"radio_source_closed":"camera_source_closed");
            if(type==A081_CONTROLLER_INPUT_RADIO) {
                hands[side].controls={};hands[side].state=0;hands[side].last_radio=0;
                hands[side].stale=true;
            }
        }
        if(type==A081_CONTROLLER_INPUT_RADIO)last_transport=0;
    }
    void radio(const a081_controller_radio_input &r)
    {
        const auto a=anchor();
        if (!fresh(r.completion_monotonic_ns,a.mono,33*ms) || r.irq_monotonic_ns>r.completion_monotonic_ns || r.reserved) {
            event("controller_radio_reject","transport_not_live");return;
        }
        // Every real SPI slice reaches this endpoint, including empty slots.
        // Link state is an Android callback value, not an IMU-rate watchdog.
        last_transport=r.completion_monotonic_ns;
        a081_controller_wire_packet p{};
        if (a081_controller_decode_wire(r.raw,&p)!=A081_WIRE_OK) return;
        Hand &h=hands[p.hand];
        if (p.type==2) {
            const auto previous=h.controls;
            a081_controller_controls_update(&h.controls,&p,r.completion_monotonic_ns,r.header.sequence);
            // ParsePatternID clears ctrl state only for the complete byte04,
            // before >=25 firmware splits GZLL/head nibbles. 0x34 is different.
            if(p.payload[25]==4)state(p.hand,0,"stock_full_status04");
            if(!previous.pattern_known || std::memcmp(previous.head_version,h.controls.head_version,3) ||
               previous.stock_enable!=h.controls.stock_enable)
                invalidate(p.hand,"pattern_or_enable_changed");
            if(h.controls.head_power_state>=6) {
                // The stock parser suppresses this invalid head-power callback.
                h.controls.head_power_state=previous.head_power_state;
                h.controls.stock_enable=previous.stock_enable;
            }
            std::printf("{\"kind\":\"controller_pattern\",\"hand\":%u,\"head_version\":[%u,%u,%u],\"head_power\":%u,\"gzll_raw\":%u,\"link_present\":%s,\"tracking_enabled\":%s,\"ctrl_state\":%u}\n",
                unsigned(p.hand),p.payload[24],p.payload[23],p.payload[22],h.controls.head_power_state,
                h.controls.gzll_raw_state,linked(h)?"true":"false",h.controls.stock_enable?"true":"false",h.state);
            return;
        }
        if (p.type!=7 || !h.controls.pattern_known) return;
        if(h.last_radio && !fresh(h.last_radio,a.mono) && !h.stale) {
            invalidate(p.hand,"imu_receipt_expired");h.stale=true;
        }
        a081_controller_imu_input input{};u64 wire{};
        a081_controller_unpack_type7(&p,h.controls.head_version[2],&input,&wire);
        bool nonzero=false;
        for (unsigned i=0;i<4;++i) nonzero|=input.values[i]!=0;
        // Current serial ParseImuPreInt: ctrl power5 for nonzero quaternion,
        // power7 for allzero, then head-power callback from cached GZLL.
        // JNI only stores5/7. Its sleep cache must not become a fresh XR pose.
        state(p.hand,nonzero?5:7,nonzero?"stock_type7_awake":"stock_type7_sleep");
        unsigned power=h.controls.head_power_state;
        if(h.controls.head_version[2]<25)power=3;
        else if(h.controls.gzll_raw_state<=2)power=1;
        else if(h.controls.gzll_raw_state==3)power=3;
        else if(h.controls.gzll_raw_state==4)power=4;
        const bool enabled=power==3;
        if(enabled!=bool(h.controls.stock_enable))invalidate(p.hand,"type7_head_power_changed");
        h.controls.head_power_state=uint8_t(power);h.controls.stock_enable=enabled;
        h.last_radio=r.completion_monotonic_ns;h.stale=false;
        a081_controller_controls_update(&h.controls,&p,r.completion_monotonic_ns,r.header.sequence);
        if(!nonzero || !enabled)a081_controller_controls_release(&h.controls);
        const u64 t=input.aligned_time_ns;
        const int64_t receipt=int64_t(r.completion_monotonic_ns)+a.offset;
        const bool ok=enabled && nonzero && finite(input.values) &&
            quaternion(input.values) && t>h.last_clock && a.uncertainty<=100000 &&
            std::abs(a.offset-initial.offset)<=1000000 && receipt>0 &&
            fresh(t,u64(receipt),33*ms-a.uncertainty) && u64(receipt)-t>a.uncertainty &&
            fresh(t,a.boot,33*ms-a.uncertainty) && a.boot-t>a.uncertainty;
        if (!ok) {
            ++h.rejected_imu;
            if (h.rejected_imu<=8 || h.rejected_imu%500==0)
                std::printf("{\"kind\":\"controller_imu_reject\",\"hand\":%u,\"sample_ns\":%llu,\"now_ns\":%llu,\"count\":%llu,\"ctrl_state\":%u,\"head_power\":%u}\n",
                    unsigned(p.hand),(unsigned long long)t,(unsigned long long)a.boot,(unsigned long long)h.rejected_imu,h.state,power);
            return;
        }
        // A rejected future/reset clock must never poison the monotonic gate.
        // A real sleep/enable/transport epoch change resets this gate; a fresh
        // sample still has to satisfy the same original33ms receipt window.
        const int rc=sdk.imu(sdk.object,input.values,input.sideband,t,p.hand);
        if (rc==0) { h.last_clock=t;h.last_imu=t; ++h.accepted_imu; }
        else event("controller_imu_reject","SDK_return_nonzero",p.hand);
    }
    void frame(const a081_controller_frame_input &f,int fd,int radio_fd)
    {
        auto lease=std::make_shared<Lease>(fd);
        const auto a=anchor();
        a081::stock::camera::StockY8Time stock_time{};
        const bool exposure_valid=f.ctrl_exposure_ns<=1000000000ULL &&
            a081::stock::camera::stock_a081_y8_time(f.ctrl_sof_boottime_ns,f.ctrl_exposure_ns,stock_time) &&
            stock_time.exposure_start_boottime_ns==f.ctrl_exposure_start_boottime_ns;
        const unsigned expected_flags=A081_CONTROLLER_FRAME_REQUIRED & ~A081_CONTROLLER_FRAME_RAW_QVR_POSE;
        const bool phase_valid=f.ctrl_exposure_phase<3 && f.ctrl_phase_anchor_sof_qtimer &&
            f.ctrl_control_apply_sof_qtimer>=f.ctrl_phase_anchor_sof_qtimer &&
            f.ctrl_control_applied_boottime_ns && f.ctrl_control_applied_boottime_ns<f.ctrl_sof_boottime_ns;
        if (!phase_valid || !exposure_valid || !fresh(f.ctrl_exposure_start_boottime_ns,a.boot) ||
            (f.flags & ~A081_CONTROLLER_FRAME_RAW_QVR_POSE)!=expected_flags || f.view_width!=640 || f.height!=480 || f.stride_bytes!=1280 ||
            f.mosaic_bytes[0]!=614400 || f.mosaic_bytes[1]!=614400 || f.camera_clock_bracket_ns>=5000 ||
            !fresh(f.ctrl_sof_boottime_ns,a.boot) || !fresh(f.tracking_sof_boottime_ns,a.boot) ||
            (f.ctrl_sof_boottime_ns>f.tracking_sof_boottime_ns ? f.ctrl_sof_boottime_ns-f.tracking_sof_boottime_ns :
             f.tracking_sof_boottime_ns-f.ctrl_sof_boottime_ns)>=5*ms ||
            live_leases>8) {
            event("controller_frame_reject","metadata_or_capacity");return;
        }
        for (unsigned i=0;i<2;++i)
            require(u64(f.mosaic_offsets[i])+f.mosaic_bytes[i]<=lease->bytes,"mosaic outside memfd");
        // Preserve the HMD sample time. The image uses stock exposure-start
        // time, while the input retains raw QCFT SOF for independent checks.
        const bool hmd_valid=(f.flags&A081_CONTROLLER_FRAME_RAW_QVR_POSE) &&
            fresh(f.hmd_pose_boottime_ns,f.ctrl_exposure_start_boottime_ns,20*ms) &&
            finite(f.hmd_position) && finite(f.hmd_orientation_xyzw) && quaternion(f.hmd_orientation_xyzw);
        if (hmd_valid) {
            const int hmd_rc=sdk.hmd(sdk.object,f.hmd_position,f.hmd_orientation_xyzw,f.hmd_pose_boottime_ns);
            require(hmd_rc==1,"SDK HMD input return");
        }
        const unsigned phase=f.ctrl_exposure_phase;
        if (phase==2) {
            if (hmd_valid && snapshot_reference_hand_<2 && !snapshot_reference_saved_ &&
                fresh(hands[snapshot_reference_hand_].last_imu,a.boot) &&
                fresh(hands[snapshot_reference_hand_].last_clock,a.boot))
                snapshot(f,static_cast<const uint8_t *>(lease->memory),snapshot_reference_hand_,a.boot,false);
            return;
        }
        const unsigned side=phase==0 ? 1:0;
        Hand &h=hands[side];
        const auto timestamp_request=[&]() {
            if (radio_fd<0 || !h.controls.pattern_known) return;
            a081_controller_timestamp_request req{};
            req.header={A081_CONTROLLER_INPUT_MAGIC,A081_CONTROLLER_INPUT_VERSION,sizeof(req),A081_CONTROLLER_OUTPUT_TIMESTAMP_REQUEST,
                        ++timestamp_sequence,now(CLOCK_MONOTONIC)};
            req.ctrl_sof_boottime_ns=f.ctrl_sof_boottime_ns;
            req.ctrl_exposure_start_boottime_ns=f.ctrl_exposure_start_boottime_ns;
            req.ctrl_exposure_ns=f.ctrl_exposure_ns;req.ctrl_frame_id=f.ctrl_frame_id;req.hand=side;
            req.exposure_phase=f.ctrl_exposure_phase;req.phase_anchor_sof_qtimer=f.ctrl_phase_anchor_sof_qtimer;
            if (send(radio_fd,&req,sizeof(req),MSG_DONTWAIT|MSG_NOSIGNAL)!=sizeof(req))
                event("controller_timestamp_drop","radio_socket_backpressure",side);
        };
        if (!hmd_valid) {
            timestamp_request();
            ++missing_hmd_frames;
            if (missing_hmd_frames<=4 || missing_hmd_frames%75==0)
                event("controller_frame_reject","no_valid_head_pose_clock_only",side);
            return;
        }
        if (!h.controls.stock_enable || h.state!=5 || !fresh(h.last_radio,a.mono) ||
            (f.stock_hand_state[side] && f.stock_hand_state[side]!=h.state)) {
            timestamp_request();return;
        }
        auto *base=static_cast<uint8_t *>(lease->memory);
        a081_controller::BufferVector tracking{base+f.mosaic_offsets[0],base+f.mosaic_offsets[0]+f.view_width};
        a081_controller::BufferVector ctrl{base+f.mosaic_offsets[1],base+f.mosaic_offsets[1]+f.view_width};
        a081_controller::BufferVector tokens{lease.get()};
        // Captured shared ownership keeps both mmap mosaics alive through all
        // async copies/destruction of the original SDK's release functions.
        auto release_tracking=[lease, id=f.tracking_frame_id](int frame,a081_controller::BufferVector,a081_controller::BufferVector) {
            if (unsigned(frame)!=id) event("controller_release_error","tracking_frame_mismatch");
            ++callbacks;
        };
        auto release_ctrl=[lease, id=f.ctrl_frame_id](int frame,a081_controller::BufferVector,a081_controller::BufferVector) {
            if (unsigned(frame)!=id) event("controller_release_error","ctrl_frame_mismatch");
            ++callbacks;
        };
        const int rc=sdk.images(sdk.object,tracking,tokens,f.view_width,f.height,f.stride_bytes,
            f.ctrl_exposure_start_boottime_ns,int(f.tracking_frame_id),int(side),1,release_tracking,ctrl,tokens,f.ctrl_frame_id,release_ctrl);
        if (rc==0) { h.last_image=f.ctrl_exposure_start_boottime_ns;h.last_hmd=f.hmd_pose_boottime_ns;++h.images;++h.epoch_images; }
        ++h.image_attempts;
        if (detailed_logging || h.image_attempts <= 8 || h.image_attempts % 75 == 0)
        std::printf("{\"kind\":\"controller_sdk_image\",\"hand\":%u,\"frame\":%u,\"sdk_return\":%d,\"sof_ns\":%llu,\"exposure_start_ns\":%llu,\"exposure_ns\":%llu,\"hmd_sample_ns\":%llu,\"leases\":%u,\"exposure_phase\":%u}\n",
            side,f.ctrl_frame_id,rc,(unsigned long long)f.ctrl_sof_boottime_ns,(unsigned long long)f.ctrl_exposure_start_boottime_ns,(unsigned long long)f.ctrl_exposure_ns,(unsigned long long)f.hmd_pose_boottime_ns,live_leases.load(),phase);
        timestamp_request();
        if (rc==0 && !snapshot_directory_.empty() && h.epoch_images>25 &&
            snapshot_counts_[side]<3 && fresh(h.last_imu,a.boot))
            snapshot(f,base,side,a.boot);
    }
    void poses()
    {
        const auto a=anchor();
        const bool report=detailed_logging || a.mono>=next_status;
        if (report) next_status=a.mono+1000*ms;
        for (unsigned side=0;side<2;++side) {
            Hand &h=hands[side];
            if(h.last_radio && !fresh(h.last_radio,a.mono) && !h.stale) {
                invalidate(side,"imu_receipt_expired");h.stale=true;
            }
            double p[3]{},q[4]{},motion[4][3]{};u64 t=a.boot;
            const float confidence=sdk.pose(sdk.object,p,q,&t,motion[0],motion[1],motion[2],motion[3],side);
            // The stock JNI accepts initialized SDK poses at confidence >=0.1,
            // including inertial prediction between visual updates. GetPose
            // returns0 until InitEKF and after its own stale-IMU ClearState.
            const bool source=h.controls.stock_enable && h.state==5 && fresh(h.last_radio,a.mono) &&
                fresh(h.last_imu,a.boot);
            const bool accepted=source && std::isfinite(confidence) && confidence>=.1f && finite(p) && finite(q) && quaternion(q) &&
                fresh(t,a.boot) && a.uncertainty<=100000 && std::abs(a.offset-initial.offset)<=1000000;
            if (report) {
                std::printf("{\"kind\":\"controller_sdk_pose\",\"hand\":%u,\"query_boottime_ns\":%llu,\"sdk_time_ns\":%llu,\"confidence\":",
                    side,(unsigned long long)a.boot,(unsigned long long)t);
                if (std::isfinite(confidence)) std::printf("%.9g",confidence);else std::printf("null");
                std::printf(",\"position_sdk\":");array(p,3);std::printf(",\"quaternion_wxyz_sdk\":");array(q,4);
                const bool optical=accepted && confidence==1.0f;
                std::printf(",\"source_fresh\":%s,\"visual_inputs_fresh\":%s,\"sdk_pose_accepted\":%s,\"sdk_recent_visual_confidence\":%s,\"last_image_boottime_ns\":%llu,\"last_imu_boottime_ns\":%llu,\"coordinate_profile\":%u,\"position_tracking_openvr\":%s}\n",
                    source?"true":"false",fresh(h.last_image,a.boot)&&fresh(h.last_hmd,a.boot)?"true":"false",
                    accepted?"true":"false",optical?"true":"false",
                    (unsigned long long)h.last_image,(unsigned long long)h.last_imu,candidate_profile?1u:0u,
                    candidate_profile&&accepted?"true":"false");
                std::printf("{\"kind\":\"controller_status\",\"hand\":%u,\"ctrl_state\":%u,\"head_power\":%u,\"gzll_raw\":%u,\"link_present\":%s,\"tracking_enabled\":%s,\"transport_fresh\":%s,\"imu_fresh\":%s,\"epoch\":%llu}\n",
                    side,h.state,h.controls.head_power_state,h.controls.gzll_raw_state,linked(h)?"true":"false",
                    h.controls.stock_enable?"true":"false",fresh(last_transport,a.mono)?"true":"false",
                    fresh(h.last_imu,a.boot)?"true":"false",(unsigned long long)h.epoch);
            }
            if (output_fd>=0) output_pose(side,a,t,p,q,confidence,accepted);
        }
        if (report && !detailed_logging) summary();
    }
    void summary()
    {
        for (unsigned side=0;side<2;++side) std::printf("{\"kind\":\"controller_sdk_summary\",\"hand\":%u,\"imu_calls\":%llu,\"imu_rejected\":%llu,\"image_calls\":%llu,\"live_leases\":%u,\"release_callbacks\":%u}\n",
            side,(unsigned long long)hands[side].accepted_imu,(unsigned long long)hands[side].rejected_imu,
            (unsigned long long)hands[side].images,live_leases.load(),callbacks.load());
    }
private:
    static bool linked(const Hand &h)
    {
        return h.controls.pattern_known && (h.controls.head_power_state==3 || h.controls.head_power_state==4);
    }
    void snapshot(const a081_controller_frame_input &f,const uint8_t *base,unsigned side,u64 received_boot,bool submitted=true)
    {
        const auto group=submitted ? ++snapshot_counts_[side] : 1;
        if (!submitted) snapshot_reference_saved_=true;
        const auto prefix=snapshot_directory_+(submitted ? "/hand"+std::to_string(side)+"-group"+std::to_string(group) :
                                                               "/head-phase2-reference");
        for (unsigned slot=0;slot<4;++slot) {
            const auto path=prefix+"-slot"+std::to_string(slot)+".pgm";
            FILE *file=std::fopen(path.c_str(),"wb");
            if (!file) { event("controller_snapshot_error","image_open",side);return; }
            bool ok=std::fprintf(file,"P5\n640 480\n255\n")>0;
            const auto *view=base+f.mosaic_offsets[slot/2]+(slot%2)*f.view_width;
            // SDK views share a 1280-byte mosaic row. Preserve every Y8
            // sample; the second view starts 640 bytes into each row.
            for (unsigned y=0;ok && y<f.height;++y)
                ok=std::fwrite(view+y*f.stride_bytes,1,f.view_width,file)==f.view_width;
            if (std::fclose(file)) ok=false;
            if (!ok) { event("controller_snapshot_error","image_write",side);return; }
        }
        const auto &h=hands[side];
        FILE *file=std::fopen((prefix+".json").c_str(),"wb");
        if (!file) { event("controller_snapshot_error","metadata_open",side);return; }
        const int written=std::fprintf(file,
            "{\"kind\":\"controller_sdk_snapshot\",\"hand\":%u,\"clock_hand\":%u,\"group\":%u,"
            "\"submitted_to_sdk\":%s,\"purpose\":\"%s\",\"sdk_return\":%s,"
            "\"successful_images_in_epoch\":%llu,\"tracking_frame\":%u,\"ctrl_frame\":%u,"
            "\"exposure_phase\":%u,\"sensor_sequence\":%llu,\"sof_boottime_ns\":%llu,"
            "\"exposure_start_boottime_ns\":%llu,\"exposure_ns\":%llu,\"tracking_sof_boottime_ns\":%llu,"
            "\"phase_anchor_sof_qtimer\":%llu,\"control_apply_sof_qtimer\":%llu,"
            "\"control_applied_boottime_ns\":%llu,\"hmd_sample_boottime_ns\":%llu,"
            "\"last_imu_boottime_ns\":%llu,\"last_clock_boottime_ns\":%llu,"
            "\"accepted_imu_calls\":%llu,\"frame_received_boottime_ns\":%llu,\"imu_age_ns\":%llu,"
            "\"width\":640,\"height\":480,\"source_stride_bytes\":1280,\"slots\":[0,1,2,3]}\n",
            submitted ? side : 2,side,group,submitted ? "true" : "false",
            submitted ? "successful_controller_sdk_input" : "head_phase2_reference",submitted ? "0" : "null",
            (unsigned long long)h.epoch_images,f.tracking_frame_id,f.ctrl_frame_id,
            f.ctrl_exposure_phase,(unsigned long long)f.ctrl_sensor_sequence,(unsigned long long)f.ctrl_sof_boottime_ns,
            (unsigned long long)f.ctrl_exposure_start_boottime_ns,(unsigned long long)f.ctrl_exposure_ns,
            (unsigned long long)f.tracking_sof_boottime_ns,(unsigned long long)f.ctrl_phase_anchor_sof_qtimer,
            (unsigned long long)f.ctrl_control_apply_sof_qtimer,(unsigned long long)f.ctrl_control_applied_boottime_ns,
            (unsigned long long)f.hmd_pose_boottime_ns,(unsigned long long)h.last_imu,(unsigned long long)h.last_clock,
            (unsigned long long)h.accepted_imu,(unsigned long long)received_boot,
            (unsigned long long)(received_boot-h.last_imu));
        const int closed=std::fclose(file);
        if (written<0 || closed) { event("controller_snapshot_error","metadata_write",side);return; }
        if (submitted) snapshot_reference_hand_=side;
        std::printf("{\"kind\":\"controller_sdk_snapshot_saved\",\"hand\":%u,\"group\":%u,\"frame\":%u,\"exposure_phase\":%u,\"submitted_to_sdk\":%s}\n",
                    submitted ? side : 2,group,f.ctrl_frame_id,f.ctrl_exposure_phase,submitted ? "true" : "false");
    }
    void output_pose(unsigned side,const Anchor &a,u64 t,const double *p,const double *q,float confidence,bool accepted);
    Sdk sdk;int output_fd;Anchor initial;bool candidate_profile;std::array<Hand,2> hands{};u64 session{},sequence{},timestamp_sequence{},last_transport{},next_status{},missing_hmd_frames{};
    std::string snapshot_directory_;
    std::array<unsigned,2> snapshot_counts_{};
    unsigned snapshot_reference_hand_{2};
    bool snapshot_reference_saved_{};
};

void Worker::output_pose(unsigned side,const Anchor &a,u64 t,const double *p,const double *q,float confidence,bool accepted)
{
    a081_controller_pose_packet packet{};
    packet.magic=A081_CONTROLLER_POSE_MAGIC;packet.version=A081_CONTROLLER_POSE_VERSION;packet.bytes=sizeof(packet);
    packet.session_id=session;packet.sequence=++sequence;packet.hand=side;
    packet.send_monotonic_ns=now(CLOCK_MONOTONIC);
    const int64_t mono=int64_t(t)-a.offset;
    packet.sample_monotonic_ns=mono>0 ? u64(mono):0;
    packet.coordinate_profile=candidate_profile?A081_CONTROLLER_PROFILE_OPENVR_HEAD_RAW:A081_CONTROLLER_PROFILE_UNVERIFIED;
    if(linked(hands[side]) && fresh(last_transport,a.mono))packet.flags=A081_CONTROLLER_CONNECTED;
    if (accepted) packet.flags|=A081_CONTROLLER_ORIENTATION_VALID|
        A081_CONTROLLER_POSITION_VALID|A081_CONTROLLER_CLOCK_VALID|A081_CONTROLLER_SDK_POSE;
    // The pinned SDK's confidence is exactly1 only inside its recent visual
    // update window (<100ms). The wrapper's 0.1 floor is not visual evidence.
    // This diagnostic is independent of the OEM's valid predicted pose.
    const bool optical=accepted && confidence==1.0f;
    if(candidate_profile && optical)packet.flags|=A081_CONTROLLER_OPTICAL_VALID;
    packet.confidence=std::isfinite(confidence)?confidence:0;
    for(unsigned i=0;i<3;++i)packet.position[i]=std::isfinite(p[i])?float(p[i]):0;
    for(unsigned i=0;i<3;++i)packet.orientation_xyzw[i]=std::isfinite(q[i+1])?float(q[i+1]):0;
    packet.orientation_xyzw[3]=std::isfinite(q[0])?float(q[0]):0;
    if(candidate_profile) {
        // SDK HMD ingress uses D*R and D*p, D=[[0,-1,0],[-1,0,0],[0,0,-1]].
        // Head profile C=Rz(+90): C*inverse(D)=Rx(180). Preserve the native
        // controller local axes; physical grip/aim calibration is separate.
        packet.position[1]=-packet.position[1];packet.position[2]=-packet.position[2];
        packet.orientation_xyzw[0]=std::isfinite(q[0])?float(q[0]):0;
        packet.orientation_xyzw[1]=std::isfinite(q[3])?float(-q[3]):0;
        packet.orientation_xyzw[2]=std::isfinite(q[2])?float(q[2]):0;
        packet.orientation_xyzw[3]=std::isfinite(q[1])?float(-q[1]):0;
    }
    ssize_t result;
    do { result=write(output_fd,&packet,sizeof(packet)); } while(result<0 && errno==EINTR);
    require(result==sizeof(packet),"pose output pipe closed/short write");
}
} // namespace

int main(int argc,char **argv)
{
    if (argc == 2 && !std::strcmp(argv[1], "--print-abi")) {
        std::puts(A081_CONTROLLER_INPUT_ABI_DESCRIPTOR); return 0;
    }
    const char *library=nullptr,*socket_path=nullptr,*snapshot_directory=nullptr;unsigned seconds=30;int output=-1;
    bool candidate_coordinates=false;
    for (int i=1;i<argc;++i) {
        if (!std::strcmp(argv[i],"--sdk") && i+1<argc) library=argv[++i];
        else if (!std::strcmp(argv[i],"--input-socket") && i+1<argc) socket_path=argv[++i];
        else if (!std::strcmp(argv[i],"--output-fd") && i+1<argc) output=std::atoi(argv[++i]);
        else if (!std::strcmp(argv[i],"--snapshot-dir") && i+1<argc) snapshot_directory=argv[++i];
        else if (!std::strcmp(argv[i],"--candidate-profile") && i+1<argc) {
            if(std::strcmp(argv[++i],"sdk-world-to-portrait-v1"))return 64;
            candidate_coordinates=true;
        }
        else if (!std::strcmp(argv[i],"--seconds") && i+1<argc) {
            char *end=nullptr;errno=0;const auto value=std::strtoul(argv[++i],&end,10);
            if(errno || !*argv[i] || *end || value>300)return 64;
            seconds=unsigned(value);
        }
        else return 64;
    }
    if (!library || library[0]!='/' || !socket_path || socket_path[0]!='/' || seconds>300 || output==0 || output==1 || output==2) return 64;
    if (snapshot_directory && snapshot_directory[0]!='/') return 64;
    detailed_logging = seconds != 0;
    setvbuf(stdout,nullptr,_IOLBF,0);signal(SIGTERM,stopped);signal(SIGINT,stopped);signal(SIGPIPE,SIG_IGN);
    install_fault_diagnostic();
    int server=-1,status=1;std::array<Client,4> clients{};
    try {
        require(getuid()==0,"Expected root isolated worker");
        FILE *model=fopen("/proc/device-tree/model","r");require(model,"A081 model unavailable");
        char name[128]{};fread(name,1,sizeof(name)-1,model);fclose(model);
        require(!std::strcmp(name,"iQIYI XR MIX (A081)"),"Unexpected model");
        sockaddr_un addr{};addr.sun_family=AF_UNIX;require(std::strlen(socket_path)<sizeof(addr.sun_path),"socket path too long");
        std::strcpy(addr.sun_path,socket_path);
        // The supervisor treats this pathname as readiness. Original SDK
        // factory/start and the first pose query can block: complete them
        // before exposing a socket, otherwise real radio input queues until
        // its freshness deadline has already expired.
        Worker worker(library,output,candidate_coordinates,snapshot_directory);
        worker.poses();
        server=socket(AF_UNIX,SOCK_SEQPACKET|SOCK_CLOEXEC|SOCK_NONBLOCK,0);require(server>=0,"socket");
        umask(0077);require(!bind(server,reinterpret_cast<sockaddr *>(&addr),sizeof(addr)),"bind new input socket");
        require(!chmod(socket_path,0600),"input socket permissions");
        require(!listen(server,4),"listen");
        std::printf("{\"kind\":\"controller_input_ready\",\"monotonic_ns\":%llu}\n",
                    (unsigned long long)now(CLOCK_MONOTONIC));
        const auto deadline=now(CLOCK_MONOTONIC)+u64(seconds)*1000000000ULL;u64 next_pose=0;
        while (!stopping && (!seconds || now(CLOCK_MONOTONIC)<deadline)) {
            pollfd pfds[5]{{server,POLLIN,0}};
            for (unsigned i=0;i<4;++i) pfds[i+1]={clients[i].fd,POLLIN,0};
            const int ready=poll(pfds,5,2);if (ready<0 && errno==EINTR)continue;require(ready>=0,"poll");
            if (pfds[0].revents&POLLIN) {
                int fd=accept4(server,nullptr,nullptr,SOCK_CLOEXEC|SOCK_NONBLOCK);
                if (fd>=0) {
                    ucred peer{};socklen_t size=sizeof(peer);bool admitted=false;
                    if (!getsockopt(fd,SOL_SOCKET,SO_PEERCRED,&peer,&size) && peer.uid==0) {
                        for (auto &c:clients) if (c.fd<0) { c={fd,0,0};admitted=true;break; }
                    }
                    if (!admitted)close(fd);
                }
            }
            for (unsigned i=0;i<4;++i) {
                auto &c=clients[i];if (c.fd<0 || !(pfds[i+1].revents&(POLLIN|POLLHUP|POLLERR))) continue;
                alignas(8) std::array<unsigned char,sizeof(a081_controller_frame_input)> bytes{};alignas(cmsghdr) char ancillary[CMSG_SPACE(4*sizeof(int))]{};
                iovec iov{bytes.data(),bytes.size()};msghdr msg{};msg.msg_iov=&iov;msg.msg_iovlen=1;msg.msg_control=ancillary;msg.msg_controllen=sizeof(ancillary);
                const ssize_t count=recvmsg(c.fd,&msg,MSG_DONTWAIT|MSG_CMSG_CLOEXEC);
                const int receive_error=count<0?errno:0;
                if (count<0 && (errno==EAGAIN || errno==EINTR))continue;
                std::array<int,4> fds{{-1,-1,-1,-1}};unsigned nfds=0;
                for (auto *cm=CMSG_FIRSTHDR(&msg);cm;cm=CMSG_NXTHDR(&msg,cm)) {
                    if (cm->cmsg_level!=SOL_SOCKET || cm->cmsg_type!=SCM_RIGHTS)continue;
                    size_t n=(cm->cmsg_len-CMSG_LEN(0))/sizeof(int);auto *received=reinterpret_cast<int *>(CMSG_DATA(cm));
                    for(size_t j=0;j<n;++j) { if(nfds<4)fds[nfds++]=received[j];else close(received[j]); }
                }
                const auto *h=reinterpret_cast<const a081_controller_input_header *>(bytes.data());
                const auto received_at=now(CLOCK_MONOTONIC);
                bool valid=count>=ssize_t(sizeof(*h)) && !(msg.msg_flags&(MSG_TRUNC|MSG_CTRUNC)) && h->magic==A081_CONTROLLER_INPUT_MAGIC &&
                    h->version==A081_CONTROLLER_INPUT_VERSION && h->bytes==uint32_t(count) && h->sequence>c.sequence && fresh(h->sent_monotonic_ns,received_at) &&
                    (!c.type || c.type==h->type);
                if (valid && h->type==A081_CONTROLLER_INPUT_RADIO && count==88 && !nfds) {
                    c.sequence=h->sequence;c.type=h->type;worker.radio(*reinterpret_cast<const a081_controller_radio_input *>(bytes.data()));
                } else if (valid && h->type==A081_CONTROLLER_INPUT_FRAME && count==ssize_t(sizeof(a081_controller_frame_input)) && nfds==1) {
                    c.sequence=h->sequence;c.type=h->type;int radio_fd=-1;
                    for(auto &peer:clients)if(peer.type==A081_CONTROLLER_INPUT_RADIO)radio_fd=peer.fd;
                    int image_fd=fds[0];fds[0]=-1;
                    worker.frame(*reinterpret_cast<const a081_controller_frame_input *>(bytes.data()),image_fd,radio_fd);
                } else {
                    if(count!=0)std::printf("{\"kind\":\"controller_input_reject\",\"count\":%d,\"errno\":%d,\"msg_flags\":%d,\"fds\":%u,\"magic\":%u,\"version\":%u,\"bytes\":%u,\"type\":%u,\"previous_type\":%u,\"sequence\":%llu,\"previous_sequence\":%llu,\"sent_monotonic_ns\":%llu,\"received_monotonic_ns\":%llu,\"header_age_ns\":%lld,\"header_valid\":%s}\n",
                        int(count),receive_error,msg.msg_flags,nfds,h->magic,h->version,h->bytes,h->type,c.type,
                        (unsigned long long)h->sequence,(unsigned long long)c.sequence,
                        (unsigned long long)h->sent_monotonic_ns,(unsigned long long)received_at,
                        (long long)(int64_t(received_at)-int64_t(h->sent_monotonic_ns)),valid?"true":"false");
                    if(c.type)worker.source_closed(c.type);
                    close(c.fd);c={};event("controller_input_closed",count==0?"source_eof":"invalid_packet");
                }
                for(int fd:fds)if(fd>=0)close(fd);
            }
            const auto t=now(CLOCK_MONOTONIC);
            if(t>=next_pose){worker.poses();next_pose=t+20*ms;}
        }
        worker.source_closed(A081_CONTROLLER_INPUT_RADIO);
        worker.poses();worker.summary();status=0;
    } catch(const std::exception &e) {
        std::fprintf(stderr,"controller-sdk-pose: %s (errno=%d)\n",e.what(),errno);
    }
    for(auto &c:clients)if(c.fd>=0)close(c.fd);
    if(server>=0)close(server);
    // Never unload the original singleton while its worker threads/callbacks
    // are live; process exit owns SDK cleanup. Explicitly report live leases.
    std::fflush(nullptr);_exit(status);
}
