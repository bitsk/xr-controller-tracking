// SPDX-License-Identifier: BSL-1.0
// Read-only native OpenVR observation. No compositor or controller commands.
#include <openvr.h>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <time.h>
#include <unistd.h>
#if !defined(__aarch64__)
#error This observer must target native AArch64 Linux.
#endif
static bool identity(const char *path, const char *expected)
{
    FILE *f = fopen(path, "rb"); if (!f) return false;
    char text[256]{}; size_t n = fread(text, 1, sizeof(text), f);
    bool ok = !ferror(f) && n < sizeof(text); fclose(f);
    while (n && (text[n-1] == '\0' || text[n-1] == '\n')) --n;
    return ok && n == strlen(expected) && !memcmp(text, expected, n);
}
static int64_t now_ns()
{
    timespec t{}; if (clock_gettime(CLOCK_MONOTONIC, &t)) return -1;
    return int64_t(t.tv_sec) * 1000000000 + t.tv_nsec;
}
int main(int argc, char **argv)
{
    setvbuf(stdout, nullptr, _IOLBF, 0);
    const char *registry = getenv("VR_PATHREG_OVERRIDE");
    char *end = nullptr;
    long seconds = argc == 3 ? strtol(argv[2], &end, 10) : 0;
    if (argc != 3 || !end || *end || seconds < 1 || seconds > 600 || argv[1][0] != '/' ||
        !registry || registry[0] != '/' || getuid() != 1000 || geteuid() != 1000 ||
        !identity("/etc/a081-rootfs-id", "a081-steamos-arm64") ||
        !identity("/proc/device-tree/model", "iQIYI Qiyu MIX (A081)")) {
        fputs("Expected UID1000 A081 SteamOS, private VR_PATHREG_OVERRIDE, absolute libopenvr_api.so and seconds1..600\n", stderr);
        return 22;
    }
    alarm(unsigned(seconds) + 15);
    void *lib = dlopen(argv[1], RTLD_NOW | RTLD_LOCAL);
    if (!lib) { fprintf(stderr, "%s\n", dlerror()); return 1; }
    auto init = reinterpret_cast<uint32_t (*)(vr::EVRInitError *, vr::EVRApplicationType, const char *)>(dlsym(lib, "VR_InitInternal2"));
    auto get = reinterpret_cast<void *(*)(const char *, vr::EVRInitError *)>(dlsym(lib, "VR_GetGenericInterface"));
    auto shutdown = reinterpret_cast<void (*)()>(dlsym(lib, "VR_ShutdownInternal"));
    if (!init || !get || !shutdown) { dlclose(lib); return 2; }
    vr::EVRInitError error = vr::VRInitError_None;
    uint32_t token = init(&error, vr::VRApplication_Background, nullptr);
    printf("{\"kind\":\"openvr_init\",\"token\":%u,\"error\":%d}\n", token, error);
    if (!token || error) { dlclose(lib); return 3; }
    auto finish = [&](int rc) { shutdown(); dlclose(lib); alarm(0); return rc; };
    auto *system = static_cast<vr::IVRSystem *>(get(vr::IVRSystem_Version, &error));
    if (!system || error) return finish(4);
    const char *serials[2] = {"A081-STOCK-CONTROLLER-L", "A081-STOCK-CONTROLLER-R"};
    unsigned counts[2]{}, valid_counts[2]{}, missing_counts[2]{}, hmd_valid = 0;
    vr::TrackedDeviceIndex_t found[2] = {vr::k_unTrackedDeviceIndexInvalid, vr::k_unTrackedDeviceIndexInvalid};
    const int64_t start = now_ns();
    for (unsigned frame = 0; frame < unsigned(seconds) * 100; ++frame) {
        if (frame % 100 == 0) {
            found[0] = found[1] = vr::k_unTrackedDeviceIndexInvalid;
            for (uint32_t id = 1; id < vr::k_unMaxTrackedDeviceCount; ++id) {
                if (system->GetTrackedDeviceClass(id) != vr::TrackedDeviceClass_Controller) continue;
                char serial[128]{}; vr::ETrackedPropertyError pe = vr::TrackedProp_Success;
                uint32_t n = system->GetStringTrackedDeviceProperty(id, vr::Prop_SerialNumber_String, serial, sizeof(serial), &pe);
                if (pe || !n || n > sizeof(serial)) continue;
                for (unsigned hand = 0; hand < 2; ++hand) if (!strcmp(serial, serials[hand])) found[hand] = id;
            }
        }
        vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount]{};
        system->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseRawAndUncalibrated, 0, poses, vr::k_unMaxTrackedDeviceCount);
        int64_t now = now_ns();
        hmd_valid += poses[0].bDeviceIsConnected && poses[0].bPoseIsValid && poses[0].eTrackingResult == vr::TrackingResult_Running_OK;
        for (unsigned hand = 0; hand < 2; ++hand) {
            uint32_t id = found[hand];
            if (id == vr::k_unTrackedDeviceIndexInvalid) {
                ++missing_counts[hand];
                printf("{\"kind\":\"controller_pose\",\"monotonic_ns\":%lld,\"hand\":%u,\"present\":false,\"valid\":false}\n", (long long)now, hand);
                continue;
            }
            const auto &p = poses[id];
            bool finite = true;
            for (auto &row : p.mDeviceToAbsoluteTracking.m) for (float v : row) finite &= std::isfinite(v);
            bool role = system->GetControllerRoleForTrackedDeviceIndex(id) ==
                        (hand ? vr::TrackedControllerRole_RightHand : vr::TrackedControllerRole_LeftHand);
            bool valid = p.bDeviceIsConnected && p.bPoseIsValid && p.eTrackingResult == vr::TrackingResult_Running_OK && finite && role;
            ++counts[hand]; valid_counts[hand] += valid;
            printf("{\"kind\":\"controller_pose\",\"monotonic_ns\":%lld,\"hand\":%u,\"device\":%u,\"present\":true,\"connected\":%s,\"valid\":%s,\"tracking\":%d,\"finite\":%s,\"correct_role\":%s,\"matrix\":[",
                   (long long)now, hand, id, p.bDeviceIsConnected ? "true" : "false", valid ? "true" : "false", p.eTrackingResult,
                   finite ? "true" : "false", role ? "true" : "false");
            for (unsigned row = 0; row < 3; ++row) for (unsigned col = 0; col < 4; ++col) {
                if (row || col) putchar(',');
                float v = p.mDeviceToAbsoluteTracking.m[row][col];
                if (std::isfinite(v)) printf("%.9g", v); else fputs("null", stdout);
            }
            puts("]}");
        }
        int64_t target = start + (int64_t(frame) + 1) * 10000000;
        timespec deadline{time_t(target / 1000000000), long(target % 1000000000)};
        int rc; do { rc = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &deadline, nullptr); } while (rc == EINTR);
        if (rc || now < start) return finish(5);
    }
    printf("{\"kind\":\"controller_observation_end\",\"observed_seconds\":%.9f,\"samples_per_hand\":%ld,\"present\":[%u,%u],\"valid\":[%u,%u],\"missing\":[%u,%u],\"hmd_valid\":%u,\"physical_accuracy_verified\":false,\"observation_complete\":true}\n",
           double(now_ns()-start)/1e9, seconds*100, counts[0], counts[1], valid_counts[0], valid_counts[1], missing_counts[0], missing_counts[1], hmd_valid);
    return finish(0); // Observation completion is not controller acceptance.
}
