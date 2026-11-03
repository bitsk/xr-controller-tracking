#define _GNU_SOURCE
#include <xr/device/a081_controller_abi.h>

#include <dlfcn.h>
#include <errno.h>
#include <inttypes.h>
#include <link.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#if !defined(__ANDROID__) || !defined(__arm__)
#error "Build this probe for Android ARM32 with the NDK; never run it via emulation."
#endif
_Static_assert(sizeof(void *) == 4, "SDK requires 32-bit pointers");
_Static_assert(sizeof(double) == 8, "SDK requires 64-bit double");

static void quote(const char *text)
{
    putchar('"');
    for (const unsigned char *p = (const unsigned char *)text; *p; ++p) {
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

static void result(const char *stage, const char *status, const char *detail)
{
    printf("{\"stage\":");
    quote(stage);
    printf(",\"status\":");
    quote(status);
    if (detail) {
        printf(",\"detail\":");
        quote(detail);
    }
    puts("}");
}

static void expired(int signal_number)
{
    static const char message[] =
        "{\"stage\":\"deadline\",\"status\":\"timeout\"}\n";
    (void)signal_number;
    (void)write(STDERR_FILENO, message, sizeof(message) - 1);
    _exit(124);
}

struct build_id_request {
    uintptr_t base;
    char hex[65];
};

static int find_build_id(struct dl_phdr_info *info, size_t size, void *opaque)
{
    struct build_id_request *request = opaque;
    (void)size;
    if ((uintptr_t)info->dlpi_addr != request->base)
        return 0;
    for (ElfW(Half) i = 0; i < info->dlpi_phnum; ++i) {
        const ElfW(Phdr) *phdr = &info->dlpi_phdr[i];
        if (phdr->p_type != PT_NOTE)
            continue;
        const unsigned char *p = (const unsigned char *)(
            info->dlpi_addr + phdr->p_vaddr);
        const unsigned char *end = p + phdr->p_memsz;
        while ((size_t)(end - p) >= sizeof(ElfW(Nhdr))) {
            ElfW(Nhdr) note;
            memcpy(&note, p, sizeof(note));
            p += sizeof(note);
            size_t namesz = ((size_t)note.n_namesz + 3) & ~(size_t)3;
            size_t descsz = ((size_t)note.n_descsz + 3) & ~(size_t)3;
            if (namesz > (size_t)(end - p) ||
                descsz > (size_t)(end - p) - namesz)
                break;
            if (note.n_type == NT_GNU_BUILD_ID && note.n_namesz == 4 &&
                memcmp(p, "GNU", 4) == 0 && note.n_descsz <= 32) {
                const unsigned char *value = p + namesz;
                for (uint32_t j = 0; j < note.n_descsz; ++j)
                    snprintf(request->hex + j * 2, 3, "%02x", value[j]);
                return 1;
            }
            p += namesz + descsz;
        }
    }
    return 1;
}

static int check_vtable(void *object, uintptr_t base)
{
    const uintptr_t *vtable;
    memcpy(&vtable, object, sizeof(vtable));
    if ((uintptr_t)vtable != base + A081_CONTROLLER_VTABLE_OFFSET) {
        result("vtable", "mismatch", "Unexpected SDK class/vtable");
        return 0;
    }
    static const struct {
        unsigned slot;
        uintptr_t elf_offset;
    } expected[] = {
        {A081_SLOT_INITIAL_MODE, 0x38451c},
        {A081_SLOT_IMU_INPUT, 0x384df8},
        {A081_SLOT_IMAGE_INPUT, 0x3850d8},
        {A081_SLOT_POSE_LISTENER, 0x385ab0},
        {A081_SLOT_GET_POSE, 0x3865b0},
        {A081_SLOT_RIGHT_IMU, 0x386efe},
        {A081_SLOT_RIGHT_IMAGE, 0x386f20},
        {A081_SLOT_SCHEDULER, 0x3873d8},
        {A081_SLOT_VSYNC, 0x387458}
    };
    for (size_t i = 0; i < sizeof(expected) / sizeof(expected[0]); ++i) {
        uintptr_t pointer = vtable[expected[i].slot / sizeof(uintptr_t)];
        uintptr_t offset = (pointer & ~(uintptr_t)1) - base;
        printf("{\"stage\":\"vtable_slot\",\"slot\":%u,"
               "\"elf_offset\":\"0x%" PRIxPTR "\",\"thumb\":%s}\n",
               expected[i].slot, offset, pointer & 1 ? "true" : "false");
        if (offset != expected[i].elf_offset || !(pointer & 1)) {
            result("vtable", "mismatch", "Unexpected method entry");
            return 0;
        }
    }
    result("vtable", "ok", "Pinned SDK entries match; no methods invoked");
    return 1;
}

static int snpe_dsp_probe(void *library)
{
    static const char *const names[] = {
        "_ZN3zdl4SNPE11SNPEFactory18isRuntimeAvailableENS_8DlSystem9Runtime_tE",
        "_ZN3zdl8DlSystem16getLastErrorCodeEv",
        "_ZN3zdl8DlSystem18getLastErrorStringEv"
    };
    void *symbols[3];
    for (size_t i = 0; i < 3; ++i) {
        dlerror();
        symbols[i] = dlsym(library, names[i]);
        const char *error = dlerror();
        if (error || !symbols[i]) {
            result("snpe_symbol", "error", error ? error : names[i]);
            return 66;
        }
    }
    bool (*available)(int);
    int (*last_error_code)(void);
    const char *(*last_error_string)(void);
    memcpy(&available, &symbols[0], sizeof(available));
    memcpy(&last_error_code, &symbols[1], sizeof(last_error_code));
    memcpy(&last_error_string, &symbols[2], sizeof(last_error_string));
    const bool supported = available(2);
    const int code = last_error_code();
    const char *detail = last_error_string();
    printf("{\"stage\":\"snpe_runtime\",\"runtime\":2,\"available\":%s,"
           "\"last_error_code\":%d,\"last_error_string\":", supported ? "true" : "false", code);
    quote(detail ? detail : "");
    puts("}");
    return supported ? 0 : 1;
}

int main(int argc, char **argv)
{
    const char *mode = argc > 1 ? argv[1] : "";
    int create = strcmp(mode, "--create") == 0;
    int load = strcmp(mode, "--load") == 0;
    int exec_only = strcmp(mode, "--exec") == 0;
    int snpe_dsp = strcmp(mode, "--snpe-dsp") == 0;
    if ((!create && !load && !exec_only && !snpe_dsp) ||
        (exec_only ? argc != 2 : argc < 3 || argc > 4)) {
        fprintf(stderr, "usage: %s --exec\n"
                "       %s --load|--create /absolute/path/libControllerSDK_EXPORT.so"
                " [timeout-seconds:1..120]\n"
                "       %s --snpe-dsp /absolute/path/libSNPE.so"
                " [timeout-seconds:1..120]\n", argv[0], argv[0], argv[0]);
        return 64;
    }
    unsigned timeout_seconds = 20;
    if (argc == 4) {
        char *end;
        errno = 0;
        unsigned long value = strtoul(argv[3], &end, 10);
        if (errno || !*argv[3] || *end || value < 1 || value > 120)
            return 64;
        timeout_seconds = (unsigned)value;
    }
    setvbuf(stdout, NULL, _IONBF, 0);
    result("exec", "ok", "Android ARM32 Bionic process entered main");
    if (exec_only)
        _exit(0);
    if (argv[2][0] != '/') {
        result("arguments", "error", "SDK path must be absolute in the helper root");
        _exit(64);
    }
    struct sigaction action = {0};
    action.sa_handler = expired;
    sigemptyset(&action.sa_mask);
    if (sigaction(SIGALRM, &action, NULL) < 0) {
        result("deadline", "error", strerror(errno));
        _exit(70);
    }
    alarm(timeout_seconds);
    result("dlopen", "begin", argv[2]);
    void *library = dlopen(argv[2], RTLD_NOW | RTLD_LOCAL);
    if (!library) {
        result("dlopen", "error", dlerror());
        _exit(65);
    }
    result("dlopen", "ok", NULL);
    if (snpe_dsp)
        _exit(snpe_dsp_probe(library));
    dlerror();
    void *symbol = dlsym(library, A081_CONTROLLER_FACTORY_SYMBOL);
    const char *error = dlerror();
    if (error || !symbol) {
        result("factory_symbol", "error", error ? error : "NULL symbol");
        _exit(66);
    }
    Dl_info location;
    if (!dladdr(symbol, &location)) {
        result("factory_symbol", "error", "dladdr failed");
        _exit(66);
    }
    uintptr_t base = (uintptr_t)location.dli_fbase;
    uintptr_t factory_offset = ((uintptr_t)symbol & ~(uintptr_t)1) - base;
    struct build_id_request request = {.base = base, .hex = {0}};
    dl_iterate_phdr(find_build_id, &request);
    result("build_id", "observed", request.hex);
    if (strcmp(request.hex, A081_CONTROLLER_SDK_BUILD_ID) ||
        factory_offset != A081_CONTROLLER_FACTORY_OFFSET ||
        !((uintptr_t)symbol & 1)) {
        result("identity", "error", "SDK build-id or factory entry differs from reference");
        _exit(67);
    }
    result("identity", "ok", A081_CONTROLLER_SDK_BUILD_ID);
    if (!create) {
        result("complete", "ok", "Library loaded; factory not called");
        _exit(0);
    }
    a081_controller_factory_fn factory;
    _Static_assert(sizeof(factory) == sizeof(symbol), "ARM pointer ABI mismatch");
    memcpy(&factory, &symbol, sizeof(factory));
    result("create", "begin", "getInstance(HAND=2, mode=2)");
    void *object = factory(A081_HAND_BOTH, 2);
    if (!object) {
        result("create", "error", "Factory returned NULL");
        _exit(68);
    }
    result("create", "ok", "Factory returned a service object");
    if (!check_vtable(object, base))
        _exit(69);
    result("complete", "ok", "Factory and vtable only; no tracking result claimed");
    /* Factory is a singleton with worker threads. Do not run unverified SDK
     * destructors, delete it, or unload its code while its threads may run.
     */
    _exit(0);
}
