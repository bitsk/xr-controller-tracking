// SPDX-License-Identifier: GPL-3.0-or-later
// Bounded observation only: keep the real liblog calls and their return values.
#include <android/log.h>
#include <dlfcn.h>
#include <errno.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#ifndef A081_LOG_SOURCE_SHA
#define A081_LOG_SOURCE_SHA "unspecified"
#endif

namespace {
using VPrint = int (*)(int, const char *, const char *, va_list);
using Write = int (*)(int, const char *, const char *);
VPrint original_vprint;
Write original_write;
thread_local bool forwarding;
pthread_mutex_t capture_mutex = PTHREAD_MUTEX_INITIALIZER;
constexpr unsigned limit_per_key = 12;
struct Entry { uint64_t key; unsigned count; } entries[512]{};

uint64_t hash_text(uint64_t value, const char *text)
{
    if (text) for (; *text; ++text) value = (value ^ uint8_t(*text)) * 1099511628211ULL;
    return (value ^ 255U) * 1099511628211ULL;
}

// Generic "%s" and write() callers otherwise collapse every error into one
// format. Use a bounded numeric-normalized prefix to retain distinct messages.
void message_class(const char *text, char (&out)[161])
{
    unsigned n = 0;
    while (*text && n < sizeof(out) - 1) {
        const char c = *text;
        if ((c >= '0' && c <= '9') ||
            ((c == '-' || c == '+') && text[1] >= '0' && text[1] <= '9')) {
            out[n++] = '#';
            ++text;
            while ((*text >= '0' && *text <= '9') || *text == '.' || *text == 'x' ||
                   *text == 'X' || (*text >= 'a' && *text <= 'f') ||
                   (*text >= 'A' && *text <= 'F') || *text == '+' || *text == '-') ++text;
        } else {
            out[n++] = *text++;
        }
    }
    out[n] = 0;
}

void append_json(char *line, size_t &used, size_t capacity, const char *text, bool &truncated)
{
    if (!text) text = "";
    for (; *text; ++text) {
        const unsigned char c = *text;
        char escaped[7];
        size_t n = 1;
        escaped[0] = char(c);
        if (c == '"' || c == '\\') { escaped[0] = '\\'; escaped[1] = char(c); n = 2; }
        else if (c < 32 || c >= 127) { snprintf(escaped, sizeof(escaped), "\\u%04x", c); n = 6; }
        if (used + n >= capacity) { truncated = true; break; }
        memcpy(line + used, escaped, n);
        used += n;
    }
    line[used] = 0;
}

void capture(int priority, const char *tag, const char *format, const char *message, bool truncated)
{
    char normalized[161];
    message_class(message, normalized);
    uint64_t key = hash_text(1469598103934665603ULL, tag);
    key = hash_text(key, format);
    // For ordinary printf formats, the format itself is already a stable key.
    if (!format || !strcmp(format, "%s") || !strcmp(format, "%s\n"))
        key = hash_text(key, normalized);
    if (!key) key = 1;
    pthread_mutex_lock(&capture_mutex);
    Entry *entry = nullptr;
    for (auto &candidate : entries) {
        if (candidate.key == key || !candidate.key) { entry = &candidate; break; }
    }
    if (!entry || entry->count > limit_per_key) { pthread_mutex_unlock(&capture_mutex); return; }
    entry->key = key;
    const unsigned occurrence = ++entry->count;
    char line[4096];
    size_t used = size_t(snprintf(line, sizeof(line),
        "{\"kind\":\"controller_sdk_android_log\",\"priority\":%d,\"source_key\":\"%016llx\","
        "\"occurrence\":%u,\"repeat_suppression\":%s,\"tag\":\"", priority,
        static_cast<unsigned long long>(key), occurrence, occurrence > limit_per_key ? "true" : "false"));
    append_json(line, used, 900, tag, truncated);
    used += size_t(snprintf(line + used, sizeof(line) - used, "\",\"message\":\""));
    append_json(line, used, sizeof(line) - 64,
                occurrence > limit_per_key ? "Further occurrences of this source/format are suppressed" : message,
                truncated);
    used += size_t(snprintf(line + used, sizeof(line) - used, "\",\"truncated\":%s}\n", truncated ? "true" : "false"));
    (void)write(STDERR_FILENO, line, used);
    pthread_mutex_unlock(&capture_mutex);
}

__attribute__((constructor)) void initialize()
{
    original_vprint = reinterpret_cast<VPrint>(dlsym(RTLD_NEXT, "__android_log_vprint"));
    original_write = reinterpret_cast<Write>(dlsym(RTLD_NEXT, "__android_log_write"));
    const char *line = original_vprint && original_write ?
        "{\"kind\":\"controller_sdk_android_log_ready\",\"original_functions_resolved\":true,"
        "\"source_sha256\":\"" A081_LOG_SOURCE_SHA "\",\"limit_per_key\":12,\"key_capacity\":512}\n" :
        "{\"kind\":\"controller_sdk_android_log_ready\",\"original_functions_resolved\":false}\n";
    (void)write(STDERR_FILENO, line, strlen(line));
}

int forward_vprint(int priority, const char *tag, const char *format, va_list args)
{
    if (!original_vprint) { errno = ENOSYS; return -ENOSYS; }
    if (forwarding) return original_vprint(priority, tag, format, args);
    forwarding = true;
    const int incoming_errno = errno;
    char message[1536];
    va_list copied;
    va_copy(copied, args);
    const int length = vsnprintf(message, sizeof(message), format ? format : "", copied);
    va_end(copied);
    capture(priority, tag, format, message, length < 0 || size_t(length) >= sizeof(message));
    errno = incoming_errno;
    const int result = original_vprint(priority, tag, format, args);
    const int original_errno = errno;
    forwarding = false;
    errno = original_errno;
    return result;
}
}

extern "C" int __android_log_vprint(int priority, const char *tag, const char *format, va_list args)
{
    return forward_vprint(priority, tag, format, args);
}

extern "C" int __android_log_print(int priority, const char *tag, const char *format, ...)
{
    va_list args;
    va_start(args, format);
    const int result = forward_vprint(priority, tag, format, args);
    va_end(args);
    return result;
}

extern "C" int __android_log_write(int priority, const char *tag, const char *message)
{
    if (!original_write) { errno = ENOSYS; return -ENOSYS; }
    if (forwarding) return original_write(priority, tag, message);
    forwarding = true;
    const int incoming_errno = errno;
    capture(priority, tag, nullptr, message ? message : "", false);
    errno = incoming_errno;
    const int result = original_write(priority, tag, message);
    const int original_errno = errno;
    forwarding = false;
    errno = original_errno;
    return result;
}
