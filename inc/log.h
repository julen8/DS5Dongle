#pragma once

#include <cstdio>

#define ENABLE_DEBUG 0
#define ENABLE_INFO 0

#define DUMP_FMT_(func_name, fmt, ...) "%s:%d:%s: " fmt, __FILE_NAME__, __LINE__, func_name, ##__VA_ARGS__

#define TRACE_FUNC_NAME_(priority, func_name, fmt, ...) printf(DUMP_FMT_(func_name, #priority ": " fmt "\n", ##__VA_ARGS__))

#if ENABLE_DEBUG
#    define LOGD_FUNC_NAME(func_name, fmt, ...) TRACE_FUNC_NAME_(DEBUG, func_name, fmt, ##__VA_ARGS__)
#else
#    define LOGD_FUNC_NAME(func_name, fmt, ...) ((void)0)
#endif

#if ENABLE_INFO
#    define LOGI_FUNC_NAME(func_name, fmt, ...) TRACE_FUNC_NAME_(INFO, func_name, fmt, ##__VA_ARGS__)
#else
#    define LOGI_FUNC_NAME(func_name, fmt, ...) ((void)0)
#endif

#define LOGW_FUNC_NAME(func_name, fmt, ...) TRACE_FUNC_NAME_(WARNING, func_name, fmt, ##__VA_ARGS__)
#define LOGE_FUNC_NAME(func_name, fmt, ...) TRACE_FUNC_NAME_(ERROR, func_name, fmt, ##__VA_ARGS__)

#define LOGD(fmt, ...) LOGD_FUNC_NAME(__func__, fmt, ##__VA_ARGS__)
#define LOGI(fmt, ...) LOGI_FUNC_NAME(__func__, fmt, ##__VA_ARGS__)
#define LOGW(fmt, ...) LOGW_FUNC_NAME(__func__, fmt, ##__VA_ARGS__)
#define LOGE(fmt, ...) LOGE_FUNC_NAME(__func__, fmt, ##__VA_ARGS__)

#if ENABLE_DEBUG
inline void printHex(const uint8_t *data, size_t size) {
    for (size_t i = 0; i < size; i++) {
        printf("%02x ", static_cast<unsigned>(data[i]));
    }
    printf("\n");
}
#else
#    define printHex(data, size) ((void)0)
#endif