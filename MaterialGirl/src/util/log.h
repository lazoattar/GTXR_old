#ifndef LOG_H
#define LOG_H

#include <stdbool.h>

#define LOG_LEVEL_FATAL 0
#define LOG_LEVEL_ERROR 1
#define LOG_LEVEL_WARN  2
#define LOG_LEVEL_INFO  3

#ifndef LOG_LEVEL
#define LOG_LEVEL LOG_LEVEL_INFO
#endif /* LOG_LEVEL */

void log_message(int level, bool newline, const char *format, ...);

#if LOG_LEVEL >= LOG_LEVEL_FATAL
#define LOG_FATAL(format, ...) log_message(LOG_LEVEL_FATAL, true, format, ##__VA_ARGS__)
#define _LOG_FATAL(format, ...) log_message(LOG_LEVEL_FATAL, false, format, ##__VA_ARGS__)
#else
#define LOG_FATAL(format, ...)
#define _LOG_FATAL(format, ...)
#endif

#if LOG_LEVEL >= LOG_LEVEL_ERROR
#define LOG_ERROR(format, ...) log_message(LOG_LEVEL_ERROR, true, format, ##__VA_ARGS__)
#define _LOG_ERROR(format, ...) log_message(LOG_LEVEL_ERROR, false, format, ##__VA_ARGS__)
#else
#define LOG_ERROR(format, ...)
#define _LOG_ERROR(format, ...)
#endif

#if LOG_LEVEL >= LOG_LEVEL_WARN
#define LOG_WARN(format, ...) log_message(LOG_LEVEL_WARN, true, format, ##__VA_ARGS__)
#define _LOG_WARN(format, ...) log_message(LOG_LEVEL_WARN, false, format, ##__VA_ARGS__)
#else
#define LOG_WARN(format, ...)
#define _LOG_WARN(format, ...)
#endif

#if LOG_LEVEL >= LOG_LEVEL_INFO
#define LOG_INFO(format, ...) log_message(LOG_LEVEL_INFO, true, format, ##__VA_ARGS__)
#define _LOG_INFO(format, ...) log_message(LOG_LEVEL_INFO, false, format, ##__VA_ARGS__)
#else
#define LOG_INFO(format, ...)
#define _LOG_INFO(format, ...)
#endif

#endif /* LOG_H */