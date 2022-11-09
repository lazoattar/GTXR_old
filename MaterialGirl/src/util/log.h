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

void log_message(int level, const char *format, bool newline, ...);

#if LOG_LEVEL >= LOG_LEVEL_FATAL
#define LOG_FATAL(str, format, ...) log_message(LOG_LEVEL_FATAL, format, true, __VA_ARGS__)
#define _LOG_FATAL(str, format, ...) log_message(LOG_LEVEL_FATAL, format, false, __VA_ARGS__)
#else
#define LOG_FATAL(...)
#define _LOG_FATAL(...)
#endif

#if LOG_LEVEL >= LOG_LEVEL_ERROR
#define LOG_ERROR(...) log_message(LOG_LEVEL_ERROR, format, true, __VA_ARGS__)
#define _LOG_ERROR(...) log_message(LOG_LEVEL_ERROR,  format, false, __VA_ARGS__)
#else
#define LOG_ERROR(...)
#define _LOG_ERROR(...)
#endif

#if LOG_LEVEL >= LOG_LEVEL_WARN
#define LOG_WARN(...) log_message(LOG_LEVEL_WARN, format, true, __VA_ARGS__)
#define _LOG_WARN(...) log_message(LOG_LEVEL_WARN, format, false, __VA_ARGS__)
#else
#define LOG_WARN(...)
#define _LOG_WARN(...)
#endif

#if LOG_LEVEL >= LOG_LEVEL_INFO
#define LOG_INFO(...) log_message(LOG_LEVEL_INFO, format, true, __VA_ARGS__)
#define _LOG_INFO(...) log_message(LOG_LEVEL_INFO, format, false, __VA_ARGS__)
#else
#define LOG_INFO(...)
#define _LOG_INFO(...)
#endif

#endif /* LOG_H */