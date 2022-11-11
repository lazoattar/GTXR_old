#include "log.h"

#include <stdio.h>
#include <stdint.h>
#include <string.h>

extern void USART3_DMA1_Stream3_Write(char *data, uint16_t length);

void log_message(int level, const char *format, bool newline, ...) {
    int len = strlen(format);
    // Add 2 for (potential) newline and null character
    // Add 11 for log level
     
    // Log levels:
    // [FATAL]: 
    // [ERROR]:
    // [WARNING]: 
    // [INFO]: 

    int len_padded = len * 2 + 2 + 11;

    char buffer[len_padded];

    int write_len = 0;

    va_list args;
    va_start(args, format);
    write_len = sprintf(buffer, format, args);
    va_end(args);

    if (newline == true) {
        buffer[write_len] = '\n';
        write_len++;
        buffer[write_len] = '\0';
    }

    switch (level) {
        case LOG_LEVEL_FATAL:
            sprintf(buffer, "%s%s", "[FATAL]: ", buffer);
            break;
        case LOG_LEVEL_ERROR:
            sprintf(buffer, "%s%s", "[ERROR]: ", buffer);
            break;
        case LOG_LEVEL_WARN:
            sprintf(buffer, "%s%s", "[WARNING]: ", buffer);
            break;
        case LOG_LEVEL_INFO:
            sprintf(buffer, "%s%s", "[INFO]: ", buffer);
            break;
        default:
            break;
    }

    USART3_DMA1_Stream3_Write(buffer, write_len);
}