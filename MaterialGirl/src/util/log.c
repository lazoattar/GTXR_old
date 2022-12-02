#include "log.h"

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdarg.h>

extern void USART3_DMA1_Stream3_Write(char *data, uint16_t length);

void log_message(int level, bool newline, const char *format, ...) {
    int len = strlen(format);
    // Add 2 for (potential) newline and null character
    // Add 11 for log level
     
    // Log levels:
    // [FATAL]: 
    // [ERROR]:
    // [WARNING]: 
    // [INFO]: 

    int len_padded = len * 5 + 2 + 11;

    char buffer[len_padded];
    int write_len = 0;
    
    va_list args;

    switch (level) {
        case LOG_LEVEL_FATAL:
            write_len = sprintf(buffer, "%s", "[FATAL]: ");
            break;
        case LOG_LEVEL_ERROR:
            write_len = sprintf(buffer, "%s", "[ERROR]: ");
            break;
        case LOG_LEVEL_WARN:
            write_len = sprintf(buffer, "%s", "[WARNING]: ");
            break;
        case LOG_LEVEL_INFO:
            write_len = sprintf(buffer, "%s", "[INFO]: ");
            break;
        default:
            break;
    }
    
    va_start(args, format);
    write_len += vsprintf(buffer + write_len, format, args);

    if (newline == true) {
        buffer[write_len] = '\n';
        write_len++;
        buffer[write_len] = '\0';
    }
    
    USART3_DMA1_Stream3_Write(buffer, write_len);
    
    va_end(args);
}