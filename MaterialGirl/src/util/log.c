#include "log.h"

#include <stdio.h>
#include <stdint.h>
#include <string.h>

extern void USART3_DMA1_Stream3_Write(char *data, uint16_t length);

void log_message(int level, const char *format, bool newline, ...) {
    int len = strlen(format);
    int len_padded = len * 2 + 1;

    char buffer[len_padded];

    int write_len = 0;

    va_list args;
    va_start(args, format);
    write_len = sprintf(buffer, format, args);
    va_end(args);

    USART3_DMA1_Stream3_Write(buffer, write_len);
}