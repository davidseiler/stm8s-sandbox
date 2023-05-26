#include <stdint.h>

/* Initialize the lcd with default settings */
void init_lcd();

/* Write a single line to the display, up to 16 characters */
void write_line(char* data, uint8_t size);