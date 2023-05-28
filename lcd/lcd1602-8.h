#include <stdint.h>

/* HD44780 LCD1602 operating in 8-bit mode*/


/* Initialize the lcd with default settings */
void LCD_init();

/* Clear display and set cursor to home location */
void LCD_clear();

/* Write a single character to the display */
void LCD_write_character(char c, uint16_t display_speed_ms);

/* Write full display (16x2) with refresh*/
void LCD_write_16_2(char data[32], uint16_t display_speed_ms);

/* Write data of any size to the LCD's DDRAM. This device is limited to 80 bytes,
 but only displays addresses 0-15 and 40-56*/
void LCD_write(char* data, uint8_t size, uint16_t display_speed_ms);