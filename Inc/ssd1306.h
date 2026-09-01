#ifndef SSD1306_H
#define SSD1306_H

#include "main.h"   // pulls in hi2c1 handle + stdint via HAL headers
#include <stdint.h>
#include <stdbool.h>

#define SSD1306_I2C_ADDR   (0x3C << 1)   // 0x78, 8-bit HAL format
#define SSD1306_WIDTH      128
#define SSD1306_HEIGHT     64
#define SSD1306_PAGES      (SSD1306_HEIGHT / 8)   // 8

void SSD1306_Init(void);
void SSD1306_DisplayOn(void);
void SSD1306_DisplayOff(void);
void SSD1306_Clear(void);
void SSD1306_UpdateScreen(void);
void SSD1306_GotoXY(uint8_t x, uint8_t page);
void SSD1306_Puts(const char *str);
void SSD1306_PutsCentered(uint8_t page, const char *str);

// Convenience: clears, writes up to 2 centered lines, pushes to the panel
void SSD1306_ShowMessage(const char *line1, const char *line2);

#endif /* SSD1306_H */