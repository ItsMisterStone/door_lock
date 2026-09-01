#include "ssd1306.h"
#include <string.h>

extern I2C_HandleTypeDef hi2c1;   // already defined/inited in main.c

static uint8_t SSD1306_Buffer[SSD1306_WIDTH * SSD1306_PAGES]; // 1024 bytes framebuffer
static uint8_t CurrentX = 0;
static uint8_t CurrentPage = 0;

// Lookup table to rapidly stretch 4 bits into 8 bits (doubling them vertically)
static const uint8_t stretch_lookup[16] = {
    0x00, 0x03, 0x0C, 0x0F,
    0x30, 0x33, 0x3C, 0x3F,
    0xC0, 0xC3, 0xCC, 0xCF,
    0xF0, 0xF3, 0xFC, 0xFF
};

/* ---- Minimal 5x7 font -----------------------------------------------
 * Only covers the characters actually used by this project's messages
 * (letters/space/!). Add more rows the same way if you need new text.
 * Each glyph is 5 columns; a 1px blank column is inserted after each
 * character automatically by SSD1306_Puts().
 * ----------------------------------------------------------------------*/
typedef struct {
    char ch;
    uint8_t glyph[5];
} FontChar_t;

static const FontChar_t font5x7[] = {
    {' ', {0x00,0x00,0x00,0x00,0x00}},
    {'!', {0x00,0x00,0x5F,0x00,0x00}},
    {'A', {0x7E,0x11,0x11,0x11,0x7E}},
    {'B', {0x7F,0x49,0x49,0x49,0x36}},
    {'D', {0x7F,0x41,0x41,0x41,0x3E}},
    {'G', {0x3E,0x41,0x49,0x49,0x7A}},
    {'I', {0x00,0x41,0x7F,0x41,0x00}},
    {'R', {0x7F,0x09,0x19,0x29,0x46}},
    {'S', {0x46,0x49,0x49,0x49,0x31}},
    {'a', {0x20,0x54,0x54,0x54,0x78}},
    {'c', {0x38,0x44,0x44,0x44,0x20}},
    {'d', {0x38,0x44,0x44,0x48,0x7F}},
    {'e', {0x38,0x54,0x54,0x54,0x18}},
    {'h', {0x7F,0x08,0x04,0x04,0x78}},
    {'i', {0x00,0x44,0x7D,0x40,0x00}},
    {'k', {0x7F,0x10,0x28,0x44,0x00}},
    {'n', {0x7C,0x08,0x04,0x04,0x78}},
    {'o', {0x38,0x44,0x44,0x44,0x38}},
    {'p', {0x7C,0x14,0x14,0x14,0x08}},
    {'q', {0x08,0x14,0x14,0x18,0x7C}},
    {'r', {0x7C,0x08,0x04,0x04,0x08}},
    {'s', {0x48,0x54,0x54,0x54,0x24}},
    {'t', {0x04,0x3F,0x44,0x40,0x20}},
    {'u', {0x3C,0x40,0x40,0x20,0x7C}},
    {'w', {0x3C,0x40,0x38,0x40,0x3C}},
};

static const uint8_t* GetGlyph(char ch)
{
    for (uint8_t i = 0; i < sizeof(font5x7) / sizeof(font5x7[0]); i++) {
        if (font5x7[i].ch == ch) return font5x7[i].glyph;
    }
    static const uint8_t blank[5] = {0,0,0,0,0};
    return blank; // unknown char -> blank cell, never crashes
}

static void SSD1306_WriteCommand(uint8_t cmd)
{
    uint8_t data[2] = {0x00, cmd}; // 0x00 = "next byte is a command"
    HAL_I2C_Master_Transmit(&hi2c1, SSD1306_I2C_ADDR, data, 2, 100);
}

static const uint8_t ssd1306_init_cmds[] = {
    0xAE,       // Display OFF
    0xD5, 0x80, // Clock divide ratio / osc freq
    0xA8, 0x3F, // Multiplex ratio = 64
    0xD3, 0x00, // Display offset = 0
    0x40,       // Start line = 0
    0x8D, 0x14, // Charge pump ON
    0x20, 0x00, // Memory addressing mode = horizontal
    0xA1,       // Segment remap
    0xC8,       // COM output scan direction (flipped)
    0xDA, 0x12, // COM pins hardware config
    0x81, 0xCF, // Contrast
    0xD9, 0xF1, // Pre-charge period
    0xDB, 0x40, // VCOMH deselect level
    0xA4,       // Resume display from RAM
    0xA6,       // Normal (non-inverted) display
    0xAF        // Display ON
};

void SSD1306_Init(void)
{
    HAL_Delay(100); // let the panel's regulator stabilize after power-up
    for (uint8_t i = 0; i < sizeof(ssd1306_init_cmds); i++) {
        SSD1306_WriteCommand(ssd1306_init_cmds[i]);
    }
    SSD1306_Clear();
    SSD1306_UpdateScreen();
}

void SSD1306_DisplayOn(void)  { SSD1306_WriteCommand(0xAF); }
void SSD1306_DisplayOff(void) { SSD1306_WriteCommand(0xAE); }

void SSD1306_Clear(void)
{
    memset(SSD1306_Buffer, 0x00, sizeof(SSD1306_Buffer));
}

void SSD1306_UpdateScreen(void)
{
    for (uint8_t page = 0; page < SSD1306_PAGES; page++) {
        SSD1306_WriteCommand(0xB0 + page); // set page address
        SSD1306_WriteCommand(0x00);        // set lower column addr = 0
        SSD1306_WriteCommand(0x10);        // set higher column addr = 0

        uint8_t data[SSD1306_WIDTH + 1];
        data[0] = 0x40; // "next bytes are display data"
        memcpy(&data[1], &SSD1306_Buffer[page * SSD1306_WIDTH], SSD1306_WIDTH);
        HAL_I2C_Master_Transmit(&hi2c1, SSD1306_I2C_ADDR, data, sizeof(data), 100);
    }
}

void SSD1306_GotoXY(uint8_t x, uint8_t page)
{
    CurrentX = x;
    CurrentPage = page;
}

void SSD1306_Puts(const char *str)
{
    while (*str) {
        const uint8_t *glyph = GetGlyph(*str);
        if (CurrentX > SSD1306_WIDTH - 12) break; // Out of room on this line (10px char + 2px space)

        for (uint8_t col = 0; col < 5; col++) {
            // Stretch the 8 vertical pixels into 16 vertical pixels (split across 2 pages)
            uint8_t top_page_col = stretch_lookup[glyph[col] & 0x0F];
            uint8_t bot_page_col = stretch_lookup[glyph[col] >> 4];

            // Write each column twice to double the horizontal width
            for (uint8_t w = 0; w < 2; w++) {
                if (CurrentPage < SSD1306_PAGES)
                    SSD1306_Buffer[CurrentPage * SSD1306_WIDTH + CurrentX] = top_page_col;
                
                if (CurrentPage + 1 < SSD1306_PAGES)
                    SSD1306_Buffer[(CurrentPage + 1) * SSD1306_WIDTH + CurrentX] = bot_page_col;
                
                CurrentX++;
            }
        }
        
        // 2px blank spacing between characters
        for (uint8_t w = 0; w < 2; w++) {
            if (CurrentPage < SSD1306_PAGES)
                SSD1306_Buffer[CurrentPage * SSD1306_WIDTH + CurrentX] = 0x00;
            if (CurrentPage + 1 < SSD1306_PAGES)
                SSD1306_Buffer[(CurrentPage + 1) * SSD1306_WIDTH + CurrentX] = 0x00;
            CurrentX++;
        }
        str++;
    }
}

void SSD1306_PutsCentered(uint8_t page, const char *str)
{
    uint16_t width_px = (uint16_t)strlen(str) * 12; // 10px doubled glyph + 2px space
    uint8_t x = (width_px < SSD1306_WIDTH) ? (uint8_t)((SSD1306_WIDTH - width_px) / 2) : 0;
    SSD1306_GotoXY(x, page);
    SSD1306_Puts(str);
}

void SSD1306_ShowMessage(const char *line1, const char *line2)
{
    SSD1306_Clear();
    
    // Characters now occupy 2 pages vertically, so the page spacing is widened
    if (line2 != NULL) {
        SSD1306_PutsCentered(2, line1); // Occupies pages 2 and 3
        SSD1306_PutsCentered(5, line2); // Occupies pages 5 and 6
    } else {
        SSD1306_PutsCentered(3, line1); // Occupies pages 3 and 4
    }
    
    SSD1306_UpdateScreen();
}