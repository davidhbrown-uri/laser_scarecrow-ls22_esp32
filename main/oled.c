/*
    Control software for URI Laser Scarecrow, 2022 Model
    Copyright (C) 2022-2023 David H. Brown

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include <stdio.h>
#include <stdarg.h>
#include "oled.h"
#include "i2c.h"
#include "util.h"
#include "../components/ssd1306/ssd1306.h"

#define LS_OLED_WIDTH 128
#define LS_OLED_HEIGHT 64
#define LS_OLED_ADDRESS 0x3C
#define LS_OLED_ADDRESS_ALTERNATE 0x3D
#define LS_OLED_LINES (LS_OLED_HEIGHT / 8)
#define LS_OLED_CHARS (LS_OLED_WIDTH / 8)
#define LS_OLED_RETURN_IF_NOT_PRESENT {if (_ls_oled_status != LS_OLED_PRESENT) { return; }}

SSD1306_t _ls_oled_ssd1306_dev;

static enum ls_oled_status_t _ls_oled_status = LS_OLED_UNKNOWN;

static char _ls_oled_line_buffer[LS_OLED_CHARS];
static uint8_t _ls_oled_line_number = 0;

static uint8_t _ls_oled_logo[] = {
    // 'URI agronomy vegetables-128x64', 128x64px
    0xff, 0xff, 0xff, 0xff, 0xe0, 0x41, 0xfb, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xf4, 0xb8, 0x7c, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xf2, 0xc4, 0x3d, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xdf, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xfa, 0x6b, 0x1f, 0x5f, 0xff, 0xff, 0xff, 0xff, 0xdf, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xfe, 0xb4, 0x42, 0xbf, 0xff, 0xff, 0xff, 0xff, 0x8f, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xfe, 0x97, 0x80, 0x0f, 0xff, 0xff, 0xff, 0xff, 0x0f, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xfe, 0x54, 0xa8, 0xaf, 0xff, 0xff, 0xff, 0xff, 0x03, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0x56, 0xb0, 0x47, 0xff, 0xff, 0xff, 0xfe, 0x46, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0x55, 0x98, 0x2f, 0xff, 0xff, 0xff, 0xfe, 0x06, 0x7f, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0x22, 0xce, 0x03, 0xff, 0xff, 0xff, 0xde, 0x20, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0x9d, 0x01, 0x17, 0xff, 0xff, 0xff, 0xc6, 0x10, 0x7f, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0x86, 0xa1, 0x83, 0xff, 0xff, 0xff, 0xc3, 0x30, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0xaa, 0x41, 0xff, 0xff, 0xfe, 0x00, 0x60, 0x7f, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xd2, 0x49, 0xa0, 0x7f, 0xff, 0xfe, 0x54, 0x44, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0x81, 0x96, 0x54, 0x3e, 0xff, 0xfe, 0x02, 0x01, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xd0, 0x97, 0x52, 0x1f, 0xbf, 0xff, 0x00, 0x03, 0xef, 0xff, 0xff,
    0xff, 0xff, 0xf7, 0xff, 0xff, 0xc9, 0x95, 0x51, 0x0f, 0xbf, 0xff, 0x48, 0x4f, 0x8f, 0xff, 0xff,
    0xff, 0xff, 0xe7, 0xff, 0xff, 0x90, 0x55, 0xac, 0x23, 0x9f, 0xff, 0x8d, 0x0c, 0x25, 0xff, 0xff,
    0xff, 0xff, 0xef, 0xff, 0xff, 0xc4, 0x85, 0xa8, 0x13, 0xaf, 0xff, 0xe4, 0x24, 0x00, 0x7f, 0xff,
    0xff, 0xff, 0xcf, 0xff, 0xff, 0xc2, 0x59, 0x6a, 0x81, 0xcf, 0xf1, 0x40, 0x00, 0x00, 0x7f, 0xff,
    0xff, 0xff, 0xcf, 0xff, 0xd5, 0x62, 0x89, 0x68, 0x54, 0xe5, 0xf8, 0x10, 0x00, 0x00, 0xff, 0xff,
    0xff, 0xff, 0x97, 0xff, 0x2d, 0x5b, 0x46, 0x2d, 0x10, 0xd0, 0xf0, 0x40, 0x00, 0x00, 0xe0, 0x7f,
    0xff, 0xfe, 0x77, 0xfc, 0x52, 0x84, 0x45, 0x1a, 0x02, 0x60, 0x70, 0x00, 0x10, 0x03, 0x80, 0x3f,
    0xff, 0xfe, 0x73, 0xdd, 0x5d, 0x01, 0x60, 0x4b, 0x5a, 0x30, 0x20, 0x00, 0x10, 0x07, 0x80, 0x7f,
    0xff, 0xf8, 0xfb, 0x95, 0x42, 0xa8, 0xe4, 0x4d, 0x01, 0x00, 0x29, 0x10, 0x4c, 0x0b, 0x00, 0xff,
    0xff, 0xf9, 0xf0, 0x32, 0x3c, 0x10, 0x21, 0xa2, 0x94, 0x88, 0x00, 0x00, 0x00, 0x00, 0x04, 0x0f,
    0xff, 0xfb, 0xf0, 0x05, 0xa3, 0x65, 0x20, 0x85, 0xc2, 0x40, 0x00, 0x44, 0x02, 0x00, 0x1a, 0x0f,
    0xff, 0xf7, 0x80, 0x02, 0x59, 0x14, 0x90, 0xd4, 0x2a, 0x40, 0x00, 0x2a, 0x02, 0x00, 0x00, 0x03,
    0xff, 0xe7, 0x80, 0x01, 0x56, 0x90, 0x54, 0x12, 0xa1, 0x00, 0x00, 0x29, 0x00, 0x40, 0x00, 0x00,
    0xff, 0x8f, 0x00, 0x40, 0xad, 0x4e, 0x02, 0x8b, 0x45, 0x20, 0x02, 0x9c, 0x00, 0x00, 0x00, 0x01,
    0xff, 0x1e, 0x00, 0x90, 0x55, 0xa2, 0x28, 0x02, 0x28, 0x90, 0x00, 0x57, 0x02, 0x00, 0x07, 0x07,
    0xff, 0xee, 0x90, 0xc8, 0x56, 0xb9, 0x25, 0xf1, 0x42, 0x40, 0x10, 0x54, 0x50, 0x03, 0x03, 0xff,
    0xff, 0xec, 0x49, 0xa0, 0x53, 0x4a, 0x85, 0x78, 0xa9, 0x20, 0x05, 0x5a, 0x00, 0x01, 0x00, 0xff,
    0xff, 0xed, 0x1c, 0x80, 0x2d, 0xb0, 0x0a, 0xf4, 0x20, 0x40, 0x00, 0x6a, 0x00, 0x00, 0x01, 0xff,
    0xff, 0xec, 0x20, 0x44, 0x57, 0xaa, 0x13, 0x7b, 0x19, 0x00, 0x11, 0x28, 0x80, 0x00, 0x01, 0xff,
    0xfe, 0x8c, 0x04, 0x20, 0x1a, 0xa9, 0x01, 0x9c, 0x8c, 0xa0, 0x08, 0x08, 0x40, 0x42, 0x00, 0xff,
    0xf8, 0x00, 0x20, 0x04, 0x2b, 0xec, 0x94, 0xec, 0x08, 0x00, 0x02, 0xa0, 0x00, 0x63, 0x06, 0xff,
    0xf0, 0x00, 0x00, 0x00, 0x5d, 0x52, 0x02, 0xff, 0xc2, 0x10, 0x00, 0x00, 0x00, 0x03, 0x83, 0xff,
    0xe4, 0x00, 0x02, 0x20, 0x0b, 0xb5, 0x04, 0xbf, 0xc1, 0x80, 0x00, 0x22, 0x06, 0x01, 0xc1, 0xff,
    0xc2, 0x00, 0x00, 0x00, 0x6a, 0xd2, 0x03, 0xff, 0x50, 0x80, 0x00, 0x01, 0x03, 0x00, 0xf1, 0xff,
    0xd1, 0x0a, 0x80, 0x00, 0x2e, 0xeb, 0x13, 0xbf, 0xc0, 0x00, 0x00, 0x10, 0x00, 0x00, 0xff, 0xff,
    0x81, 0x90, 0x01, 0x08, 0x97, 0x74, 0x07, 0xff, 0xa0, 0x00, 0x00, 0x40, 0x20, 0x00, 0x7f, 0xff,
    0x81, 0x3c, 0x08, 0x40, 0x55, 0xaf, 0x03, 0xff, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3f, 0xff,
    0x88, 0x01, 0x00, 0x00, 0x56, 0xf5, 0x06, 0xff, 0xf8, 0x80, 0x00, 0x08, 0x10, 0xc5, 0x3f, 0xff,
    0x02, 0x28, 0x00, 0x01, 0x37, 0x3d, 0x8b, 0xbf, 0xfc, 0x00, 0x00, 0x00, 0x18, 0xff, 0xff, 0xff,
    0x40, 0x01, 0x00, 0x02, 0x53, 0xee, 0x06, 0xff, 0x7c, 0x06, 0x10, 0x00, 0x01, 0x21, 0xb7, 0xff,
    0x04, 0x88, 0x00, 0x01, 0x0d, 0x7a, 0xc5, 0xef, 0xff, 0x7f, 0x86, 0x00, 0x00, 0x00, 0x07, 0xff,
    0x80, 0x01, 0x00, 0x80, 0x2b, 0x6a, 0xf5, 0x7f, 0xff, 0xff, 0x0c, 0x00, 0x00, 0x00, 0x1f, 0xff,
    0x80, 0x40, 0x5f, 0xfd, 0x0b, 0x75, 0xf5, 0x5f, 0xfc, 0x80, 0x00, 0x00, 0x00, 0x40, 0x3f, 0xff,
    0x80, 0x00, 0x3f, 0xfe, 0x14, 0x88, 0x02, 0xa2, 0x00, 0x00, 0x0a, 0xff, 0x07, 0xff, 0xff, 0xff,
    0x82, 0x02, 0x3f, 0xff, 0xf1, 0x20, 0x00, 0x03, 0x00, 0x05, 0xff, 0xff, 0x02, 0xff, 0xff, 0xff,
    0xc0, 0x40, 0x7f, 0xff, 0xf8, 0x00, 0x00, 0x01, 0x07, 0xff, 0xff, 0xff, 0x82, 0x7f, 0xff, 0xff,
    0xe0, 0x00, 0x7f, 0xff, 0xfc, 0x28, 0x00, 0x00, 0xff, 0x9f, 0xff, 0x7f, 0xc1, 0x0f, 0xff, 0xff,
    0xf0, 0x00, 0xf7, 0xbf, 0xff, 0xff, 0xf3, 0xdf, 0x7e, 0x06, 0xec, 0x39, 0xe1, 0xdf, 0xff, 0xff,
    0xf8, 0x03, 0xf3, 0x3f, 0xff, 0xcf, 0xf3, 0x9f, 0x7c, 0xb6, 0xe5, 0x9b, 0xf0, 0x7f, 0xff, 0xff,
    0xfd, 0x07, 0xf7, 0x3f, 0xff, 0xdf, 0xf7, 0x9f, 0x9b, 0x14, 0xe9, 0x9b, 0xf8, 0x1f, 0xff, 0xff,
    0xff, 0xff, 0xf7, 0x63, 0x19, 0x86, 0x31, 0x99, 0x8a, 0xa4, 0xe8, 0x33, 0xfc, 0x00, 0x1f, 0xff,
    0xff, 0xff, 0xf6, 0x42, 0x10, 0x8e, 0xb0, 0xb1, 0x30, 0x8d, 0xc8, 0x73, 0xff, 0x00, 0x23, 0xff,
    0xff, 0xff, 0xf6, 0xc6, 0x91, 0x9d, 0xa4, 0xa3, 0x26, 0x1d, 0xdb, 0x37, 0xff, 0xff, 0xf9, 0xff,
    0xff, 0xff, 0xf4, 0x9c, 0x37, 0xb5, 0x2d, 0x2f, 0xd7, 0xe9, 0x93, 0x37, 0xff, 0xff, 0xfc, 0xff,
    0xff, 0xff, 0xf1, 0xc2, 0x30, 0x8c, 0x63, 0x21, 0x1b, 0x9c, 0x33, 0xa7, 0xff, 0xff, 0xfe, 0x7f,
    0xff, 0xff, 0xfb, 0xc7, 0xb1, 0xdf, 0xe7, 0xf3, 0x38, 0x1c, 0xff, 0xef, 0xff, 0xff, 0xff, 0xbf,
    0xff, 0xff, 0xff, 0xff, 0x3f, 0xff, 0xff, 0xff, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xfe, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

enum ls_oled_status_t ls_oled_status(void)
{
    return _ls_oled_status;
}

void ls_oled_init(void)
{
    if (ls_i2c_probe_address(LS_OLED_ADDRESS))
    {
        _ls_oled_ssd1306_dev._address = LS_OLED_ADDRESS;
        _ls_oled_status = LS_OLED_PRESENT;
    }
    else if (ls_i2c_probe_address(LS_OLED_ADDRESS_ALTERNATE))
    {
        _ls_oled_ssd1306_dev._address = LS_OLED_ADDRESS_ALTERNATE;
        _ls_oled_status = LS_OLED_PRESENT;
    }
    else
    {
        _ls_oled_status = LS_OLED_NOT_FOUND;
        return;
    }
    _ls_oled_ssd1306_dev._flip = false;
    ssd1306_init(&_ls_oled_ssd1306_dev, 128, 64);
    ssd1306_contrast(&_ls_oled_ssd1306_dev, 0xff);
    ls_oled_blank_screen();
}
void ls_oled_show_logo(void)
{
LS_OLED_RETURN_IF_NOT_PRESENT
    xSemaphoreTake(i2c_mux, LSI2C_MUX_TICKS);
    ssd1306_bitmaps(&_ls_oled_ssd1306_dev, 0, 0, _ls_oled_logo, 128, 64, false);
    xSemaphoreGive(i2c_mux);
}

void ls_oled_blank_screen(void)
{
LS_OLED_RETURN_IF_NOT_PRESENT
    xSemaphoreTake(i2c_mux, LSI2C_MUX_TICKS);
    ssd1306_clear_screen(&_ls_oled_ssd1306_dev, false);
    xSemaphoreGive(i2c_mux);
    _ls_oled_line_number = 0;
}

void ls_oled_goto_line(uint8_t line_number)
{
    _ls_oled_line_number = line_number % LS_OLED_LINES;
}

void _ls_oled_fill_line_buffer(char c)
{
    for (int i = 0; i < LS_OLED_CHARS; i++)
    {
        _ls_oled_line_buffer[i] = c;
    }
}

void ls_oled_println(char * format, ...)
{
LS_OLED_RETURN_IF_NOT_PRESENT
    _ls_oled_fill_line_buffer(' ');
    va_list args;
    va_start(args, format);
    vsnprintf(_ls_oled_line_buffer, sizeof(_ls_oled_line_buffer), format, args);
    va_end(args);
    xSemaphoreTake(i2c_mux, LSI2C_MUX_TICKS);
    ssd1306_display_text(&_ls_oled_ssd1306_dev, _ls_oled_line_number, _ls_oled_line_buffer, LS_OLED_CHARS, false);
    xSemaphoreGive(i2c_mux);
    ls_oled_goto_line(_ls_oled_line_number + 1);
}
void ls_oled_clear_line(uint8_t line_number)
{
    line_number = line_number % LS_OLED_LINES;
    xSemaphoreTake(i2c_mux, LSI2C_MUX_TICKS);
    ssd1306_clear_line(&_ls_oled_ssd1306_dev, line_number, false);
    xSemaphoreGive(i2c_mux);
}