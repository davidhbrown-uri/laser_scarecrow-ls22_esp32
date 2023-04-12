#include "leds.h"
#include "util.h"
#include "debug.h"
#include "../components/ESP32-NeoPixel-WS2812-RMT/ws2812_control.h"

static int _ledcycle_off[] = {GRB_OFF};
ls_ledcycle_t LEDCYCLE_OFF = {sizeof(_ledcycle_off) / sizeof(int), portMAX_DELAY, 0, _ledcycle_off};

// values calculated using a separate program and the Adafruit neopixel library, plus some regex massaging
static int _ledcycle_rainbow[] = {0x00FF00, 0x03FF00, 0x14FF00, 0x39FF00, 0x78FF00, 0xD7FF00, 0xFFB400, 0xFF6000, 0xFF2A00, 0xFF0D00, 0xFF0100, 0xFF0000, 0xFF0007, 0xFF001E, 0xFF004B, 0xFF0094, 0xFF00FF, 0x9400FF, 0x4B00FF, 0x1E00FF, 0x0700FF, 0x0000FF, 0x0001FF, 0x000DFF, 0x002AFF, 0x0060FF, 0x00B4FF, 0x00FFD7, 0x00FF78, 0x00FF39, 0x00FF14, 0x00FF03};
ls_ledcycle_t LEDCYCLE_RAINBOW = {sizeof(_ledcycle_rainbow) / sizeof(int), pdMS_TO_TICKS(100), 2, _ledcycle_rainbow};

static int _ledcycle_warning[] = {GRB_GREEN, GRB_RED, GRB_GREEN, GRB_OFF, GRB_GREEN, GRB_GREEN, GRB_OFF, GRB_OFF, GRB_GREEN, GRB_OFF};
ls_ledcycle_t LEDCYCLE_WARNING = {sizeof(_ledcycle_warning) / sizeof(int), pdMS_TO_TICKS(100), 0, _ledcycle_warning};

// the snoring sound is computed to take 357 ticks, peaking around 71-73; design this to take 360 ticks
static int _ledcycle_sleep[] = {32, 64, 128, 192, 224, 255, 255, 255, 255, 224, 192, 183, 165, 165, 147, 147, 129, 129, 111, 111, 100, 92, 92, 64, 48, 48, 48, 48, 32, 32, 32, 16, 16, 8, 4, 2};
ls_ledcycle_t LEDCYCLE_SLEEP = {sizeof(_ledcycle_sleep) / sizeof(int), 10, 0, _ledcycle_sleep};

static int _ledcycle_controls_upper[] = {GRB_GREEN, GRB_GREEN, GRB_OFF, GRB_OFF, GRB_OFF, GRB_OFF, GRB_OFF, GRB_OFF, GRB_OFF, GRB_OFF};
ls_ledcycle_t LEDCYCLE_CONTROLS_UPPER = {sizeof(_ledcycle_controls_upper) / sizeof(int), pdMS_TO_TICKS(200), 0, _ledcycle_controls_upper};

static int _ledcycle_controls_lower[] = {GRB_MAGENTA, GRB_OFF, GRB_MAGENTA, GRB_OFF, GRB_OFF, GRB_OFF, GRB_OFF, GRB_OFF, GRB_OFF, GRB_OFF};
ls_ledcycle_t LEDCYCLE_CONTROLS_LOWER = {sizeof(_ledcycle_controls_lower) / sizeof(int), pdMS_TO_TICKS(200), 0, _ledcycle_controls_lower};

static int _ledcycle_controls_both[] = {GRB_MAGENTA, GRB_OFF, GRB_GREEN, GRB_OFF, GRB_MAGENTA, GRB_OFF, GRB_OFF, GRB_OFF, GRB_OFF, GRB_OFF};
ls_ledcycle_t LEDCYCLE_CONTROLS_BOTH = {sizeof(_ledcycle_controls_both) / sizeof(int), pdMS_TO_TICKS(200), 0, _ledcycle_controls_both};

static int _ledcycle_static[CONFIG_WS2812_NUM_LEDS];
ls_ledcycle_t LEDCYCLE_STATIC = {CONFIG_WS2812_NUM_LEDS, portMAX_DELAY, 1, _ledcycle_static};

static int _ledcycle_red_flash[] = {0x0F00, 0x7F00, 0xFF00, 0xFF00, 0x7F00, 0x0F00, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
ls_ledcycle_t LEDCYCLE_RED_FLASH = {sizeof(_ledcycle_red_flash) / sizeof(int), pdMS_TO_TICKS(100), 3, _ledcycle_red_flash};

static int _ledcycle_green_pulse[] = {0x000000, 0x100000, 0x200000, 0x400000, 0x800000, 0xA00000, 0xC00000, 0xC00000,
                                      0xC00000, 0xC00000, 0xA00000, 0x800000, 0x400000, 0x200000, 0x100000, 0x000000,
                                      0, 0, 0, 0, 0, 0, 0, 0};
ls_ledcycle_t LEDCYCLE_GREEN_PULSE = {sizeof(_ledcycle_green_pulse) / sizeof(int), pdMS_TO_TICKS(100), 2, _ledcycle_green_pulse};
static int _ledcycle_yellow_pulse[] = {0x000000, 0x102000, 0x203000, 0x406000, 0x809000, 0xA0B000, 0xC0E000, 0xC0E000,
                                       0xC0E000, 0xC0E000, 0xA0B000, 0x809000, 0x406000, 0x203000, 0x102000, 0x000000};
ls_ledcycle_t LEDCYCLE_YELLOW_PULSE = {sizeof(_ledcycle_yellow_pulse) / sizeof(int), pdMS_TO_TICKS(100), 2, _ledcycle_yellow_pulse};
;

void ls_leds_handler_task(void *pvParameter)
{
    struct led_state ws2812_state;
    for (int i = 0; i < CONFIG_WS2812_NUM_LEDS; i++)
    {
        ws2812_state.leds[i] = GRB_OFF;
    }
    ws2812_write_leds(ws2812_state);
    ls_ledcycle_t cycle_queued;
    ls_ledcycle_t current_cycle = LEDCYCLE_OFF;
    uint16_t cycle_position = 0;
    TickType_t cycle_speed = current_cycle.speed;
    while (1)
    {
        if (xQueueReceive(ls_leds_queue, &cycle_queued, cycle_speed) == pdTRUE)
        {
            if (current_cycle.data != cycle_queued.data)
            { // there is a new ledcycle to display
                current_cycle = cycle_queued;
                cycle_position = 0;
                cycle_speed = current_cycle.speed;
            }
        }
        for (int i = 0; i < CONFIG_WS2812_NUM_LEDS; i++)
        {
            ws2812_state.leds[i] = current_cycle.data[(cycle_position + i * current_cycle.offset) % current_cycle.length];
        }
        ws2812_write_leds(ws2812_state);
        cycle_position++;
    } // while 1
}

void ls_leds_off(void)
{
    xQueueSend(ls_leds_queue, (void *)&LEDCYCLE_OFF, 0); // don't block if queue full
}

void ls_leds_init(void)
{
    ws2812_control_init();
    ls_leds_queue = xQueueCreate(32, sizeof(ls_ledcycle_t));
}

void ls_leds_cycle(struct ls_ledcycle ledcycle)
{
    xQueueSend(ls_leds_queue, (void *)&ledcycle, 0); // don't block if queue full
}

void ls_leds_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    _ledcycle_static[0] = ((int)green << 16) | ((int)red << 8) | (int)blue;
    for (int i = 1; i < CONFIG_WS2812_NUM_LEDS; i++) // copy to rest
    {
        _ledcycle_static[i] = _ledcycle_static[0];
    }
    ls_leds_cycle(LEDCYCLE_STATIC);
}
void ls_leds_single(int which, int color)
{
    if (which >= 0 && which < CONFIG_WS2812_NUM_LEDS)
    {
        _ledcycle_static[which] = color;
    }
    ls_leds_cycle(LEDCYCLE_STATIC);
}

static int _ledcycle_2sec[20];
ls_ledcycle_t LEDCYCLE_TWO_SECONDS = {sizeof(_ledcycle_2sec) / sizeof(int), pdMS_TO_TICKS(2000) / (sizeof(_ledcycle_2sec) / sizeof(int)), 0, _ledcycle_2sec};

void ls_leds_pulses_2sec(int count, int color)
{
    int slots = sizeof(_ledcycle_2sec) / sizeof(int);
    count = _constrain(count, 1, slots / 2);
    int step = (count * 3 < slots/2) ? 2 : 1;
    for (int i = 0; i < slots; i += step * 2)
    {
        _ledcycle_2sec[i] = (i / 2 / step < count) ? color : GRB_OFF;
        _ledcycle_2sec[i + step - 1] = (i / 2 / step < count) ? color : GRB_OFF; // same index if step=1
        _ledcycle_2sec[i + step] = GRB_OFF;
        _ledcycle_2sec[i + step + step - 1] = GRB_OFF; // same index if step=1
    }
#ifdef LSDEBUG_LEDS
    ls_debug_printf("ls_leds_pulses_2sec has %d slots; count %d => step=%d\n", slots, count, step);
    for (int i = 0; i < slots; i++)
    {
        ls_debug_printf("0x%06x ", _ledcycle_2sec[i]);
    }
    ls_debug_printf("\n");
#endif
    ls_leds_cycle(LEDCYCLE_TWO_SECONDS);
}