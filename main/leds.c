#include "leds.h"
#include "../components/ESP32-NeoPixel-WS2812-RMT/ws2812_control.h"

static int _ledcycle_off[] = {GRB_OFF};
ls_ledcycle_t LEDCYCLE_OFF = {1, portMAX_DELAY, 0, _ledcycle_off};

// values calculated using a separate program and the Adafruit neopixel library, plus some regex massaging
static int _ledcycle_rainbow[] = {0x00FF00, 0x03FF00, 0x14FF00, 0x39FF00, 0x78FF00, 0xD7FF00, 0xFFB400, 0xFF6000, 0xFF2A00, 0xFF0D00, 0xFF0100, 0xFF0000, 0xFF0007, 0xFF001E, 0xFF004B, 0xFF0094, 0xFF00FF, 0x9400FF, 0x4B00FF, 0x1E00FF, 0x0700FF, 0x0000FF, 0x0001FF, 0x000DFF, 0x002AFF, 0x0060FF, 0x00B4FF, 0x00FFD7, 0x00FF78, 0x00FF39, 0x00FF14, 0x00FF03};
ls_ledcycle_t LEDCYCLE_RAINBOW = {32, pdMS_TO_TICKS(200), 2, _ledcycle_rainbow};

static int _ledcycle_warning[] = {GRB_GREEN, GRB_RED, GRB_GREEN, GRB_OFF, GRB_GREEN, GRB_GREEN, GRB_OFF, GRB_OFF, GRB_GREEN, GRB_OFF};
ls_ledcycle_t LEDCYCLE_WARNING = {10, pdMS_TO_TICKS(100), 0, _ledcycle_warning};

// the snoring sound is computed to take 357 ticks, peaking around 71-73; design this to take 360 ticks
static int _ledcycle_sleep[] = {32, 64, 128, 192, 224, 255, 255, 255, 255, 224, 192, 183, 165, 165, 147, 147, 129, 129, 111, 111, 100, 92, 92, 64, 48, 48, 48, 48, 32, 32, 32, 16, 16, 8, 4, 2};
ls_ledcycle_t LEDCYCLE_SLEEP = {36, 10, 0, _ledcycle_sleep};

static int _ledcycle_controls_upper[] = {GRB_YELLOW, GRB_YELLOW, GRB_OFF, GRB_OFF, GRB_OFF, GRB_OFF, GRB_OFF, GRB_OFF, GRB_OFF, GRB_OFF};
ls_ledcycle_t LEDCYCLE_CONTROLS_UPPER = {10, pdMS_TO_TICKS(200), 0, _ledcycle_controls_upper};

static int _ledcycle_controls_lower[] = {GRB_MAGENTA, GRB_OFF, GRB_MAGENTA, GRB_OFF, GRB_OFF, GRB_OFF, GRB_OFF, GRB_OFF, GRB_OFF, GRB_OFF};
ls_ledcycle_t LEDCYCLE_CONTROLS_LOWER = {10, pdMS_TO_TICKS(200), 0, _ledcycle_controls_lower};

static int _ledcycle_controls_both[] = {GRB_MAGENTA, GRB_OFF, GRB_YELLOW, GRB_OFF, GRB_MAGENTA, GRB_OFF, GRB_OFF, GRB_OFF, GRB_OFF, GRB_OFF};
ls_ledcycle_t LEDCYCLE_CONTROLS_BOTH = {10, pdMS_TO_TICKS(200), 0, _ledcycle_controls_both};

static int _ledcycle_static[] = {GRB_OFF};
ls_ledcycle_t LEDCYCLE_STATIC = {1, portMAX_DELAY, 0, _ledcycle_static};

static int _ledcycle_red_flash[] = {0x0F00, 0x7F00, 0xFF00, 0xFF00, 0x7F00, 0x0F00, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0,0,0,0,0,0,0,0,0};
ls_ledcycle_t LEDCYCLE_RED_FLASH = {30, pdMS_TO_TICKS(100), 3, _ledcycle_red_flash};



void ls_leds_handler_task(void *pvParameter)
{
    struct led_state ws2812_state;
    for (int i = 0; i < CONFIG_WS2812_NUM_LEDS; i++)
    {
        ws2812_state.leds[i] = GRB_OFF;
    }
    ws2812_write_leds(ws2812_state);
    ls_ledcycle_t current_cycle = LEDCYCLE_OFF;
    uint16_t cycle_position = 0;
    TickType_t cycle_speed = current_cycle.speed;
    while (1)
    {
        if (xQueueReceive(ls_leds_queue, &current_cycle, cycle_speed) == pdTRUE)
        {
            // there is a new ledcycle to display
            cycle_position = 0;
            cycle_speed = current_cycle.speed;
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
    ls_leds_cycle(LEDCYCLE_STATIC);
}
