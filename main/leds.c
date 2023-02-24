#include "leds.h"
#include "ws2812_control.h"

static int _ledcycle_off[] = {GRB_OFF};
ls_ledcycle_t LEDCYCLE_OFF = {1, portMAX_DELAY, 0, _ledcycle_off};

static int _ledcycle_rainbow[] = {GRB_RED, GRB_YELLOW, GRB_GREEN, GRB_CYAN, GRB_BLUE, GRB_MAGENTA};
ls_ledcycle_t LEDCYCLE_RAINBOW = {6, pdMS_TO_TICKS(200), 1, _ledcycle_rainbow};

static int _ledcycle_warning[] = {GRB_GREEN, GRB_RED, GRB_GREEN, GRB_OFF, GRB_GREEN, GRB_GREEN, GRB_OFF, GRB_OFF, GRB_GREEN, GRB_OFF};
ls_ledcycle_t LEDCYCLE_WARNING = {10, pdMS_TO_TICKS(100), 0, _ledcycle_warning};

// the snoring sound is computed to take 357 ticks, peaking around 71-73; design this to take 360 ticks
static int _ledcycle_sleep[] = {32, 64, 128, 192, 224, 255, 255, 255, 255, 224, 192, 183, 165, 165, 147, 147, 129, 129, 111, 111, 100, 92, 92, 64, 48, 48, 48, 48, 32, 32, 32, 16, 16, 8, 4, 2};
ls_ledcycle_t LEDCYCLE_SLEEP = {36, 10, 0, _ledcycle_sleep};

static int _ledcycle_controls_upper[] = {GRB_YELLOW, GRB_YELLOW, GRB_OFF, GRB_OFF, GRB_OFF, GRB_OFF, GRB_OFF, GRB_OFF, GRB_OFF, GRB_OFF};
ls_ledcycle_t LEDCYCLE_CONTROLS_UPPER = {10, pdMS_TO_TICKS(200), 0, _ledcycle_controls_upper};

static int _ledcycle_controls_lower[] = {GRB_MAGENTA, GRB_OFF, GRB_MAGENTA, GRB_OFF, GRB_OFF, GRB_OFF, GRB_OFF, GRB_OFF, GRB_OFF, GRB_OFF};
ls_ledcycle_t LEDCYCLE_CONTROLS_LOWER = {10, pdMS_TO_TICKS(200), 0, _ledcycle_controls_lower};

static int _ledcycle_controls_both[] = {GRB_MAGENTA, GRB_OFF,  GRB_YELLOW, GRB_OFF, GRB_MAGENTA, GRB_OFF, GRB_OFF, GRB_OFF, GRB_OFF, GRB_OFF};
ls_ledcycle_t LEDCYCLE_CONTROLS_BOTH = {10, pdMS_TO_TICKS(200), 0, _ledcycle_controls_both};

static int _ledcycle_static[] = {GRB_OFF};
ls_ledcycle_t LEDCYCLE_STATIC = {1, portMAX_DELAY, 0, _ledcycle_static};

void ls_leds_handler_task(void *pvParameter)
{
    struct ws2812_led_state_t ws2812_state;
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
    xQueueSend(ls_leds_queue, (void *) &ledcycle, 0); // don't block if queue full
}

void ls_leds_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    _ledcycle_static[0] = ((int) green << 16) | ((int) red << 8) | (int) blue;
    ls_leds_cycle(LEDCYCLE_STATIC);
}
