/**
 * @file coverage.c
 * @author David Brown
 * @brief
 * @version 0.1
 * @date 2022-12-27
 *
 * @copyright Copyright (c) 2022
 *
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "coverage.h"
#include "map.h"
#include "debug.h"
#ifdef LSDEBUG_COVERAGE
#include "buzzer.h"
#endif

static bool _coverage_ready = false;
static struct timeval _ls_coverage_most_recent;
static int _ls_coverage_positions_index;

bool ls_coverage_is_ready(void)
{
    return _coverage_ready;
}

double _span_coverage_percent_of_ideal(struct ls_map_SpanNode *span)
{
    return ((double)span->coverage) * 100000.0 / LS_COVERAGE_POSITIONS_COUNT / ((double)span->permil);
}

void ls_coverage_initialize(void)
{
    struct ls_map_SpanNode *currentSpan = ls_map_span_first;
    gettimeofday(&_ls_coverage_most_recent, NULL);
    do
    {
        currentSpan->coverage = 0;
        currentSpan = currentSpan->next;
    } while (currentSpan != ls_map_span_first);
    _coverage_ready = false;
    _ls_coverage_positions_index = 0;
}

struct ls_map_SpanNode *ls_coverage_next_span(void)
{
    if (!ls_coverage_is_ready())
    {
        return ls_map_span_next(ls_stepper_get_position(), ls_stepper_get_direction(), ls_map_span_first);
    }
    struct ls_map_SpanNode *currentSpan = ls_map_span_first;
    struct ls_map_SpanNode *least_coverage_span = ls_map_span_first;
    // _set_span_coverage(); // now being done in the coverage task
    do
    {
#ifdef LSDEBUG_COVERAGE
        ls_debug_printf("%d-%d [%d‰] seen %d times (%1.2f%%)\n", currentSpan->begin, currentSpan->end, currentSpan->permil, currentSpan->coverage, _span_coverage_percent_of_ideal(currentSpan));
#endif
        currentSpan = currentSpan->next;
        if (_span_coverage_percent_of_ideal(currentSpan) < _span_coverage_percent_of_ideal(least_coverage_span))
        {
            least_coverage_span = currentSpan;
        }
    } while (currentSpan != ls_map_span_first);
    return least_coverage_span;
}

void ls_coverage_task(void *pvParameter)
{
#ifdef LSDEBUG_COVERAGE
/* as of 2022-12-27 does not seem to be leaking memory. I hadn't thought I was allocating anything dynamically, but confirmation was nice
Power-on: Beginning ls_coverage_task with 268188 heap memory free.
Controls: Beginning ls_coverage_task with 267732 heap memory free.
Controls: Beginning ls_coverage_task with 267732 heap memory free.
Controls: Beginning ls_coverage_task with 267732 heap memory free.
Wake-up:  Beginning ls_coverage_task with 267732 heap memory free.
*/
    ls_debug_printf("\nBeginning ls_coverage_task with %d heap memory free.\n", xPortGetFreeHeapSize());
#endif
    int gpio = lsgpio_laserpowerenable();
    BaseType_t outreg = gpio < 32 ? GPIO_OUT_REG : GPIO_OUT1_REG;
    gpio_num_t pin = (gpio_num_t)(gpio & 0x1F);
    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);
    int64_t elapsed_sec = tv_now.tv_sec - _ls_coverage_most_recent.tv_sec;
    if (!ls_coverage_is_ready() || elapsed_sec > LS_COVERAGE_POSITIONS_INVALID_AFTER_SEC)
    {
#ifdef LSDEBUG_COVERAGE
        ls_debug_printf("\nInitializing coverage after %d seconds elapsed.\n", (int) elapsed_sec);
#endif
        ls_coverage_initialize();
    }
    while (1)
    {
        if ((GPIO_REG_READ(outreg) >> pin) & 1U)
        {
#ifdef LSDEBUG_COVERAGE
            ls_buzzer_tone(ls_stepper_get_position() * 2 + 1000);
#endif
            gettimeofday(&_ls_coverage_most_recent, NULL);
            if (_coverage_ready)
            {
                ls_map_span_at(ls_laser_positions[_ls_coverage_positions_index])->coverage--; // decrement the oldest reading's span only if we've already filled the ring
            }
            ls_laser_positions[_ls_coverage_positions_index] = ls_stepper_get_position();
            ls_map_span_at(ls_laser_positions[_ls_coverage_positions_index])->coverage++; // increment the current reading's span
            _ls_coverage_positions_index++;
            if (_ls_coverage_positions_index >= LS_COVERAGE_POSITIONS_COUNT)
            {
                _ls_coverage_positions_index = 0;
                _coverage_ready = true;
#ifdef LSDEBUG_COVERAGE_POSITIONS
                ls_buzzer_effect(LS_BUZZER_PLAY_HOME_SUCCESS);
                // don't use ls_debug_printf because we can take/release print_mux too many times too rapidly for queue, effectively deadlocking in our own loop(?)s
                // assert failed: xQueueGenericSend queue.c:832 (pxQueue->pcHead != ((void *)0) || pxQueue->u.xSemaphore.xMutexHolder == ((void *)0) || pxQueue->u.xSemaphore.xMutexHolder == xTaskGetCurrentTaskHandle())
                // Backtrace:0x40081aa2:0x3ffbe4500x400871a9:0x3ffbe470 0x4008ce09:0x3ffbe490 0x40087a93:0x3ffbe5b0 0x400d5b09:0x3ffbe5f0 0x4008a441:0x3ffbe610
                // 0x40081aa2: panic_abort at C:/Users/dave/esp/esp-idf/components/esp_system/panic.c:402
                // 0x400871a9: esp_system_abort at C:/Users/dave/esp/esp-idf/components/esp_system/esp_system.c:128
                // 0x4008ce09: __assert_func at C:/Users/dave/esp/esp-idf/components/newlib/assert.c:85
                // 0x40087a93: xQueueGenericSend at C:/Users/dave/esp/esp-idf/components/freertos/queue.c:830 (discriminator 8)
                // 0x400d5b09: ls_coverage_debug_task at R:/URI_Projects/laser-scarecrow/ls2022_esp32/main/debug.c:27
                // 0x4008a441: vPortTaskWrapper at C:/Users/dave/esp/esp-idf/components/freertos/port/xtensa/port.c:131
                // ...no, guess that wasn't quite it, but I was able to avoid the problem by increasing the number of ticks xSemaphoneTake is given here (1=>10) and in ls_debug_printf (1=>5)
                // Sometimes not all of these will print, but at leasst it doesn't panic.
                xSemaphoreTake(print_mux, 10);
                printf("\nCoverage Positions:\n");
                for (int i = 0; i < LS_COVERAGE_POSITIONS_COUNT; i += 16)
                {
                    for (int j = i; j < i + 16; j++)
                    {
                        printf("%4d ", ls_laser_positions[j]);
                    }
                    printf("\n");
                }
                xSemaphoreGive(print_mux);
#endif
            }
        }
        vTaskDelay(pdMS_TO_TICKS(LS_COVERAGE_POSITIONS_MS));
    }
}

#ifdef LSDEBUG_COVERAGE_MEASURE
// https://esp32.com/viewtopic.php?t=5793 for how to read state of OUTPUT pin
void ls_coverage_debug_task(void *pvParameter)
{
    int gpio = lsgpio_laserpowerenable();
    BaseType_t outreg = gpio < 32 ? GPIO_OUT_REG : GPIO_OUT1_REG;
    gpio_num_t pin = (gpio_num_t)(gpio & 0x1F);
    int state = 0;
    ls_debug_printf("Beginning LSDEBUG_COVERAGE_MEASURE output\n");
    while (1)
    {
        // pin is output - read the GPIO_OUT_REG register
        state = (GPIO_REG_READ(outreg) >> pin) & 1U;
        if (1 == state)
        {
            ls_debug_printf("@%d\n", ls_stepper_get_position());
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
#endif