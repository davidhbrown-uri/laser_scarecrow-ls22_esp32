/*
    Control software for URI Laser Scarecrow, 2022 Model
    Copyright (C) 2022  David H. Brown

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
#include "map.h"
#include "tape.h"
#include "config.h"
#include "util.h"
#include "tapemode.h"
#include "stepper.h"
#include "buzzer.h"
#include "math.h"

#define LS_MAP_ENTRIES_REQUIRED (LS_STEPPER_STEPS_PER_ROTATION / 32 / LS_MAP_RESOLUTION)
static uint32_t IRAM_ATTR _ls_map_data[LS_MAP_ENTRIES_REQUIRED];
// http://www.mathcs.emory.edu/~cheung/Courses/255/Syllabus/1-C-intro/bit-array.html
#define ls_map_set_bit(map_index) (_ls_map_data[(map_index / 32)] |= (1 << (map_index % 32)))
#define ls_map_clear_bit(map_index) (_ls_map_data[(map_index / 32)] &= ~(1 << (map_index % 32)))
#define ls_map_read_bit(map_index) (_ls_map_data[(map_index / 32)] & (1 << (map_index % 32)))

void ls_map_enable_at(int32_t stepper_position)
{
    ls_map_set_bit(stepper_position / LS_MAP_RESOLUTION);
}
void ls_map_disable_at(int32_t stepper_position)
{
    ls_map_clear_bit(stepper_position / LS_MAP_RESOLUTION);
}
/**
 * @brief return 0 if the laser should be off and 1 if it should be on
 *
 * @param stepper_position
 * @return uint32_t to work in IRAM
 */
uint32_t IRAM_ATTR ls_map_is_enabled_at(int32_t stepper_position)
{
    return ((ls_map_read_bit(stepper_position / LS_MAP_RESOLUTION)) > 0) ? 1 : 0;
}

enum ls_map_status_t _ls_map_status = LS_MAP_STATUS_NOT_BUILT;
void ls_map_set_status(enum ls_map_status_t status)
{
    _ls_map_status = status;
}
enum ls_map_status_t ls_map_get_status(void)
{
    return _ls_map_status;
}

static uint16_t _ls_map_raw_adc[LS_STEPPER_STEPS_PER_ROTATION / LS_MAP_RESOLUTION];
static uint16_t _ls_map_histo_bins[LS_MAP_HISTOGRAM_BINCOUNT];
static uint16_t _ls_map_min_adc = 4095;
static uint16_t _ls_map_max_adc = 0;

uint16_t ls_map_min_adc(void)
{
    return _ls_map_min_adc;
}
uint16_t ls_map_max_adc(void)
{
    return _ls_map_max_adc;
}

void _ls_state_map_build_histogram(uint16_t min_adc, uint16_t max_adc)
{
#ifdef LSDEBUG_MAP
    ls_debug_printf("Raw ADC tape readings from %d to %d\n", min_adc, max_adc);
#endif
    for (int i = 0; i < LS_MAP_HISTOGRAM_BINCOUNT; i++)
    {
        _ls_map_histo_bins[i] = 0;
    }
    for (int i = 0; i < LS_STEPPER_STEPS_PER_ROTATION / LS_MAP_RESOLUTION; i++)
    {
        int bin = _map(_constrain(_ls_map_raw_adc[i], min_adc, max_adc), min_adc, max_adc, 0, LS_MAP_HISTOGRAM_BINCOUNT - 1);
        _ls_map_histo_bins[bin] += 2;
        if (bin > 0)
        {
            _ls_map_histo_bins[bin - 1]++;
        }
        if (bin + 1 < LS_MAP_HISTOGRAM_BINCOUNT)
        {
            _ls_map_histo_bins[bin + 1]++;
        }
    }
#ifdef LSDEBUG_MAP
    const char *histo_bar = "##############################"; // 30 characters
    int max_count_in_bin = 0;
    for (int i = 0; i < LS_MAP_HISTOGRAM_BINCOUNT; i++)
    {
        if (_ls_map_histo_bins[i] > max_count_in_bin)
        {
            max_count_in_bin = _ls_map_histo_bins[i];
        }
    }

    for (int i = 0; i < LS_MAP_HISTOGRAM_BINCOUNT; i++)
    {
        ls_debug_printf("%2d [%4d]: %3d %*.*s\n", i, _map(i, 0, LS_MAP_HISTOGRAM_BINCOUNT - 1, _ls_map_min_adc, _ls_map_max_adc), _ls_map_histo_bins[i],
                        _map(_ls_map_histo_bins[i], 0, max_count_in_bin, 0, 30), _map(_ls_map_histo_bins[i], 0, max_count_in_bin, 0, 30), histo_bar);
    }
#endif
}

void _ls_state_map_build_histogram_get_peaks_edges(int *low_peak_bin, int *low_edge_bin, int *high_peak_bin, int *high_edge_bin)
{
    *low_peak_bin = 0;
    for (int i = 1; i < LS_MAP_HISTOGRAM_BINCOUNT / 2; i++)
    {
        if (_ls_map_histo_bins[i] > _ls_map_histo_bins[*low_peak_bin])
        {
            *low_peak_bin = i;
        }
    }
    *low_edge_bin = *low_peak_bin;
    for (int i = *low_peak_bin + 1; i < LS_MAP_HISTOGRAM_BINCOUNT; i++)
    {
        if (_ls_map_histo_bins[i] > 1 && _ls_map_histo_bins[i] > _ls_map_histo_bins[i - 1] / 3)
        {
            *low_edge_bin = i;
        }
        else
        {
            break;
        }
    }
    //   _ls_map_low_threshold = _map(low_edge_bin, 0, LS_MAP_HISTOGRAM_BINCOUNT - 1, _ls_map_min_adc, _ls_map_max_adc);

    *high_peak_bin = LS_MAP_HISTOGRAM_BINCOUNT - 1;
    for (int i = LS_MAP_HISTOGRAM_BINCOUNT - 2; i > LS_MAP_HISTOGRAM_BINCOUNT / 2; i--)
    {
        if (_ls_map_histo_bins[i] > _ls_map_histo_bins[*high_peak_bin])
        {
            *high_peak_bin = i;
        }
    }
    *high_edge_bin = *high_peak_bin;
    for (int i = *high_peak_bin - 1; i >= 0; i--)
    {
        if (_ls_map_histo_bins[i] > 1 && _ls_map_histo_bins[i] > _ls_map_histo_bins[i + 1] / 3)
        {
            *high_edge_bin = i;
        }
        else
        {
            break;
        }
    }
//    _ls_map_high_threshold = _map(high_edge_bin, 0, LS_MAP_HISTOGRAM_BINCOUNT - 1, _ls_map_min_adc, _ls_map_max_adc);
#ifdef LSDEBUG_MAP
    ls_debug_printf("Low peak in bin %d; low_edge_bin = %d\n", *low_peak_bin, *low_edge_bin);
    ls_debug_printf("High peak in bin %d; high_edge_bin = %d\n", *high_peak_bin, *high_edge_bin);
#endif
}

/**
 * @brief remaps according to specified thresholds and _ls_map_raw_adc[] filled by calls to _ls_state_map_build_read_and_set_map()
 *
 * @param[out] enable_count
 * @param[out] disable_count
 * @param[out] misread_count
 * @param low_threshold
 * @param high_threshold
 */
void _ls_state_map_build_set_map(int *enable_count, int *disable_count, int *misread_count, uint16_t low_threshold, uint16_t high_threshold)
{
    *enable_count = *disable_count = *misread_count = 0;
    for (int map_index = 0; map_index < LS_STEPPER_STEPS_PER_ROTATION / LS_MAP_RESOLUTION; map_index++)
    {
        enum ls_state_map_reading reading = LS_STATE_MAP_READING_MISREAD;
        uint16_t raw_adc = _ls_map_raw_adc[map_index];
        int32_t position = map_index * LS_MAP_RESOLUTION;
        switch (ls_tapemode())
        {
        case LS_TAPEMODE_BLACK:
        case LS_TAPEMODE_BLACK_SAFE:
            if (raw_adc <= low_threshold || raw_adc <= LS_REFLECTANCE_ADC_MAX_WHITE_BUCKET)
            {
                reading = LS_STATE_MAP_READING_ENABLE;
            }
            if (raw_adc >= high_threshold || raw_adc >= LS_REFLECTANCE_ADC_MIN_BLACK_TAPE)
            {
                reading = LS_STATE_MAP_READING_DISABLE;
            }
            break;
        case LS_TAPEMODE_REFLECT:
        case LS_TAPEMODE_REFLECT_SAFE:
            if (raw_adc >= high_threshold || raw_adc >= LS_REFLECTANCE_ADC_MIN_BLACK_BUCKET)
            {
                reading = LS_STATE_MAP_READING_ENABLE;
            }
            if (raw_adc <= low_threshold || raw_adc <= LS_REFLECTANCE_ADC_MAX_SILVER_TAPE)
            {
                reading = LS_STATE_MAP_READING_DISABLE;
            }
            break;
        default:
            // we are ignoring the map, so why are we building a map?
            reading = LS_STATE_MAP_READING_ENABLE;
        }
        switch (reading)
        {
        case LS_STATE_MAP_READING_ENABLE:
            (*enable_count)++;
            ls_map_enable_at(position);
            break;
        case LS_STATE_MAP_READING_DISABLE:
            (*disable_count)++;
            ls_map_disable_at(position);
            break;
        case LS_STATE_MAP_READING_MISREAD:
            (*misread_count)++;
            ls_map_disable_at(position);
            break;
        default:; // init case only for previous
        }
#ifdef LSDEBUG_MAP
        xSemaphoreTake(print_mux, portMAX_DELAY);
        if (map_index % 40 == 0)
        {
            printf("map @%4d: ", position);
        }
        printf("%c", ls_map_is_enabled_at(position) ? 'O' : '.');
        if (map_index % 40 == 39)
        {
            printf("\n");
        }
        xSemaphoreGive(print_mux);
#endif
    } // for each map reading
}

/**
 * @brief Called multiple times as arm rotates, so initialize counts to 0 before first call
 *
 * @param[out] enable_count must be initizlied to 0 before first call
 * @param[out] disable_count must be initizlied to 0 before first call
 * @param[out] misread_count must be initizlied to 0 before first call
 */
void _ls_state_map_build_read_and_set_map(int *enable_count, int *disable_count, int *misread_count)
{
    enum ls_state_map_reading reading = LS_STATE_MAP_READING_MISREAD;
    BaseType_t raw_adc = ls_tape_sensor_read();
    BaseType_t pitch = 1000;
    BaseType_t position = ls_stepper_get_position();
    BaseType_t map_index = position / LS_MAP_RESOLUTION;
    if (map_index >= 0 && map_index < LS_STEPPER_STEPS_PER_ROTATION / LS_MAP_RESOLUTION)
    {
        _ls_map_raw_adc[position / LS_MAP_RESOLUTION] = raw_adc;
    }
    else
    {
        printf("Map index out of range for position %d => %d out of %d\n",
               position, map_index, LS_STEPPER_STEPS_PER_ROTATION / LS_MAP_RESOLUTION);
    }
    if (raw_adc > _ls_map_max_adc)
    {
        _ls_map_max_adc = raw_adc;
    }
    if (raw_adc < _ls_map_min_adc)
    {
        _ls_map_min_adc = raw_adc;
    }
    switch (ls_tapemode())
    {
    case LS_TAPEMODE_BLACK:
    case LS_TAPEMODE_BLACK_SAFE:
        pitch = _map(_constrain(raw_adc, LS_REFLECTANCE_ADC_MAX_WHITE_BUCKET, LS_REFLECTANCE_ADC_MIN_BLACK_TAPE),
                     LS_REFLECTANCE_ADC_MAX_WHITE_BUCKET, LS_REFLECTANCE_ADC_MIN_BLACK_TAPE, 1024, 2048);
        ;
        if (raw_adc <= LS_REFLECTANCE_ADC_MAX_WHITE_BUCKET)
        {
            reading = LS_STATE_MAP_READING_ENABLE;
        }
        if (raw_adc >= LS_REFLECTANCE_ADC_MIN_BLACK_TAPE)
        {
            reading = LS_STATE_MAP_READING_DISABLE;
        }
        break;
    case LS_TAPEMODE_REFLECT:
    case LS_TAPEMODE_REFLECT_SAFE:
        pitch = _map(_constrain(raw_adc, LS_REFLECTANCE_ADC_MAX_SILVER_TAPE, LS_REFLECTANCE_ADC_MIN_BLACK_BUCKET),
                     LS_REFLECTANCE_ADC_MIN_BLACK_BUCKET, LS_REFLECTANCE_ADC_MAX_SILVER_TAPE, 1024, 2048);
        if (raw_adc >= LS_REFLECTANCE_ADC_MIN_BLACK_BUCKET)
        {
            reading = LS_STATE_MAP_READING_ENABLE;
        }
        if (raw_adc <= LS_REFLECTANCE_ADC_MAX_SILVER_TAPE)
        {
            reading = LS_STATE_MAP_READING_DISABLE;
        }
        break;
    default:
        // we are ignoring the map, so why are we building a map?
        reading = LS_STATE_MAP_READING_ENABLE;
    }
    ls_buzzer_tone(pitch);
    switch (reading)
    {
    case LS_STATE_MAP_READING_ENABLE:
        (*enable_count)++;
        ls_map_enable_at(position);
        break;
    case LS_STATE_MAP_READING_DISABLE:
        (*disable_count)++;
        ls_map_disable_at(position);
        break;
    case LS_STATE_MAP_READING_MISREAD:
        (*misread_count)++;
        ls_map_disable_at(position);
        break;
    default:; // init case only for previous
    }
#ifdef LSDEBUG_MAP
    ls_debug_printf("map @%d: %d [%d]=>%c\n", position, raw_adc, reading, ls_map_is_enabled_at(position) ? 'O' : '.');
#endif
}

/**
 * @brief Discover the active spans in the tape map
 *
 * The tape mapping must have succeeded.
 *
 * @return int length of longest span
 */
int ls_map_find_spans()
{
    int max_span_length = 0;
    // create the first span with invalid begin/end and loop to itself
    ls_map_span_first = (struct ls_map_SpanNode *)malloc(sizeof(struct ls_map_SpanNode));
    ls_map_span_first->begin = ls_map_span_first->end = LS_MAP_SPANNODE_INVALID_POSITION;
    ls_map_span_first->prev = ls_map_span_first->next = ls_map_span_first;
    struct ls_map_SpanNode *current_span = ls_map_span_first;
    // check if step 0 is in a span and work backwards from end of map to find the start if so
    bool in_span = (bool)ls_map_is_enabled_at(0);
    if (in_span)
    {
#ifdef LSDEBUG_MAP
        ls_debug_printf("First span includes position 0... ");
#endif
        ls_map_span_first->begin = ls_map_span_first->end = 0;
        for (int i = LS_STEPPER_STEPS_PER_ROTATION - 1; i >= 0; i--)
        {
            if ((bool)ls_map_is_enabled_at(i))
            {
                ls_map_span_first->begin = i;
            }
            else
            {
#ifdef LSDEBUG_MAP
                ls_debug_printf("and extends back to position %d.\n", ls_map_span_first->begin);
#endif
                break;
            }
        }
    }
    int stop_at_step = in_span ? ls_map_span_first->begin : LS_STEPPER_STEPS_PER_ROTATION;
    // continue from step 1
    for (int i = 1; i < stop_at_step; i++)
    {
        bool enabled = (bool)ls_map_is_enabled_at(i);

        if (enabled)
        {
            if (in_span)
            {
                current_span->end = i;
            }
            else
            {
                // need to allocate the next node UNLESS ls_map_span_first is still initialized to LS_MAP_SPANNODE_INVALID_POSITION
                if (LS_MAP_SPANNODE_INVALID_POSITION != ls_map_span_first->begin)
                {
                    struct ls_map_SpanNode *new_span = (struct ls_map_SpanNode *)malloc(sizeof(struct ls_map_SpanNode));
                    // and link it in at the tail of the DLL
                    new_span->prev = current_span;
                    current_span->next = new_span;
                    ls_map_span_first->prev = new_span;
                    new_span->next = ls_map_span_first;
                    // this is now our current span
                    current_span = new_span;
                }
                // initialize the current span
#ifdef LSDEBUG_MAP
                ls_debug_printf("Found span beginning at step %d\n", i);
#endif
                current_span->begin = current_span->end = i;
                in_span = true;
            }
        } // if enabled at this step
        else
        { // not enabled
            if (in_span)
            {
                // check if this is the longest span
                int current_span_length = current_span->end - current_span->begin + 1;
                if (current_span_length < 0)
                {
                    current_span_length += LS_STEPPER_STEPS_PER_ROTATION;
                }
                if (current_span_length > max_span_length)
                {
                    max_span_length = current_span_length;
                }
#ifdef LSDEBUG_MAP
                ls_debug_printf("Found span of %d steps from %d to %d; longest span is currently %d.\n", current_span_length,
                                current_span->begin, current_span->end, max_span_length);
#endif
            } // if ending a span
            in_span = false;
        }
    }

    if (ls_map_span_first->next == ls_map_span_first && max_span_length < LS_STEPPER_STEPS_PER_ROTATION / 3)
    {
#ifdef LSDEBUG_MAP
        ls_debug_printf("Only one small span; move forward or backwards with roughly equal probability.\n");
#endif
        ls_stepper_set_random_reverse_per255((uint8_t)127);
    }
    ls_stepper_set_move_strategy(ls_stepper_move_strategy_next_span);
    return max_span_length;
}

struct ls_map_SpanNode *ls_map_span_next(int32_t step, enum ls_stepper_direction direction, struct ls_map_SpanNode *starting_span)
{
    // short-circuit if there is only one span:
    if (starting_span->next == starting_span || starting_span->prev == starting_span)
    {
        return starting_span;
    }
    struct ls_map_SpanNode *current_span = starting_span;
    // check to see whether step is actually in a span
    do
    {
        if (current_span->begin <= current_span->end && step >= current_span->begin && step <= current_span->end)
        {
            return current_span; // inside a mid-range span
        }
        if (current_span->begin > current_span->end && (step >= current_span->begin || step <= current_span->end))
        {
            return current_span; // inside a wrapped span
        }
        current_span = current_span->next;
    } while (current_span != starting_span);
    // current span is again starting span

    do
    {
        if (LS_STEPPER_DIRECTION_FORWARD == direction)
        { // work forwards through list
            if (step > current_span->end && step <= current_span->next->begin)
            {
                return current_span->next;
            }
            current_span = current_span->next;
        }
        else
        { // look backwards through list
            if (step < current_span->begin && step >= current_span->prev->end)
            {
                return current_span->prev;
            }
            current_span = current_span->prev;
        }
    } while (current_span != starting_span);

    // should not happen?
    return starting_span;
}

void ls_stepper_move_strategy_next_span(struct ls_stepper_move_t *move)
{
    // if still in a span after move, just do a random move.
    if (ls_map_is_enabled_at(ls_stepper_get_position()))
    {
        ls_stepper_move_strategy_random(move);
        return;
    }
    // ended outside active span, target the next span.
    struct ls_map_SpanNode *next_span = ls_map_span_next(ls_stepper_get_position(), ls_stepper_get_direction(), ls_map_span_first);
    int32_t span_length = next_span->end - next_span->begin + (next_span->begin > next_span->end ? LS_STEPPER_STEPS_PER_ROTATION : 0);
    // int32_t target = (next_span->begin + (span_length * (255-_ls_stepper_random_reverse_per255) / 255)) ;
    // add half a random move
    uint32_t random = esp_random();
    // target +=
    //     ((random >> 16) * (ls_settings_get_stepper_random_max() - LS_STEPPER_MOVEMENT_STEPS_MIN) / 65536)
    //     / (((uint8_t)random & 0xFF) > _ls_stepper_random_reverse_per255 ? -2 : 2);
    // favor edges by squaring the span_length when selecting an offset from the middle??
    double rand0to1 = pow((double)(random >> 16) / 65536.0, 0.5);
    int32_t target = (next_span->begin + span_length / 2) + (random & 1 ? 1 : -1) * (int32_t)floor((double)span_length / 2.0 * rand0to1);
    // int32_t target = next_span->begin + (random >> 16) * span_length / 65536; // any point within span
    //  constrain to 0..STEPS_PER_ROTATION
    target = target % LS_STEPPER_STEPS_PER_ROTATION;
    int32_t steps = target - ls_stepper_get_position();
    move->direction = steps < 0 ? LS_STEPPER_DIRECTION_REVERSE : LS_STEPPER_DIRECTION_FORWARD;
    move->steps = abs(steps);
#ifdef LSDEBUG_STEPPER
    ls_debug_printf("Laser disabled at end of random move; moving to %d in span (%d-%d) ", target, next_span->begin, next_span->end);
#endif
}

#ifdef LS_TEST_SPANNODE
void ls_map_test_spannode()
{
    ls_debug_printf("Testing spannode functions\n");
    bool passed = true; // && with result of each test
    int32_t step;

    // place a span right in the middle of the step range; this is th eonly span for now
    struct ls_map_SpanNode *mid = (struct ls_map_SpanNode *)malloc(sizeof(struct ls_map_SpanNode));
    mid->begin = LS_STEPPER_STEPS_PER_ROTATION * 3 / 8;
    mid->end = LS_STEPPER_STEPS_PER_ROTATION * 5 / 8;
    mid->next = mid->prev = mid;
    ls_debug_printf("Span `mid` (%d-%d): is only span for first tests.\n", mid->begin, mid->end);

    step = LS_STEPPER_STEPS_PER_ROTATION * 2 / 8;
    passed = passed && ls_map_span_next(step, LS_STEPPER_DIRECTION_FORWARD, mid) == mid;
    ls_debug_printf("Single span `mid` is next for step %d moving forward: %s\n", step, passed ? "pass" : "FAIL");

    step = LS_STEPPER_STEPS_PER_ROTATION * 4 / 8;
    passed = passed && ls_map_span_next(step, LS_STEPPER_DIRECTION_FORWARD, mid) == mid;
    ls_debug_printf("Single span `mid` is next for step %d moving forward: %s\n", step, passed ? "pass" : "FAIL");

    step = LS_STEPPER_STEPS_PER_ROTATION * 6 / 8;
    passed = passed && ls_map_span_next(step, LS_STEPPER_DIRECTION_FORWARD, mid) == mid;
    ls_debug_printf("Single span `mid` is next for step %d moving forward: %s\n", step, passed ? "pass" : "FAIL");

    // create a span that wraps around step 0; add it to the list with the mid span.
    struct ls_map_SpanNode *wrap = (struct ls_map_SpanNode *)malloc(sizeof(struct ls_map_SpanNode));
    wrap->begin = LS_STEPPER_STEPS_PER_ROTATION * 7 / 8;
    wrap->end = LS_STEPPER_STEPS_PER_ROTATION * 1 / 8;
    wrap->next = wrap->prev = mid;
    mid->next = mid->prev = wrap;
    ls_debug_printf("Span `wrap` (%d-%d) inserted before mid for next first tests.\n", wrap->begin, wrap->end);

    step = LS_STEPPER_STEPS_PER_ROTATION * 4 / 8;
    passed = passed && ls_map_span_next(step, LS_STEPPER_DIRECTION_FORWARD, wrap) == mid;
    ls_debug_printf("Two spans (wrap and mid): mid is next for step %d moving forward: %s\n", step, passed ? "pass" : "FAIL");
    step = LS_STEPPER_STEPS_PER_ROTATION * 2 / 8;
    passed = passed && ls_map_span_next(step, LS_STEPPER_DIRECTION_FORWARD, wrap) == mid;
    ls_debug_printf("Two spans (wrap and mid): mid is next for step %d moving forward: %s\n", step, passed ? "pass" : "FAIL");
    passed = passed && ls_map_span_next(step, LS_STEPPER_DIRECTION_REVERSE, wrap) == wrap;
    ls_debug_printf("Two spans (wrap and mid): wrap is next for step %d moving backward: %s\n", step, passed ? "pass" : "FAIL");
    step = LS_STEPPER_STEPS_PER_ROTATION * 6 / 8;
    passed = passed && ls_map_span_next(step, LS_STEPPER_DIRECTION_FORWARD, wrap) == wrap;
    ls_debug_printf("Two spans (wrap and mid): wrap is next for step %d moving forward: %s\n", step, passed ? "pass" : "FAIL");
    passed = passed && ls_map_span_next(step, LS_STEPPER_DIRECTION_REVERSE, wrap) == mid;
    ls_debug_printf("Two spans (wrap and mid): mid is next for step %d moving backward: %s\n", step, passed ? "pass" : "FAIL");

    ls_debug_printf("Spannode testing summary: %s\n", passed ? "pass" : "FAIL");
    free(mid);
    free(wrap);
}
#endif