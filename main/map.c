/*
    Control software for URI Laser Scarecrow, 2022 Model
    Copyright (C) 2022-2024 David H. Brown

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
#include "buzzer.h"
#include "config.h"
#include "leds.h"
#include "math.h"
#include "stepper.h"
#include "tape.h"
#include "tapemode.h"
#include "util.h"


#define LS_MAP_LOWPITCH 1024
#define LS_MAP_HIGHPITCH 2048
#define LS_MAP_ENTRIES_REQUIRED                                                \
  (LS_STEPPER_STEPS_PER_ROTATION / 32 / LS_MAP_RESOLUTION)
static uint32_t IRAM_ATTR _ls_map_data[LS_MAP_ENTRIES_REQUIRED];

static int32_t _ls_map_all_spans_total_steps = 0;

struct ls_map_SpanNode *_ls_map_span_at_map_reading[LS_MAP_ENTRY_COUNT];
int _stepper_position_to_map_reading(ls_stepper_position_t position) {
  return (position % LS_STEPPER_STEPS_PER_ROTATION) / LS_MAP_RESOLUTION;
}
void _init_map_span_at_map_readings(struct ls_map_SpanNode *first_span) {
  for (int i = 0; i < LS_MAP_ENTRY_COUNT; i++) {
    _ls_map_span_at_map_reading[i] = first_span;
  }
  struct ls_map_SpanNode *current_span = first_span->next;
  while (current_span != first_span) {
    for (int i = current_span->begin; i < current_span->end;
         i += LS_MAP_RESOLUTION) {
      _ls_map_span_at_map_reading[_stepper_position_to_map_reading(i)] =
          current_span;
      // ls_debug_printf("%d => %d\n", i, _stepper_position_to_map_reading(i));
    }
    current_span = current_span->next;
  }
}

// http://www.mathcs.emory.edu/~cheung/Courses/255/Syllabus/1-C-intro/bit-array.html
#define ls_map_set_bit(map_index)                                              \
  (_ls_map_data[(map_index / 32)] |= (1 << (map_index % 32)))
#define ls_map_clear_bit(map_index)                                            \
  (_ls_map_data[(map_index / 32)] &= ~(1 << (map_index % 32)))
#define ls_map_read_bit(map_index)                                             \
  (_ls_map_data[(map_index / 32)] & (1 << (map_index % 32)))

void ls_map_enable_at(ls_stepper_position_t stepper_position) {
  ls_map_set_bit(stepper_position / LS_MAP_RESOLUTION);
}
void ls_map_disable_at(int32_t stepper_position) {
  ls_map_clear_bit(stepper_position / LS_MAP_RESOLUTION);
}
/**
 * @brief return 0 if the laser should be off and 1 if it should be on
 *
 * @param stepper_position
 * @return uint32_t to work in IRAM
 */
uint32_t IRAM_ATTR ls_map_is_enabled_at(int32_t stepper_position) {
  return ((ls_map_read_bit(stepper_position / LS_MAP_RESOLUTION)) > 0) ? 1 : 0;
}

enum ls_map_status_t _ls_map_status = LS_MAP_STATUS_NOT_BUILT;
void ls_map_set_status(enum ls_map_status_t status) { _ls_map_status = status; }
enum ls_map_status_t ls_map_get_status(void) { return _ls_map_status; }

static uint16_t
    _ls_map_raw_adc[LS_STEPPER_STEPS_PER_ROTATION / LS_MAP_RESOLUTION];
static uint16_t _ls_map_histo_bins[LS_MAP_HISTOGRAM_BINCOUNT];
static uint16_t _ls_map_min_adc = 4095;
static uint16_t _ls_map_max_adc = 0;

uint16_t ls_map_min_adc(void) { return _ls_map_min_adc; }
uint16_t ls_map_max_adc(void) { return _ls_map_max_adc; }

void _ls_state_map_build_histogram(uint16_t min_adc, uint16_t max_adc) {
#ifdef LSDEBUG_MAP
  ls_debug_printf("Raw ADC tape readings from %d to %d\n", min_adc, max_adc);
#endif
  for (int i = 0; i < LS_MAP_HISTOGRAM_BINCOUNT; i++) {
    _ls_map_histo_bins[i] = 0;
  }
  for (int i = 0; i < LS_STEPPER_STEPS_PER_ROTATION / LS_MAP_RESOLUTION; i++) {
    int bin = _map(_constrain(_ls_map_raw_adc[i], min_adc, max_adc), min_adc,
                   max_adc, 0, LS_MAP_HISTOGRAM_BINCOUNT - 1);
    _ls_map_histo_bins[bin] += 2;
    if (bin > 0) {
      _ls_map_histo_bins[bin - 1]++;
    }
    if (bin + 1 < LS_MAP_HISTOGRAM_BINCOUNT) {
      _ls_map_histo_bins[bin + 1]++;
    }
  }
#ifdef LSDEBUG_MAP
  const char *histo_bar = "##############################"; // 30 characters
  int max_count_in_bin = 0;
  for (int i = 0; i < LS_MAP_HISTOGRAM_BINCOUNT; i++) {
    if (_ls_map_histo_bins[i] > max_count_in_bin) {
      max_count_in_bin = _ls_map_histo_bins[i];
    }
  }

  for (int i = 0; i < LS_MAP_HISTOGRAM_BINCOUNT; i++) {
    ls_debug_printf("%2d [%4d]: %3d %*.*s\n", i,
                    _map(i, 0, LS_MAP_HISTOGRAM_BINCOUNT - 1, _ls_map_min_adc,
                         _ls_map_max_adc),
                    _ls_map_histo_bins[i],
                    _map(_ls_map_histo_bins[i], 0, max_count_in_bin, 0, 30),
                    _map(_ls_map_histo_bins[i], 0, max_count_in_bin, 0, 30),
                    histo_bar);
  }
#endif
}

void _ls_state_map_build_histogram_get_peaks_edges(int *low_peak_bin,
                                                   int *low_edge_bin,
                                                   int *high_peak_bin,
                                                   int *high_edge_bin) {
  *low_peak_bin = 0;
  for (int i = 1; i < LS_MAP_HISTOGRAM_BINCOUNT / 2; i++) {
    if (_ls_map_histo_bins[i] > _ls_map_histo_bins[*low_peak_bin]) {
      *low_peak_bin = i;
    }
  }
  *low_edge_bin = *low_peak_bin;
  for (int i = *low_peak_bin + 1; i < LS_MAP_HISTOGRAM_BINCOUNT; i++) {
    if (_ls_map_histo_bins[i] > 1 &&
        _ls_map_histo_bins[i] > _ls_map_histo_bins[i - 1] / 3) {
      *low_edge_bin = i;
    } else {
      break;
    }
  }
  //   _ls_map_low_threshold = _map(low_edge_bin, 0, LS_MAP_HISTOGRAM_BINCOUNT -
  //   1, _ls_map_min_adc, _ls_map_max_adc);

  *high_peak_bin = LS_MAP_HISTOGRAM_BINCOUNT - 1;
  for (int i = LS_MAP_HISTOGRAM_BINCOUNT - 2; i > LS_MAP_HISTOGRAM_BINCOUNT / 2;
       i--) {
    if (_ls_map_histo_bins[i] > _ls_map_histo_bins[*high_peak_bin]) {
      *high_peak_bin = i;
    }
  }
  *high_edge_bin = *high_peak_bin;
  for (int i = *high_peak_bin - 1; i >= 0; i--) {
    if (_ls_map_histo_bins[i] > 1 &&
        _ls_map_histo_bins[i] > _ls_map_histo_bins[i + 1] / 3) {
      *high_edge_bin = i;
    } else {
      break;
    }
  }
//    _ls_map_high_threshold = _map(high_edge_bin, 0, LS_MAP_HISTOGRAM_BINCOUNT
//    - 1, _ls_map_min_adc, _ls_map_max_adc);
#ifdef LSDEBUG_MAP
  ls_debug_printf("Low peak in bin %d; low_edge_bin = %d\n", *low_peak_bin,
                  *low_edge_bin);
  ls_debug_printf("High peak in bin %d; high_edge_bin = %d\n", *high_peak_bin,
                  *high_edge_bin);
#endif
}

/**
 * @brief remaps according to specified thresholds and _ls_map_raw_adc[] filled
 * by calls to _ls_state_map_build_read_and_set_map()
 *
 * @param[out] enable_count
 * @param[out] disable_count
 * @param[out] misread_count
 * @param low_threshold
 * @param high_threshold
 */
void _ls_state_map_build_set_map(int *enable_count, int *disable_count,
                                 int *misread_count, uint16_t low_threshold,
                                 uint16_t high_threshold) {
  *enable_count = *disable_count = *misread_count = 0;
  for (int map_index = 0;
       map_index < LS_STEPPER_STEPS_PER_ROTATION / LS_MAP_RESOLUTION;
       map_index++) {
    enum ls_state_map_reading reading = LS_STATE_MAP_READING_MISREAD;
    uint16_t raw_adc = _ls_map_raw_adc[map_index];
    ls_stepper_position_t position = map_index * LS_MAP_RESOLUTION;
    switch (ls_tapemode()) {
    case LS_TAPEMODE_DARK:
    case LS_TAPEMODE_DARK_SAFE:
      if (raw_adc <= low_threshold || raw_adc <= LS_REFLECTANCE_ADC_MAX_LIGHT) {
        reading = LS_STATE_MAP_READING_ENABLE;
      }
      if (raw_adc >= high_threshold || raw_adc >= LS_REFLECTANCE_ADC_MIN_DARK) {
        reading = LS_STATE_MAP_READING_DISABLE;
      }
      break;
    case LS_TAPEMODE_LIGHT:
    case LS_TAPEMODE_LIGHT_SAFE:
      if (raw_adc >= high_threshold || raw_adc >= LS_REFLECTANCE_ADC_MIN_DARK) {
        reading = LS_STATE_MAP_READING_ENABLE;
      }
      if (raw_adc <= low_threshold || raw_adc <= LS_REFLECTANCE_ADC_MAX_LIGHT) {
        reading = LS_STATE_MAP_READING_DISABLE;
      }
      break;
    default:
      // we are ignoring the map, so why are we building a map?
      reading = LS_STATE_MAP_READING_ENABLE;
    }
    switch (reading) {
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
    if (map_index % 40 == 0) {
      printf("map @%4d: ", position);
    }
    printf("%c", ls_map_is_enabled_at(position) ? 'O' : '.');
    if (map_index % 40 == 39) {
      printf("\n");
    }
    xSemaphoreGive(print_mux);
#endif
  } // for each map reading
}

/**
 * @brief Called multiple times as arm rotates, so initialize counts to 0 before
 * first call
 *
 * @param[out] enable_count must be initizlied to 0 before first call
 * @param[out] disable_count must be initizlied to 0 before first call
 * @param[out] misread_count must be initizlied to 0 before first call
 */
void _ls_state_map_build_read_and_set_map(int *enable_count, int *disable_count,
                                          int *misread_count) {
  enum ls_state_map_reading reading = LS_STATE_MAP_READING_MISREAD;
  BaseType_t raw_adc = ls_tape_sensor_read();
  BaseType_t pitch = 1000;
  ls_stepper_position_t position = ls_stepper_get_position();
  ls_stepper_position_t map_index = position / LS_MAP_RESOLUTION;
  if (map_index >= 0 &&
      map_index < LS_STEPPER_STEPS_PER_ROTATION / LS_MAP_RESOLUTION) {
    _ls_map_raw_adc[position / LS_MAP_RESOLUTION] = raw_adc;
  } else {
    printf("Map index out of range for position %d => %d out of %d\n", position,
           map_index, LS_STEPPER_STEPS_PER_ROTATION / LS_MAP_RESOLUTION);
  }
  if (raw_adc > _ls_map_max_adc) {
    _ls_map_max_adc = raw_adc;
  }
  if (raw_adc < _ls_map_min_adc) {
    _ls_map_min_adc = raw_adc;
  }
  switch (ls_tapemode()) {
  case LS_TAPEMODE_DARK:
  case LS_TAPEMODE_DARK_SAFE:
    if (raw_adc <= LS_REFLECTANCE_ADC_MAX_LIGHT) {
      reading = LS_STATE_MAP_READING_ENABLE;
    }
    if (raw_adc >= LS_REFLECTANCE_ADC_MIN_DARK) {
      reading = LS_STATE_MAP_READING_DISABLE;
    }
    break;
  case LS_TAPEMODE_LIGHT:
  case LS_TAPEMODE_LIGHT_SAFE:
    if (raw_adc >= LS_REFLECTANCE_ADC_MIN_DARK) {
      reading = LS_STATE_MAP_READING_ENABLE;
    }
    if (raw_adc <= LS_REFLECTANCE_ADC_MAX_LIGHT) {
      reading = LS_STATE_MAP_READING_DISABLE;
    }
    break;
  default:
    // we are ignoring the map, so why are we building a map?
    reading = LS_STATE_MAP_READING_ENABLE;
  }
  pitch = _map(_constrain(raw_adc, LS_REFLECTANCE_ADC_MAX_LIGHT,
                          LS_REFLECTANCE_ADC_MIN_DARK),
               LS_REFLECTANCE_ADC_MIN_DARK, LS_REFLECTANCE_ADC_MAX_LIGHT,
               LS_MAP_LOWPITCH, LS_MAP_HIGHPITCH);

  ls_leds_rgb(_map(pitch, LS_MAP_LOWPITCH, LS_MAP_HIGHPITCH, 0, 192),
              _map(pitch, LS_MAP_LOWPITCH, LS_MAP_HIGHPITCH, 0, 128),
              _map(pitch, LS_MAP_LOWPITCH, LS_MAP_HIGHPITCH, 255, 0));
  ls_buzzer_tone(pitch);
  switch (reading) {
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
  ls_debug_printf("map @%d: %d [%d]=>%c\n", position, raw_adc, reading,
                  ls_map_is_enabled_at(position) ? 'O' : '.');
#endif
}

/**
 * @brief Calculate the number of steps included in a span
 *
 * @param span
 * @return int32_t
 */
int32_t _ls_map_span_length(struct ls_map_SpanNode *span) {
  return 1                         // length is inclusive of endpoints
         + span->end - span->begin // steps between endpoints
         + (span->begin > span->end ? LS_STEPPER_STEPS_PER_ROTATION
                                    : 0); // in case we span home
}

bool _ls_map_is_step_in_span(ls_stepper_position_t step,
                             struct ls_map_SpanNode *span) {
  return span->end > span->begin ? // is this a normal or wrapped span?
             (step >= span->begin && step <= span->end)
                                 : // normal spans
             (step >= span->begin ||
              step <= span->end); // handle a wrapping span
}

/**
 * @brief Discover the active spans in the tape map
 *
 * The tape mapping must have succeeded.
 *
 * @return int total length of all spans
 */
int ls_map_find_spans() {
  _ls_map_all_spans_total_steps = 0;
  // create the first span with invalid begin/end and loop to itself
  ls_map_span_first =
      (struct ls_map_SpanNode *)malloc(sizeof(struct ls_map_SpanNode));
  ls_map_span_first->begin = ls_map_span_first->end =
      LS_MAP_SPANNODE_INVALID_POSITION;
  ls_map_span_first->prev = ls_map_span_first->next = ls_map_span_first;
  ls_map_span_first->letter = 'A';
  struct ls_map_SpanNode *current_span = ls_map_span_first;
  // check if step 0 is in a span and work backwards from end of map to find the
  // start if so
  bool in_span = (bool)ls_map_is_enabled_at(0);
  if (in_span) {
#ifdef LSDEBUG_MAP
    ls_debug_printf("First span includes position 0... ");
#endif
    ls_map_span_first->begin = ls_map_span_first->end = 0;
    for (ls_stepper_position_t i = LS_STEPPER_STEPS_PER_ROTATION - 1; i >= 0;
         i--) {
      if ((bool)ls_map_is_enabled_at(i)) {
        ls_map_span_first->begin = i;
      } else {
#ifdef LSDEBUG_MAP
        ls_debug_printf("and extends back to position %d.\n",
                        ls_map_span_first->begin);
#endif
        break;
      }
    }
  }
  int stop_at_step =
      in_span ? ls_map_span_first->begin : LS_STEPPER_STEPS_PER_ROTATION;
  // continue from step 1
  for (int i = 1; i < stop_at_step; i++) {
    bool enabled = (bool)ls_map_is_enabled_at(i);

    if (enabled) {
      if (in_span) {
        current_span->end = i;
      } else {
        // need to allocate the next node UNLESS ls_map_span_first is still
        // initialized to LS_MAP_SPANNODE_INVALID_POSITION
        if (LS_MAP_SPANNODE_INVALID_POSITION != ls_map_span_first->begin) {
          struct ls_map_SpanNode *new_span =
              (struct ls_map_SpanNode *)malloc(sizeof(struct ls_map_SpanNode));
          // and link it in at the tail of the DLL
          new_span->prev = current_span;
          current_span->next = new_span;
          ls_map_span_first->prev = new_span;
          new_span->next = ls_map_span_first;
          new_span->letter = current_span->letter + 1;
          // this is now our current span
          current_span = new_span;
        }
        // initialize the current span
#ifdef LSDEBUG_MAP
        ls_debug_printf("Found span %c beginning at step %d\n",
                        current_span->letter, i);
#endif
        current_span->begin = current_span->end = i;
        in_span = true;
      }
    }      // if enabled at this step
    else { // not enabled
      if (in_span) {
        int current_span_length = _ls_map_span_length(current_span);
#ifdef LSDEBUG_MAP
        ls_debug_printf("Found span %d..%d (%d steps long).\n",
                        current_span->begin, current_span->end,
                        current_span_length);
#endif
        _ls_map_all_spans_total_steps += current_span_length;
      } // if ending a span
      in_span = false;
    }
  }
  current_span = ls_map_span_first;
  do {
    // somehow we got a divide by zero exception here 2023-04-01
    /*
Guru Meditation Error: Core  1 panic'ed (IntegerDivideByZero). Exception was
unhandled.

Core  1 register dump:
PC      : 0x400d8fb2  PS      : 0x00060130  A0      : 0x800d6df8  A1      :
0x3ffba600 0x400d8fb2: ls_map_find_spans at
R:/URI_Projects/laser-scarecrow/ls2022_esp32/main/map.c:470 (discriminator 1)

A2      : 0x3ffafe48  A3      : 0x3ffafe48  A4      : 0x3ffafe48  A5      :
0x00000000 A6      : 0x00000000  A7      : 0x00000c78  A8      : 0x000003e8  A9
: 0x00000000 A10     : 0x0000007d  A11     : 0x000001f4  A12     : 0x000009c1
A13     : 0x00000000 A14     : 0x0000000f  A15     : 0x00000000  SAR     :
0x00000011  EXCCAUSE: 0x00000006 EXCVADDR: 0x00000000  LBEG    : 0x4000c2e0 LEND
: 0x4000c2f6  LCOUNT  : 0xffffffff


Backtrace: 0x400d8faf:0x3ffba600 0x400d6df5:0x3ffba620 0x400d6fac:0x3ffba660
0x4008a815:0x3ffba6a0 0x400d8faf: ls_map_find_spans at
R:/URI_Projects/laser-scarecrow/ls2022_esp32/main/map.c:470 (discriminator 1)
*/
    current_span->permil = _ls_map_span_length(current_span) * 1000 /
                           _ls_map_all_spans_total_steps;
    current_span = current_span->next;
  } while (current_span != ls_map_span_first);
  _init_map_span_at_map_readings(ls_map_span_first);
  return _ls_map_all_spans_total_steps;
}

/**
 * @brief Return the span at the indicated step or the first span if  step not
 * in a span
 *
 * @param step
 * @return struct ls_map_SpanNode*
 */
struct ls_map_SpanNode *_ls_map_span_at(ls_stepper_position_t step,
                                        struct ls_map_SpanNode *first_span) {
  return _ls_map_span_at_map_reading[_stepper_position_to_map_reading(step)];
  /*
  step %= LS_STEPPER_STEPS_PER_ROTATION;
  // short-circuit if there is only one span:
  if (first_span->next == first_span || first_span->prev == first_span)
  {
      return first_span;
  }
  struct ls_map_SpanNode *span = first_span;
  while (!_ls_map_is_step_in_span(step, span))
  {
      span = span->next;
  }
  return span;
  */
}

struct ls_map_SpanNode *ls_map_span_at(ls_stepper_position_t step) {
  return _ls_map_span_at(step, ls_map_span_first);
}

struct ls_map_SpanNode *
ls_map_span_next(ls_stepper_position_t step,
                 enum ls_stepper_direction_t direction,
                 struct ls_map_SpanNode *starting_span) {
  // short-circuit if there is only one span:
  if (starting_span->next == starting_span ||
      starting_span->prev == starting_span) {
    return starting_span;
  }
  struct ls_map_SpanNode *current_span = starting_span;
  // check to see whether step is actually in a span
  do {
    if (current_span->begin <= current_span->end &&
        step >= current_span->begin && step <= current_span->end) {
      return current_span; // inside a mid-range span
    }
    if (current_span->begin > current_span->end &&
        (step >= current_span->begin || step <= current_span->end)) {
      return current_span; // inside a wrapped span
    }
    current_span = current_span->next;
  } while (current_span != starting_span);
  // current span is again starting span

  do {
    if (LS_STEPPER_DIRECTION_FORWARD ==
        direction) { // work forwards through list
      if (step > current_span->end && step <= current_span->next->begin) {
        return current_span->next;
      }
      current_span = current_span->next;
    } else { // look backwards through list
      if (step < current_span->begin && step >= current_span->prev->end) {
        return current_span->prev;
      }
      current_span = current_span->prev;
    }
  } while (current_span != starting_span);

  // should not happen?
  return starting_span;
}

ls_stepper_position_t
ls_stepper_random_target_within_span(struct ls_map_SpanNode *span) {
  int32_t span_length = _ls_map_span_length(span);
  // int32_t target = (span->begin + (span_length *
  // (255-_ls_stepper_random_reverse_per255) / 255)) ; add half a random move
  uint32_t random = esp_random();
  // target +=
  //     ((random >> 16) * (ls_settings_get_stepper_random_max() -
  //     LS_STEPPER_MOVEMENT_STEPS_MIN) / 65536) / (((uint8_t)random & 0xFF) >
  //     _ls_stepper_random_reverse_per255 ? -2 : 2);
  // favor edges by squaring the span_length when selecting an offset from the
  // middle??
  double rand0to1 = pow((double)(random >> 16) / 65536.0, 0.5);
  ls_stepper_position_t target =
      (span->begin + span_length / 2) +
      (random & 1 ? 1 : -1) *
          (int32_t)floor((double)span_length / 2.0 * rand0to1);
  // int32_t target = next_span->begin + (random >> 16) * span_length / 65536;
  // // any point within span
  //  constrain to 0..STEPS_PER_ROTATION
  target = target % LS_STEPPER_STEPS_PER_ROTATION;
  return target;
}

void ls_stepper_random_move_within_current_span(
    struct ls_stepper_move_t *move) {
  ls_stepper_position_t current_position = ls_stepper_get_position();
  struct ls_map_SpanNode *span =
      _ls_map_span_at(current_position, ls_map_span_first);
  int32_t span_length = _ls_map_span_length(span);
  uint32_t random = esp_random();
  uint32_t min_steps = 1 + span_length / 20;
  int32_t span_percent = 100 * span_length / _ls_map_all_spans_total_steps;
  // steps of half the span length cover a single span well but favor shorter
  // spans increasing max_steps for shorter spans should avoid this bias max
  // steps: 100% span => 0.5x; 50% span => 0.57x; 20% span => .63x
  uint32_t max_steps = span_length * 100 / (150 + span_percent / 2);
  // on longer spans, we want a forward bias to avoid getting "stuck" for too
  // long
  uint8_t fwd_per_255 = (127 + (25 - span_percent / 4));
  move->direction = ((uint8_t)random & 0xFF) > fwd_per_255
                        ? LS_STEPPER_DIRECTION_REVERSE
                        : LS_STEPPER_DIRECTION_FORWARD;
  move->steps = min_steps + ((random >> 16) * (max_steps - min_steps) / 65536);
#ifdef LSDEBUG_STEPPER_RANDOM
  ls_debug_printf("RS_MapSpans: moving %s%d within span [%d..%d] -- %d%% fwd; "
                  "%d-%d step range; \n",
                  (move->direction == LS_STEPPER_DIRECTION_FORWARD ? "+" : "-"),
                  move->steps, span->begin, span->end, fwd_per_255 * 100 / 255,
                  min_steps, max_steps);
#endif
  ls_stepper_position_t target = ls_stepper_position_constrained(
      move->direction == LS_STEPPER_DIRECTION_FORWARD
          ? ls_stepper_get_position() + move->steps
          : ls_stepper_get_position() - move->steps);
  if (!ls_map_is_enabled_at(target)) {
#ifdef LSDEBUG_STEPPER_RANDOM
    ls_debug_printf("RS_MapSpans: DISABLED TARGET at %d\n", target);
#endif
    struct ls_map_SpanNode *new_span =
        ls_map_span_next(target, move->direction, ls_map_span_first);
    int32_t new_target = ls_stepper_random_target_within_span(new_span);
    int32_t additional_steps = abs(new_target - target);
    move->steps += additional_steps;
#ifdef LSDEBUG_STEPPER_RANDOM
    ls_debug_printf(
        "RS_MapSpans: NEW TARGET at %d; steps increased by %d to %d\n",
        new_target, additional_steps, move->steps);
#endif
  }
}

void ls_stepper_random_move_within_new_span(struct ls_stepper_move_t *move,
                                            struct ls_map_SpanNode *span) {
  ls_stepper_position_t target = ls_stepper_random_target_within_span(span);
  int32_t steps = target - ls_stepper_get_position();
  move->direction =
      steps < 0 ? LS_STEPPER_DIRECTION_REVERSE : LS_STEPPER_DIRECTION_FORWARD;
  move->steps = abs(steps);
#ifdef LSDEBUG_STEPPER_RANDOM
  ls_debug_printf("RS_MapSpans: Laser SHOULD NOT HAVE BEEN disabled at end of "
                  "random move; moving to %d in next span (%d..%d)\n",
                  target, span->begin, span->end);
#endif
}

/**
 * The logic has been improved here so that
 * ls_stepper_random_move_within_current_span() will check to see whether the
 * target is enabled and if not, extend the move to the position that would be
 * selected by ls_stepper_random_move_within_new_span(); this allows great
 * acceleration as it avoids stopping at the disabled position.
 *
 *
 */
void ls_stepper_random_strategy_map_spans(struct ls_stepper_move_t *move) {
  // if still in a span after move, do a random relative move
  if (ls_map_is_enabled_at(ls_stepper_get_position())) {
    ls_stepper_random_move_within_current_span(move);
    return;
  }
  // ended outside active span, target the next span.
  //    struct ls_map_SpanNode *next_span =
  //    ls_map_span_next(ls_stepper_get_position(), ls_stepper_get_direction(),
  //    ls_map_span_first);
  struct ls_map_SpanNode *next_span = ls_coverage_next_span();
  ls_stepper_random_move_within_new_span(move, next_span);
}

#ifdef LSDEBUG_ENABLE
#ifdef LS_TEST_SPANNODE
void ls_map_test_spannode() {
  ls_debug_printf("Testing spannode functions\n");
  bool passed = true; // && with result of each test
  ls_stepper_position_t step;

  // place a span right in the middle of the step range; this is th eonly span
  // for now
  struct ls_map_SpanNode *mid =
      (struct ls_map_SpanNode *)malloc(sizeof(struct ls_map_SpanNode));
  mid->begin = LS_STEPPER_STEPS_PER_ROTATION * 3 / 8;
  mid->end = LS_STEPPER_STEPS_PER_ROTATION * 5 / 8;
  mid->next = mid->prev = mid;
  ls_debug_printf("Span `mid` (%d-%d): is only span for first tests.\n",
                  mid->begin, mid->end);

  step = LS_STEPPER_STEPS_PER_ROTATION * 2 / 8;
  passed = passed &&
           ls_map_span_next(step, LS_STEPPER_DIRECTION_FORWARD, mid) == mid;
  ls_debug_printf("Single span `mid` is next for step %d moving forward: %s\n",
                  step, passed ? "pass" : "FAIL");

  step = LS_STEPPER_STEPS_PER_ROTATION * 4 / 8;
  passed = passed &&
           ls_map_span_next(step, LS_STEPPER_DIRECTION_FORWARD, mid) == mid;
  ls_debug_printf("Single span `mid` is next for step %d moving forward: %s\n",
                  step, passed ? "pass" : "FAIL");

  step = LS_STEPPER_STEPS_PER_ROTATION * 6 / 8;
  passed = passed &&
           ls_map_span_next(step, LS_STEPPER_DIRECTION_FORWARD, mid) == mid;
  ls_debug_printf("Single span `mid` is next for step %d moving forward: %s\n",
                  step, passed ? "pass" : "FAIL");

  // create a span that wraps around step 0; add it to the list with the mid
  // span.
  struct ls_map_SpanNode *wrap =
      (struct ls_map_SpanNode *)malloc(sizeof(struct ls_map_SpanNode));
  wrap->begin = LS_STEPPER_STEPS_PER_ROTATION * 7 / 8;
  wrap->end = LS_STEPPER_STEPS_PER_ROTATION * 1 / 8;
  wrap->next = wrap->prev = mid;
  mid->next = mid->prev = wrap;
  ls_debug_printf(
      "Span `wrap` (%d-%d) inserted before mid for next first tests.\n",
      wrap->begin, wrap->end);

  step = LS_STEPPER_STEPS_PER_ROTATION * 4 / 8;
  passed = passed &&
           ls_map_span_next(step, LS_STEPPER_DIRECTION_FORWARD, wrap) == mid;
  ls_debug_printf(
      "Two spans (wrap and mid): mid is next for step %d moving forward: %s\n",
      step, passed ? "pass" : "FAIL");
  step = LS_STEPPER_STEPS_PER_ROTATION * 2 / 8;
  passed = passed &&
           ls_map_span_next(step, LS_STEPPER_DIRECTION_FORWARD, wrap) == mid;
  ls_debug_printf(
      "Two spans (wrap and mid): mid is next for step %d moving forward: %s\n",
      step, passed ? "pass" : "FAIL");
  passed = passed &&
           ls_map_span_next(step, LS_STEPPER_DIRECTION_REVERSE, wrap) == wrap;
  ls_debug_printf("Two spans (wrap and mid): wrap is next for step %d moving "
                  "backward: %s\n",
                  step, passed ? "pass" : "FAIL");
  step = LS_STEPPER_STEPS_PER_ROTATION * 6 / 8;
  passed = passed &&
           ls_map_span_next(step, LS_STEPPER_DIRECTION_FORWARD, wrap) == wrap;
  ls_debug_printf(
      "Two spans (wrap and mid): wrap is next for step %d moving forward: %s\n",
      step, passed ? "pass" : "FAIL");
  passed = passed &&
           ls_map_span_next(step, LS_STEPPER_DIRECTION_REVERSE, wrap) == mid;
  ls_debug_printf(
      "Two spans (wrap and mid): mid is next for step %d moving backward: %s\n",
      step, passed ? "pass" : "FAIL");
  vTaskDelay(1);
  ls_debug_printf("Initializing position->span lookup...\n");
  _init_map_span_at_map_readings(wrap);
  step = LS_STEPPER_STEPS_PER_ROTATION * 0 / 8;
  passed = passed && _ls_map_span_at(step, wrap) == wrap;
  ls_debug_printf("Two spans (wrap and mid): wrap is the span at step %d: %s\n",
                  step, passed ? "pass" : "FAIL");
  step = LS_STEPPER_STEPS_PER_ROTATION * 4 / 8;
  passed = passed && _ls_map_span_at(step, wrap) == mid;
  ls_debug_printf("Two spans (wrap and mid): mid is the span at step %d: %s\n",
                  step, passed ? "pass" : "FAIL");
  step = LS_STEPPER_STEPS_PER_ROTATION * 15 / 16;
  passed = passed && _ls_map_span_at(step, wrap) == wrap;
  ls_debug_printf("Two spans (wrap and mid): wrap is the span at step %d: %s\n",
                  step, passed ? "pass" : "FAIL");

  ls_debug_printf("Spannode testing summary: %s\n", passed ? "pass" : "FAIL");
  free(mid);
  free(wrap);
}
#endif
#endif