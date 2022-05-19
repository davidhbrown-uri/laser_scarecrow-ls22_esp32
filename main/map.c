#include "map.h"
#include "tape.h"
#include "config.h"
#include "util.h"
#include "tapemode.h"
#include "stepper.h"
#include "buzzer.h"

#define LS_MAP_BYTES_REQUIRED (LS_STEPPER_STEPS_PER_ROTATION / 32 / LS_MAP_RESOLUTION)
static uint32_t IRAM_ATTR _ls_map_data[LS_MAP_BYTES_REQUIRED];
// http://www.mathcs.emory.edu/~cheung/Courses/255/Syllabus/1-C-intro/bit-array.html
#define ls_map_set_bit(map_index) ( _ls_map_data[(map_index / 32)] |= (1 << (map_index % 32)))
#define ls_map_clear_bit(map_index) ( _ls_map_data[(map_index / 32)] &= ~(1 << (map_index % 32)))
#define ls_map_read_bit(map_index)    ( _ls_map_data[(map_index / 32)] & (1 << (map_index % 32)) )


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
            ls_debug_printf("map @%4d: ", position);
        }
        ls_debug_printf("%c", ls_map_is_enabled_at(position) ? 'O' : '.');
        if (map_index % 40 == 39)
        {
            ls_debug_printf("\n");
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
    xSemaphoreTake(print_mux, portMAX_DELAY);
    ls_debug_printf("map @%d: %d [%d]=>%c\n", position, raw_adc, reading, ls_map_is_enabled_at(position) ? 'O' : '.');
    xSemaphoreGive(print_mux);
#endif
}