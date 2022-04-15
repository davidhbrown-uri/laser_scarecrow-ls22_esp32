#include "map.h"
#include "tape.h"
#include "config.h"

// IRAM supports only 32-bit reads/writes or LoadStoreError panic
static uint32_t IRAM_ATTR _ls_map_ignore = false;

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
bool IRAM_ATTR ls_map_is_enabled_at(int32_t stepper_position)
{
    return _ls_map_ignore || (ls_map_read_bit(stepper_position / LS_MAP_RESOLUTION)) > 0;
}
void ls_map_ignore(void)
{
    _ls_map_ignore = true;
}