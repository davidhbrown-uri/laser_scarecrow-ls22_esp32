#include "map.h"
#include "tape.h"
#include "config.h"

static bool _ls_map_ignore = false;

#define LS_MAP_BYTES_REQUIRED LS_STEPPER_STEPS_PER_ROTATION / 8 / LS_MAP_RESOLUTION
static uint8_t _ls_map_data[LS_MAP_BYTES_REQUIRED];
// http://www.mathcs.emory.edu/~cheung/Courses/255/Syllabus/1-C-intro/bit-array.html
#define LS_MAP_SET_BIT(map_index) ( _ls_map_data[(map_index / 8)] |= (1 << (map_index % 8)))
#define LS_MAP_CLEAR_BIT(map_index) ( _ls_map_data[(map_index / 8)] &= ~(1 << (map_index % 8)))
#define LS_MAP_READ_BIT(map_index)    ( _ls_map_data[(map_index / 8)] & (1 << (map_index % 8)) )

void ls_map_enable_at(int32_t stepper_position)
{
    LS_MAP_SET_BIT(stepper_position / LS_MAP_RESOLUTION);
}
void ls_map_disable_at(int32_t stepper_position)
{
    LS_MAP_CLEAR_BIT(stepper_position / LS_MAP_RESOLUTION);
}
bool ls_map_is_enabled_at(int32_t stepper_position)
{
    return _ls_map_ignore || (LS_MAP_READ_BIT(stepper_position / LS_MAP_RESOLUTION)) > 0;
}
void ls_map_ignore(void)
{
    _ls_map_ignore = true;
}