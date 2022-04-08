#include "map.h"
#include "tape.h"
#include "config.h"

#define LS_MAP_BYTES_REQUIRED LS_STEPPER_STEPS_PER_ROTATION / 8
static uint8_t ls_map_data[LS_MAP_BYTES_REQUIRED];

// http://www.mathcs.emory.edu/~cheung/Courses/255/Syllabus/1-C-intro/bit-array.html
#define LS_MAP_SET_BIT(pos) ( ls_map_data[(pos / 8)] |= (1 << (pos % 8)))
#define LS_MAP_CLEAR_BIT(pos) ( ls_map_data[(pos / 8)] &= ~(1 << (pos % 8)))
#define LS_MAP_READ_BIT(pos)    ( ls_map_data[(pos / 8)] & (1 << (pos % 8)) )

void ls_map_enable_at(int32_t stepper_position)
{
    LS_MAP_SET_BIT(stepper_position);
}
void ls_map_disable_at(int32_t stepper_position)
{
    LS_MAP_CLEAR_BIT(stepper_position);
}
bool ls_map_is_enabled_at(int32_t stepper_position)
{
    return (LS_MAP_READ_BIT(stepper_position)) > 0;
}
