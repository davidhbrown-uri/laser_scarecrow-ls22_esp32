#include "map.h"
#include "tape.h"
#include "config.h"

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