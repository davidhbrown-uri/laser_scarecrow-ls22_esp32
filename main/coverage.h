/**
 * @file coverage.h
 * @author David H. Brown
 * @brief Helps "random" movement evently cover desired areas
 * @version 0.1
 * @date 2022-12-26
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "stepper.h"

// record 256 positions where laser was on
#define LS_COVERAGE_POSITIONS_COUNT 256
// check for position every so often
#define LS_COVERAGE_POSITIONS_MS 500
// if this much time has elapsed since the task last was running and the task is started, clear old position data
#define LS_COVERAGE_POSITIONS_INVALID_AFTER_SEC 180 

#ifdef LSDEBUG_COVERAGE_MEASURE
void ls_coverage_debug_task(void *pvParameter);
#endif

// a ring buffer of the most recent positions where the laser was shining
ls_stepper_position_t ls_laser_positions[LS_COVERAGE_POSITIONS_COUNT];

void ls_coverage_task(void *pvParameter);
bool ls_coverage_ready(void);
struct ls_map_SpanNode *ls_coverage_next_span(void);
