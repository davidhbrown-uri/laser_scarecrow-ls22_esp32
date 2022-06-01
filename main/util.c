#include "util.h"

// https://github.com/arduino/ArduinoCore-API/issues/51#issuecomment-69873889
BaseType_t _map(BaseType_t x, BaseType_t in_min, BaseType_t in_max, BaseType_t out_min, BaseType_t out_max)
{
  if ((in_max - in_min) > (out_max - out_min)) {
    return (x - in_min) * (out_max - out_min+1) / (in_max - in_min+1) + out_min;
  }
  else
  {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }
}

BaseType_t _constrain(BaseType_t x, BaseType_t min, BaseType_t max)
{
    if (x < min) { return min; }
    if (x > max) { return max; }
    return x;
}
BaseType_t _difference_exceeds_threshold(BaseType_t previous, BaseType_t current, BaseType_t threshold)
{
  if ((previous - current) > threshold) return 1;
  if ((current - previous) > threshold) return 1;
  return 0;
}


/**
 * @brief Scales a linear input to resolve lower values more clearly:
 * out=(value^2)/(2^bits) where bits = the maximum number of bits in the input. 
 * (Like log2(value) but integer math.)
 * 
 * @param value to be rescaled
 * @param bits number of bits in the input range, e.g., 8 for values 0-255; 10 for 0-1023; 12 for 0-4095
 * @return uint16_t 
 */
uint16_t _make_log_response(uint16_t value, uint8_t bits)
{
    return (uint16_t) (((uint32_t) value * (uint32_t) value) >> bits);
}