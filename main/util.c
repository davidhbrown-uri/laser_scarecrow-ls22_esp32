#include "util.h"

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
