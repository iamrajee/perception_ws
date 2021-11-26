#ifndef MASK_UTILS_H
#define MASK_UTILS_H

#include "yolact_ros_msgs/Mask.h"

namespace  yolact_ros_msgs
{
  static inline bool test(const yolact_ros_msgs::Mask &mask, size_t x, size_t y)
  {
    size_t index = y * mask.width + x;
    size_t byte_ind = index / 8;
    size_t bit_ind = 7 - (index % 8); // bitorder 'big'
    return mask.mask[byte_ind] & (1 << bit_ind);
  }
}

#endif // MASK_UTILS_H
