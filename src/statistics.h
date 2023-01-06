#pragma once
#include <stdint.h>


class Statistics {
  public:
      uint32_t count = 0;
      float sum_x = 0.0;
      float sum_x2 = 0.0;

    void reset() {
      count = 0;
      sum_x = 0.0;
      sum_x2 = 0.0;
    }

    void add_reading(float x) {
      count += 1;
      sum_x += x;
      sum_x2 += x*x;
    }

    float mean() {
      return sum_x / count;
    }
};
