#include "utils.h"
#include <Arduino.h>

char *toPrecision(char *buffer, size_t len, float value, byte precision) {
  if (precision == 0) {
    snprintf(buffer, len, "%d", (int)value);
  } else if (precision == 1) {
    int v = (int)abs(value * 10);
    snprintf(buffer, len, value < 0 ? "-%d.%d" : "%d.%d", v / 10, v % 10);
  } else {
    int v = (int)abs(value * 100);
    snprintf(buffer, len, value < 0 ? "-%d.%02d" : "%d.%02d", v / 100, v % 100);
  }
  return buffer;
}