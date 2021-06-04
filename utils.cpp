#include "utils.h"
#include <Arduino.h>

char *toPrecision(char *buffer, long len, float value, byte precision) {
  if (precision == 0) {
    snprintf(buffer, len, "%d", (long)value);
  } else if (precision == 1) {
    int v = (int)(value * 10);
    snprintf(buffer, len, "%d.%d", v / 10, v % 10);
  } else {
    int v = (int)(value * 100);
    snprintf(buffer, len, "%d.%02d", v / 100, v % 100);
  }
  return buffer;
}