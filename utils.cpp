#include "utils.h"
#include <Arduino.h>

char *toPrecision(char *buffer,
                  size_t len,
                  float value,
                  byte precision,
                  boolean leadingPlus) {
  if (precision == 0) {
    snprintf(
        buffer, len, (leadingPlus && !(value < 0)) ? "+%d" : "%d", (int)value);
  } else if (precision == 1) {
    int v = (int)abs(value * 10);
    String formatString = "%d.%d";
    if (value < 0) {
      formatString = "-" + formatString;
    } else if (leadingPlus) {
      formatString = "+" + formatString;
    }
    snprintf(buffer, len, formatString.c_str(), v / 10, v % 10);
  } else {
    int v = (int)abs(value * 100);
    String formatString = "%d.%02d";
    if (value < 0) {
      formatString = "-" + formatString;
    } else if (leadingPlus) {
      formatString = "+" + formatString;
    }
    snprintf(buffer, len, formatString.c_str(), v / 100, v % 100);
  }
  return buffer;
}