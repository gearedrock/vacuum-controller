#include <Arduino.h>

class Setting {
public:
  String name;
  float value, minn, maxx, slowStep, fastStep, previous;
  byte displayPrecision;
  int address;
  bool persist;
  String *values;
  Setting(String name,
          float value,
          float min = 0,
          float max = 255,
          bool persist = false,
          float slowStep = 0.1,
          float fastStep = 1,
          byte displayPrecision = 1);

  Setting(String name,
          // display values
          String *values,
          float value,
          float min = 0,
          // must match values length!
          float max = 255,
          bool persist = false);
  float handlePressUp(boolean isLongPress);
  float handlePressDown(boolean isLongPress);
  char *getDisplayString(char *buf, byte len);
  void init();
  void handleUpdate();
};
