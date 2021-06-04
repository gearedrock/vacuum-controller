#include <Arduino.h>

class Setting {
public:
  String name;
  double value, minn, maxx, slowStep, fastStep, previous;
  byte displayPrecision;
  int address;
  bool persist;
  String *values;
  Setting(String name,
          double value,
          double min = 0,
          double max = 255,
          bool persist = false,
          double slowStep = 0.1,
          double fastStep = 1,
          byte displayPrecision = 1);

  Setting(String name,
          // display values
          String *values,
          double value,
          double min = 0,
          // must match values length!
          double max = 255,
          bool persist = false);
  double handlePressUp(boolean isLongPress);
  double handlePressDown(boolean isLongPress);
  String getDisplayString();
  void init();
  void handleUpdate();
};
