#include <Arduino.h>

class Setting {
  public:
    String name;
    double value, minn, maxx, slowStep, fastStep;
    int displayPrecision;
    Setting(String name, double value, double min = 0, double max = 255, double slowStep = 0.1, double fastStep = 1, int displayPrecision = 1);
    double handlePressUp(boolean isLongPress);
    double handlePressDown(boolean isLongPress);
    String getDisplayString();
};
