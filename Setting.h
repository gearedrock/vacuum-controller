#include <Arduino.h>

class Setting {
  public:
    String name;
    double value, minn, maxx, slowStep, fastStep, previous;
    int displayPrecision;
    int address;
    bool persist;
    Setting(String name, double value, double min = 0, double max = 255, bool persist = false, double slowStep = 0.1, double fastStep = 1, int displayPrecision = 1);
    double handlePressUp(boolean isLongPress);
    double handlePressDown(boolean isLongPress);
    String getDisplayString();
    void init();
    void handleUpdate();
};
