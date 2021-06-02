#include "Setting.h"
#include "utils.h"
#include <Arduino.h>
#include <EEPROM.h>

char buf[17] = "";
char numBuf[8] = "";
int startAddress;

Setting::Setting(String name,
                 double value,
                 double min,
                 double max,
                 bool persist,
                 double slowStep,
                 double fastStep,
                 int displayPrecision)
    : name(name), value(value), minn(min), maxx(max), slowStep(slowStep),
      fastStep(fastStep), displayPrecision(displayPrecision), previous(value),
      persist(persist) {
  address = startAddress++;
}

void Setting::init() {
  if (persist) {
    double setValue;
    EEPROM.get(address, setValue);
    Serial.print(name);
    Serial.print(" addr ");
    Serial.print(address);
    Serial.print(" setv ");
    Serial.print(setValue);
    Serial.println();
    if (setValue <= maxx && setValue >= minn) {
      value = setValue;
      previous = setValue;
    } else {
      EEPROM.put(address, value);
    }
  }
}

double Setting::handlePressUp(boolean isLongPress) {
  if (isLongPress) {
    value += fastStep;
  } else {
    value += slowStep;
  }
  if (value > maxx) {
    value = maxx;
  }
  return value;
}

double Setting::handlePressDown(boolean isLongPress) {
  previous = value;
  if (isLongPress) {
    value -= fastStep;
  } else {
    value -= slowStep;
  }
  if (value < minn) {
    value = minn;
  }
  return value;
}

String Setting::getDisplayString() {
  // format number
  char *num = toPrecision(numBuf, 8, value, displayPrecision);
  // display name number with extra spaces to clear the line
  snprintf(buf, 17, "%s %s           ", name.c_str(), num);
  return String(buf);
}

void Setting::handleUpdate() {
  if (persist && previous != value) {
    Serial.print("address");
    Serial.print(address);
    Serial.print(" v ");
    Serial.print(value);
    Serial.print(" p ");
    Serial.println(previous);
    EEPROM.put(address, value);
    previous = value;
  }
}
