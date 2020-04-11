#include "Setting.h"
#include <Arduino.h>
#include <EEPROM.h>

char buf[17] = "";
int startAddress;

Setting::Setting(String name, double value, double min, double max, bool persist, double slowStep, double fastStep, int displayPrecision) :
  name(name), value(value), minn(min), maxx(max), slowStep(slowStep), fastStep(fastStep), displayPrecision(displayPrecision), previous(value), persist(persist)
{
  address = startAddress++;
}

void Setting::init() {
  if (persist) {
    double setValue;
    EEPROM.get(address, setValue);
    Serial.print("ad");
    Serial.print(address);
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
  if (displayPrecision == 0) {
    snprintf(buf, 17, "%s %d           ", name.c_str(), (long)value);
  } else if (displayPrecision == 1) {
    int v = (int)(value * 10);
    snprintf(buf, 17, "%s %d.%d           ", name.c_str(), v / 10, v % 10);
  }
  else {
    int v = (int)(value * 100);
    snprintf(buf, 17, "%s %d.%02d           ", name.c_str(), v / 100, v % 100);
  }
  return String(buf);
}

void Setting::handleUpdate() {
  if (persist && previous != value) {
    Serial.print("address");
    Serial.print(address);
    Serial.print("v");
    Serial.print(value);
    Serial.print("p");
    Serial.println(previous);
    EEPROM.put(address, value);
    previous = value;
  }
}
