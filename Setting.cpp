#include "Setting.h"
#include "utils.h"
#include <Arduino.h>
#include <EEPROM.h>

char buf[17] = "";
char numBuf[8] = "";
int startAddress = 0;

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
  values = NULL;
  address = (startAddress++) * sizeof(double);
}

Setting::Setting(String name,
                 String *values,
                 double value,
                 double min,
                 double max,
                 bool persist)
    : name(name), value(value), minn(min), maxx(max), values(values),
      slowStep(1), fastStep(1), displayPrecision(0), previous(value),
      persist(persist) {
  address = (startAddress++) * sizeof(double);
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
    if (setValue <= maxx && setValue >= minn) {
      value = setValue;
      previous = setValue;
    } else {
      Serial.print(" resetting bad value!");
      EEPROM.put(address, value);
    }

    Serial.println();
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
  if (values != NULL) {
    String currVal = values[(long)value];
    snprintf(buf, 17, "%s %s           ", name.c_str(), currVal.c_str());
  } else {
    // format number
    char *num = toPrecision(numBuf, 8, value, displayPrecision);
    // display name number with extra spaces to clear the line
    snprintf(buf, 17, "%s %s           ", name.c_str(), num);
  }
  return String(buf);
}

void Setting::handleUpdate() {
  if (persist && previous != value) {
    Serial.print("Updating ");
    Serial.print(name);
    Serial.print(" address ");
    Serial.print(address);
    Serial.print(" val ");
    Serial.print(value);
    Serial.print(" prev ");
    Serial.println(previous);
    EEPROM.put(address, value);
    previous = value;
  }
}
