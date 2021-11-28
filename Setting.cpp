#include "Setting.h"
#include "utils.h"
#include <Arduino.h>
#include <EEPROM.h>

char numBuf[8] = "";
int startAddress = 0;

Setting::Setting(String name,
                 float value,
                 float min,
                 float max,
                 bool persist,
                 float slowStep,
                 float fastStep,
                 byte displayPrecision)
    : name(name), value(value), minn(min), maxx(max), slowStep(slowStep),
      fastStep(fastStep), displayPrecision(displayPrecision), previous(value),
      persist(persist) {
  values = NULL;
  address = (startAddress++) * sizeof(float);
}

Setting::Setting(String name,
                 String *values,
                 float value,
                 float min,
                 float max,
                 bool persist)
    : name(name), value(value), minn(min), maxx(max), values(values),
      slowStep(1), fastStep(1), displayPrecision(0), previous(value),
      persist(persist) {
  address = (startAddress++) * sizeof(float);
}

void Setting::init() {
  if (persist) {
    float setValue;
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

float Setting::handlePressUp(boolean isLongPress) {
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

float Setting::handlePressDown(boolean isLongPress) {
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

char *Setting::getDisplayString(char *buf, byte len) {
  if (values != NULL) {
    String currVal = values[(long)value];
    snprintf(buf, len, "%s %s           ", name.c_str(), currVal.c_str());
  } else {
    // format number
    char *num = toPrecision(numBuf, 8, value, displayPrecision);
    // display name number with extra spaces to clear the line
    snprintf(buf, len, "%s %s           ", name.c_str(), num);
  }
  return buf;
}

void Setting::handleUpdate() {
  if (persist && previous != value) {
    Serial.print(F("Updating "));
    Serial.print(name);
    Serial.print(F(" address "));
    Serial.print(address);
    Serial.print(F(" val "));
    Serial.print(value);
    Serial.print(F(" prev "));
    Serial.println(previous);
    EEPROM.put(address, value);
    previous = value;
  }
}
