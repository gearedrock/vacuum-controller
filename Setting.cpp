#include "Setting.h"
#include <Arduino.h>

char buf[17] = "";

Setting::Setting(String name, double value, double min, double max, double slowStep, double fastStep, int displayPrecision) :
  name(name), value(value), minn(min), maxx(max), slowStep(slowStep), fastStep(fastStep), displayPrecision(displayPrecision)
{}

double Setting::handlePressUp(boolean isLongPress){
  if(isLongPress){
    value += fastStep;
  }else{
    value += slowStep;
  }
  if(value>maxx){
    value = maxx;
  }
  return value;
}

double Setting::handlePressDown(boolean isLongPress){
  if(isLongPress){
    value -= fastStep;
  }else{
    value -= slowStep;
  }
  if(value<minn){
    value = minn;
  }
  return value;
}

String Setting::getDisplayString(){
  if(displayPrecision == 0){
    snprintf(buf, 17, "%s %d           ", name.c_str(), (long)value);
  } else if (displayPrecision == 1){
    int v = (int)(value*10);
    snprintf(buf, 17, "%s %d.%d           ", name.c_str(), v/10, v%10);
  }
  else{
    int v = (int)(value*100);
    snprintf(buf, 17, "%s %d.%02d           ", name.c_str(), v/100, v%100);
  }
  return String(buf);
}
