#include <LiquidCrystal.h>

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

// select the pins used on the LCD panel
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// define some values used by the panel and buttons
int lcd_key     = 0;
int adc_key_in  = 0;
#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5

#define IN_HG_HPA 33.863886666667
#define MAX_PRESSURE 120
#define STD_ATMOS 1013.25

#define enB 11
#define in4 12
#define in3 13
#define MOTOR_MIN 100
#include "Setting.h"
int lastButton = btnNONE;
unsigned long pressStart = 0;
unsigned long currentPressStart = 0;
bool wasPressed = false;
bool wasPressedLong = false;
bool running = false;
Setting pressure = Setting("Pressure", 0, 0, 12);
Setting Kp = Setting(String("Kp"), 2);
Setting Ki = Setting(String("Ki"), 5);
Setting Kd = Setting(String("Kd"), 1);
Setting *settings[] = {&pressure, &Kp, &Ki, &Kd};
int currentSetting = 0;
int numSettings = 4;

#include <PID_v1.h>

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, Kp.value, Ki.value, Kd.value, REVERSE);

// read the buttonss
int read_LCD_buttons()
{
  adc_key_in = analogRead(0);      // read the value from the sensor
  // my buttons when read are centered at these valies: 0, 144, 329, 504, 741
  // we add approx 50 to those values and check to see if we are close
  if (adc_key_in > 1000) return btnNONE; // We make this the 1st option for pressure reasons since it will be the most likely result

  // For V1.0 comment the other threshold and use the one below:

  if (adc_key_in < 50)   return btnRIGHT;
  if (adc_key_in < 195)  return btnUP;
  if (adc_key_in < 380)  return btnDOWN;
  if (adc_key_in < 555)  return btnLEFT;
  if (adc_key_in < 790)  return btnSELECT;



  return btnNONE;  // when all others fail, return this...
}

void setup()
{
  lcd.begin(16, 2);              // start the library
  lcd.setCursor(0, 0);
  lcd.print("Push the buttons"); // print a simple message
  if (!bmp.begin(BMP280_ADDRESS_ALT )) {
    lcd.print(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1) delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X2,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X2,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_250); /* Standby time. */

  // bmp_temp->printSensorDetails();

  // configure motor driver
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  // pid control
  Input = STD_ATMOS;
  Setpoint = Input;
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255);

  Serial.begin(9600);
  Serial.println(F("BMP280 Sensor event test"));
}

void loop()
{
  char buf[17] = "";
  // read sensor
  sensors_event_t temp_event, pressure_event;
  bmp_pressure->getEvent(&pressure_event);
  lcd.setCursor(0, 0);
  long sensorPressure = (long)(pressure_event.pressure / IN_HG_HPA * 100);
  snprintf(buf, 17, "Curr: %02d.%02d\" %s        ", sensorPressure / 100, sensorPressure % 100, running ? "*" : "o");
  lcd.print(buf);

  // compute button time

  unsigned long now = millis();

  lcd_key = read_LCD_buttons();  // read the buttons
  if (lcd_key != lastButton) {
    lastButton = lcd_key;
    currentPressStart = pressStart = now;
    wasPressed = false;
  }
  bool isPressedLong = false;
  bool isPressedShort = false;
  if (now - pressStart > 50 && !wasPressed) {
    isPressedShort = wasPressed = true;
  }
  if (now - pressStart > 1500) {
    if (wasPressedLong) {
      unsigned long currentPressLength = now - currentPressStart;
      if (currentPressLength > 250) {
        wasPressedLong = false;
      }
    }
    if (!wasPressedLong) {
      wasPressedLong = isPressedLong = true;
      currentPressStart = now;
    }
  }

  lcd.setCursor(0, 1);
  lcd.print(settings[currentSetting]->getDisplayString());


  switch (lcd_key)               // depending on which button was pushed, we perform an action
  {
    case btnRIGHT:
      {
        if (isPressedShort) {
          currentSetting++;
          if (currentSetting == numSettings) {
            currentSetting = 0;
          }
        }
        break;
      }
    case btnLEFT:
      {
        if (isPressedShort) {
          currentSetting--;
          if (currentSetting == -1) {
            currentSetting = numSettings - 1;
          }
        }
        break;
      }
    case btnUP:
      {
        if (isPressedShort || isPressedLong) {
          settings[currentSetting]->handlePressUp(isPressedLong);
        }
        break;
      }
    case btnDOWN:
      {
        if (isPressedShort || isPressedLong) {
          settings[currentSetting]->handlePressDown(isPressedLong);
        }
        break;
      }
    case btnSELECT:
      {
        if (isPressedShort) {
          running = !running;
        }
        break;
      }
    case btnNONE:
      {
        break;
      }
  }
  Setpoint = STD_ATMOS - (pressure.value * IN_HG_HPA / 10);
  Input = pressure_event.pressure;
  if (isPressedShort || isPressedLong) {
    Serial.println("pid");
    Serial.print("set");
    Serial.print(Setpoint);
    Serial.print(" in ");
    Serial.print(Input);
    Serial.print(" out ");
    Serial.print(Output);
    Serial.print(" run ");
    Serial.print(running);
    Serial.print(" P ");
    Serial.print(pressure.value);
    Serial.print(" Kp ");
    Serial.print(Kp.value);
    Serial.print(" Ki ");
    Serial.print(Ki.value);
    Serial.print(" Kd ");
    Serial.print(Kd.value);
    Serial.println();
  }
  if (running) {
    myPID.SetTunings(Kp.value, Ki.value, Kd.value);
    myPID.Compute();
    analogWrite(enB, Output);
  } else {
    analogWrite(enB, 0);
  }
}
