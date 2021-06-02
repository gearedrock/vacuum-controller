#include <Adafruit_BMP280.h>
#include <LiquidCrystal.h>
#include <SPI.h>
#include <Wire.h>
#include <avr/wdt.h>

Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

// select the pins used on the LCD panel
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
byte offChar[] =
    {B00000, B11011, B11011, B11011, B11011, B11011, B11011, B00000};
byte onChar[] =
    {B00000, B01000, B01100, B01110, B01110, B01100, B01000, B00000};

// define some values used by the panel and buttons
int lcd_key = 0;
int adc_key_in = 0;
#define btnRIGHT 0
#define btnUP 1
#define btnDOWN 2
#define btnLEFT 3
#define btnSELECT 4
#define btnNONE 5

#define IN_HG_HPA 33.8639
#define MAX_PRESSURE 120
double currentAtmosPressure = SENSORS_PRESSURE_SEALEVELHPA;
#define MAX_READINGS 10
double readings[MAX_READINGS] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int currReading = 0;
boolean readingFault = false;

#define enB 11
#define in4 12
#define in3 13
#define MOTOR_MIN 1
#include "Setting.h"
#include "utils.h"
int lastButton = btnNONE;
unsigned long pressStart = 0;
unsigned long currentPressStart = 0;
bool wasPressed = false;
bool wasPressedLong = false;
bool running = false;
int interval = 0;
long intervalRemaining = 0;
long pauseRemaining = 0;

Setting pressure = Setting("Pressure", 0, 0, 12, true);
Setting rampType = Setting("Norm/Wave/Ramp", 0, 0, 2, true, 1);
Setting rampPres = Setting("Ramp Pr", 2, 0, 5, true);
Setting intervalTime = Setting("Interval Tm", 10, 0, 100, true, 0.5, 5);
Setting pauseTime = Setting("Pause Tm", 2, 0, 100, true, 0.5, 5);
Setting pausePressure = Setting("Pause Pr", 0, 0, 5, true);
Setting Kp = Setting(String("Kp"), 2);
Setting Ki = Setting(String("Ki"), 5);
Setting Kd = Setting(String("Kd"), 1);
Setting *settings[] = {&pressure,
                       &rampType,
                       &rampPres,
                       &intervalTime,
                       &pauseTime,
                       &pausePressure,
                       &Kp,
                       &Ki,
                       &Kd};
int currentSetting = 0;
#define numSettings 9
int eepromStart = 0;

#include <PID_v1.h>

// Define Variables we'll be connecting to
double Setpoint, Input, Output;

// Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, Kp.value, Ki.value, Kd.value, REVERSE);

// read the buttonss
int read_LCD_buttons() {
  adc_key_in = analogRead(0); // read the value from the sensor
  // my buttons when read are centered at these valies: 0, 144, 329, 504, 741
  // we add approx 50 to those values and check to see if we are close
  if (adc_key_in > 1000)
    return btnNONE; // We make this the 1st option for pressure reasons since
                    // it will be the most likely result

  // For V1.0 comment the other threshold and use the one below:

  if (adc_key_in < 50) return btnRIGHT;
  if (adc_key_in < 195) return btnUP;
  if (adc_key_in < 380) return btnDOWN;
  if (adc_key_in < 555) return btnLEFT;
  if (adc_key_in < 790) return btnSELECT;

  return btnNONE; // when all others fail, return this...
}

void setup() {
  Serial.begin(9600);
  Serial.println(F("BMP280 Sensor event test"));
  lcd.createChar(1, offChar);
  lcd.createChar(2, onChar);
  lcd.begin(16, 2); // start the library
  lcd.setCursor(0, 0);
  lcd.print("Reading pressure");
  while (!bmp.begin(BMP280_ADDRESS_ALT)) {
    delay(100);
    lcd.setCursor(0, 1);
    lcd.print("No Sensor!");
    Serial.println(F("No Sensor!"));
    delay(1000);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X16,    /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X2,       /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_250); /* Standby time. */

  // bmp_temp->printSensorDetails();

  // configure motor driver
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  //  digitalWrite(in3, HIGH);
  //  digitalWrite(in4, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  // pid control
  Input = SENSORS_PRESSURE_SEALEVELHPA;
  Setpoint = Input;
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(MOTOR_MIN, 255);

  // average two pressure readings
  double atmosPressure = bmp.readPressure();
  delay(1200);
  atmosPressure = (atmosPressure + bmp.readPressure()) / 2;
  // set current pressure to reading (plus a bit of fuzz)
  currentAtmosPressure = atmosPressure / 100 + .5;
  // check if reading is more +/- 2 in mercury from std pressure - if so ignore
  // it
  if (currentAtmosPressure >
          SENSORS_PRESSURE_SEALEVELHPA + IN_HG_HPA + IN_HG_HPA ||
      currentAtmosPressure <
          SENSORS_PRESSURE_SEALEVELHPA - IN_HG_HPA - IN_HG_HPA) {
    Serial.println("pressure bad");
    Serial.println(currentAtmosPressure);
    currentAtmosPressure = SENSORS_PRESSURE_SEALEVELHPA;
    lcd.setCursor(0, 1);
    lcd.print("Current pressure bad");
    delay(1000);
  }
  Serial.println(currentAtmosPressure);
  Serial.println("Settings");
  for (int i = 0; i < numSettings; ++i) {
    settings[i]->init();
  }

  // setup watchdog
  wdt_enable(WDTO_1S);
}

boolean checkFault(double sensorPressure) {
  // check pressure fault
  readings[currReading++] = sensorPressure;
  if (currReading > MAX_READINGS) {
    currReading = 0;
  }
  // check for duplicate readings
  for (int i = 1; i < MAX_READINGS; ++i) {
    if (readings[0] != readings[i]) {
      return false;
    }
  }
  // if all of the readings are the same something is
  // wrong (there should be some natural variance)
  Serial.println("pressure bad");
  return true;
}

char *formatCurrentPressure(char *buf,
                            long len,
                            float eventPressure,
                            float vacPress,
                            bool running) {
  char pres[8] = "";
  char vac[8] = "";
  toPrecision(pres, 8, eventPressure, 2);
  toPrecision(vac, 8, vacPress, 1);
  snprintf(
      buf, len, "Curr:%s(%s)%c    ", pres, vac, (running ? '\x02' : '\x01'));
}

void updateButtonStates(bool *isPressedLong, bool *isPressedShort) {
  // compute button time
  unsigned long now = millis();

  lcd_key = read_LCD_buttons(); // read the buttons
  // button changed, reset counter
  if (lcd_key != lastButton) {
    lastButton = lcd_key;
    currentPressStart = pressStart = now;
    wasPressed = false;
  }

  if (now - pressStart > 50 && !wasPressed) {
    *isPressedShort = wasPressed = true;
  }
  if (now - pressStart > 1500) {
    if (wasPressedLong) {
      unsigned long currentPressLength = now - currentPressStart;
      if (currentPressLength > 250) {
        wasPressedLong = false;
      }
    }
    if (!wasPressedLong) {
      wasPressedLong = *isPressedLong = true;
      currentPressStart = now;
    }
  }
  if (false && (*isPressedLong || *isPressedShort)) {
    Serial.print("key");
    Serial.print(lcd_key);
    Serial.print(" short ");
    Serial.print(*isPressedShort);
    Serial.print(" long ");
    Serial.println(*isPressedLong);
  }
}

void loop() {
  char buf[17] = "";
  // read sensor
  sensors_event_t pressure_event;
  bmp_pressure->getEvent(&pressure_event);

  // print current pressure
  lcd.setCursor(0, 0);
  float sensorPressure = (pressure_event.pressure / IN_HG_HPA);
  float vacPress =
      ((currentAtmosPressure - pressure_event.pressure) / IN_HG_HPA);
  formatCurrentPressure(buf, 17, sensorPressure, vacPress, running);
  lcd.print(buf);

  // get button states
  bool isPressedLong = false;
  bool isPressedShort = false;
  updateButtonStates(&isPressedLong, &isPressedShort);

  wdt_reset();
  // every 250ms (button press) check for sane pressures
  if ((isPressedShort || isPressedLong)) {
    readingFault = checkFault(pressure_event.pressure);
    if (readingFault) {
      lcd.setCursor(0, 1);
      lcd.print("No Sensor Change!");
      // stop the motor
      analogWrite(enB, 0);
    }
  }
  if (readingFault) {
    // stop processing on fault
    return;
  }

  lcd.setCursor(0, 1);
  lcd.print(settings[currentSetting]->getDisplayString());

  // depending on which button was pushed, we perform an action
  switch (lcd_key) {
    case btnRIGHT: {
      if (isPressedShort) {
        currentSetting++;
        if (currentSetting == numSettings) {
          currentSetting = 0;
        }
      }
      break;
    }
    case btnLEFT: {
      if (isPressedShort) {
        currentSetting--;
        if (currentSetting == -1) {
          currentSetting = numSettings - 1;
        }
      }
      break;
    }
    case btnUP: {
      if (isPressedShort || isPressedLong) {
        settings[currentSetting]->handlePressUp(isPressedLong);
      }
      break;
    }
    case btnDOWN: {
      if (isPressedShort || isPressedLong) {
        settings[currentSetting]->handlePressDown(isPressedLong);
      }
      break;
    }
    case btnSELECT: {
      if (isPressedShort) {
        running = !running;
      }
      break;
    }
    case btnNONE: {
      if (isPressedLong) {
        // update settings
        for (int i = 0; i < numSettings; ++i) {
          settings[i]->handleUpdate();
        }
      }
      break;
    }
  }
  Setpoint = currentAtmosPressure - (pressure.value * IN_HG_HPA);
  Input = pressure_event.pressure;
  if (false && (isPressedShort || isPressedLong)) {
    Serial.println(buf);

    Serial.print(sensorPressure);
    Serial.print(" ");
    Serial.println(vacPress);

    Serial.print(" set ");
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
    if (Output <= MOTOR_MIN) {
      Output = 0;
    }
    analogWrite(enB, Output);
  } else {
    analogWrite(enB, 0);
  }
}
