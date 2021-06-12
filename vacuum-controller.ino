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
byte customChars[][8] = {
    {B00000, B11011, B11011, B11011, B11011, B11011, B11011, B00000},
    {B00000, B01000, B01100, B01110, B01110, B01100, B01000, B00000},
    {0b11111, 0b10001, 0b01110, 0b00100, 0b00100, 0b01010, 0b10101, 0b11111},
    {0b11111, 0b10001, 0b01010, 0b00100, 0b00100, 0b01010, 0b11111, 0b11111},
    {0b01110, 0b11011, 0b11011, 0b11001, 0b11111, 0b11111, 0b01110, 0b00000},
    {0b01110, 0b10101, 0b10101, 0b10111, 0b10001, 0b10001, 0b01110, 0b00000},
    {0b00000, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b00000}};
#define NUM_CHARS 7
#define PAUSE_CHAR '\x01'
#define RUN_CHAR '\x02'
#define HGLASS1_CHAR '\x03'
#define HGLASS1_CHAR '\x04'
#define CLOCK_INVERT_CHAR '\x05'
#define CLOCK_CHAR '\x06'
#define STOP_CHAR '\x07'

// define some values used by the panel and buttons
byte lcd_key = 0;
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
#define MAX_READINGS 5
double readings[MAX_READINGS] = {0, 0, 0, 0, 0};
int currReading = 0;
boolean readingFault = false;

#define enB 11
#define in4 12
#define in3 13
#define MOTOR_MIN 1
#include "Setting.h"
#include "utils.h"
#define SHORT_PRESS_TIME 50
#define LONG_PRESS_TIME 1500
#define VERY_LONG_PRESS_TIME 15000
#define LONG_PRESS_TIMEOUT 250
byte lastButton = btnNONE;
unsigned long pressStart = 0;
unsigned long currentPressStart = 0;
bool wasPressed = false;
bool wasPressedLong = false;
bool isPressedVeryLong = false;
bool running = false;

#define MS_PER_MIN 60000

String modes[] = {"Constant", "Interval", "Int + Ramp", "Waves"};
// set pressure
Setting pressure = Setting("Pressure", 0, 0, 12, true);
Setting rampType = Setting("Mode", modes, 0, 0, 3, true);
Setting intervalTime = Setting("Interval Tm", 10, 0, 100, true, 0.5, 5);
Setting pauseTime = Setting("Pause Time", 2, 0, 100, true, 0.5, 5);
Setting rampPres = Setting("Ramp Pres", 2, 0, 5, true);
Setting rampTime = Setting("Ramp Time", 2, 0, 10, true, 1);
Setting Kp = Setting(String("Kp"), 2);
Setting Ki = Setting(String("Ki"), 5);
Setting Kd = Setting(String("Kd"), 1);
Setting *settings[] = {&pressure,
                       &rampType,
                       &intervalTime,
                       &pauseTime,
                       &rampPres,
                       &rampTime,
                       &Kp,
                       &Ki,
                       &Kd};
byte currentSetting = 0;
#define numSettings 9

enum IntervalState {
  INTERVAL_START,
  IN_INTERVAL,
  PAUSED,
};
unsigned long intervalStart = 0;
IntervalState intervalState = INTERVAL_START;
byte intervalCount = 0;
double currentRampPressure = 0;
#define INTERVAL_MODE 1.0
#define INTERVAL_RAMP_MODE 2.0
#define WAVES_MODE 3.0

#include <PID_v1.h>

// Define Variables we'll be connecting to
double Setpoint, Input, Output;
float motorAvg = 255;
#define NUM_AVG 10
#define MOTOR_STABLE_AMOUNT 60

// Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, Kp.value, Ki.value, Kd.value, REVERSE);

// read the buttonss
byte read_LCD_buttons() {
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
  lcd.begin(16, 2); // start the library
  // upload custom chars
  for (byte i = 0; i < NUM_CHARS; i++) {
    lcd.createChar(i + 1, customChars[i]);
  }
  // set cursor to move out of programming mode
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
  for (byte i = 0; i < numSettings; ++i) {
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
  for (byte i = 1; i < MAX_READINGS; ++i) {
    if (readings[0] != readings[i]) {
      return false;
    }
  }
  // if all of the readings are the same something is
  // wrong (there should be some natural variance)
  Serial.println("pressure bad");
  return true;
}

void printCurrentPressure(float eventPressure, float vacPress, bool running) {
#define bufLen 17
#define numLen 5
  char buf[bufLen] = "";
  char vac[numLen] = "";
  char time[numLen] = "";
  toPrecision(vac, numLen, vacPress, 2);
#define NUM_STATE_CHARS 7
  char state[NUM_STATE_CHARS] = "     ";
  if (running && (rampType.value == INTERVAL_MODE ||
                  rampType.value == INTERVAL_RAMP_MODE)) {
    double curr = (millis() - intervalStart) / ((double)MS_PER_MIN);
    char x = ' ';
    switch (intervalState) {
      case PAUSED:
        snprintf(state,
                 NUM_STATE_CHARS,
                 "%d%c%s   ",
                 intervalCount,
                 HGLASS1_CHAR,
                 toPrecision(time, 6, pauseTime.value - curr, 1));
        break;
      case INTERVAL_START:
        x = millis() % 1000 < 500 ? CLOCK_CHAR : CLOCK_INVERT_CHAR;
        snprintf(state, NUM_STATE_CHARS, "%d%c     ", intervalCount, x);
        break;
      case IN_INTERVAL:
        snprintf(state,
                 NUM_STATE_CHARS,
                 "%d%c%s   ",
                 intervalCount,
                 CLOCK_CHAR,
                 toPrecision(time, numLen, intervalTime.value - curr, 1));
        break;
    }
  }
  snprintf(buf,
           bufLen,
           "%cCur:%s %s       ",
           (running ? RUN_CHAR : PAUSE_CHAR),
           vac,
           state);
  lcd.setCursor(0, 0);
  lcd.print(buf);
}

void printCurrentSetting() {
  char vac[numLen] = "";
  lcd.setCursor(0, 1);
  String state = settings[currentSetting]->getDisplayString();

  if (rampType.value != 0 && currentSetting == 0 && running) {
    state.trim();
    // add the ramp pressure
    toPrecision(vac, numLen, currentRampPressure, 1, true);
    state += vac;
  }
  lcd.print(state);
}

void updateButtonStates(bool *isPressedLong, bool *isPressedShort) {
  // compute button time
  unsigned long now = millis();

  lcd_key = read_LCD_buttons(); // read the buttons
  // button changed, reset counter
  if (lcd_key != lastButton) {
    lastButton = lcd_key;
    currentPressStart = pressStart = now;
    isPressedVeryLong = false;
    wasPressed = false;
  }
  // we start with the buttons not pressed.  This is to avoid triggering the
  // handler code multiple times per loop
  // so only set pressed short once - ignore if we were already pressed
  if (now - pressStart > SHORT_PRESS_TIME && !wasPressed) {
    *isPressedShort = wasPressed = true;
  }
  if (now - pressStart > LONG_PRESS_TIME) {
    // we were pressed, only "press" again after 250ms
    if (wasPressedLong) {
      unsigned long currentPressLength = now - currentPressStart;
      if (currentPressLength > LONG_PRESS_TIMEOUT) {
        wasPressedLong = false;
      }
    }
    if (!wasPressedLong) {
      // we weren't pressed before, mark as pressed
      wasPressedLong = *isPressedLong = true;
      currentPressStart = now;
    }
  }
  if (now - pressStart > VERY_LONG_PRESS_TIME) {
    // unlike the other states this just lets us know that we've been in whateve
    // "press" state for a long time. this could indicate a button stuck, but
    // more likely we'll just use it for timeouts
    isPressedVeryLong = true;
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
  // read sensor
  sensors_event_t pressure_event;
  bmp_pressure->getEvent(&pressure_event);

  // print current pressure
  float sensorPressure = (pressure_event.pressure / IN_HG_HPA);
  float vacPress =
      ((currentAtmosPressure - pressure_event.pressure) / IN_HG_HPA);
  printCurrentPressure(sensorPressure, vacPress, running);

  printCurrentSetting();

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
        if (currentSetting == 0) {
          currentSetting = numSettings - 1;
        } else {
          currentSetting--;
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
        if (running) {
          intervalStart = millis();
          intervalState = INTERVAL_START;
        } else {
          intervalStart = 0;
        }
      }
      break;
    }
    case btnNONE: {
      if (isPressedLong) {
        // update settings
        for (byte i = 0; i < numSettings; ++i) {
          settings[i]->handleUpdate();
        }
      }
      if (isPressedVeryLong) {
        // go back to initial setting
        currentSetting = 0;
      }
      break;
    }
  }
  double pressureDesired = pressure.value;

  // determine if we're in an interval
  if (intervalStart != 0 && (rampType.value == INTERVAL_MODE ||
                             rampType.value == INTERVAL_RAMP_MODE)) {
    unsigned long curr = millis() - intervalStart;
    // check if waiting for interval to start
    if (intervalState == INTERVAL_START) {
      // check if the motor has stabilized yet (or very close to)
      // Serial.print(Input);
      // Serial.print(" ");
      // Serial.print(Setpoint);
      // Serial.print(" ");
      // Serial.println(offset);
      if (motorAvg < MOTOR_STABLE_AMOUNT) {
        // start interval
        intervalState = IN_INTERVAL;
        intervalCount++;
      } else {
        // we're not there yet, advance start time
        intervalStart = millis();
      }
    }
    // check if time has advanced to next start
    if ((intervalState == PAUSED && curr > pauseTime.value * MS_PER_MIN) ||
        (intervalState == IN_INTERVAL &&
         curr > intervalTime.value * MS_PER_MIN)) {
      // flip state and reset timer
      if (intervalState == PAUSED) {
        intervalState = INTERVAL_START;
      } else {
        intervalState = PAUSED;
      }
      intervalStart = millis();
    }
    if (intervalState == IN_INTERVAL && rampType.value == INTERVAL_RAMP_MODE) {
      if (intervalCount > rampTime.value) {
        // use the set pressure
        currentRampPressure = rampPres.value;
      } else {
        // slowly ramp the pressure during the interval.
        // the fraction that will be increased per interval
        double intervalFraction = (rampPres.value / rampTime.value);
        // % of current interval + 1 fraction/completed interval
        currentRampPressure =
            ((curr) / (intervalTime.value * MS_PER_MIN)) * intervalFraction +
            (intervalFraction * (intervalCount - 1));
      }
      pressureDesired += currentRampPressure;
    }
  }

  if (running && rampType.value == WAVES_MODE) {
    currentRampPressure = sin(millis() / 1000) * 1.5;
    pressureDesired += currentRampPressure;
  }

  // update setpoint with desired pressure
  Setpoint = currentAtmosPressure - ((pressureDesired)*IN_HG_HPA);
  Input = pressure_event.pressure;
  // check if we're in paused interval
  boolean isIntervalPaused = intervalStart != 0 && intervalState == PAUSED;

  if (true && (isPressedShort || isPressedLong)) {
    // Serial.print(buf);
    Serial.print(" | sensor: ");
    Serial.print(sensorPressure);
    Serial.print(" vacuum: ");
    Serial.print(vacPress);

    Serial.print(" | interval state ");
    Serial.print(intervalState);
    Serial.print(" paused? ");
    Serial.print(isIntervalPaused);
    Serial.print(" start ");
    Serial.print(intervalStart);
    Serial.print(" ");
    Serial.print((millis() - intervalStart) / ((double)MS_PER_MIN));
    Serial.print(" ramp ");
    Serial.print(currentRampPressure);

    // Serial.print(" | set ");
    // Serial.print(Setpoint);
    // Serial.print(" in ");
    // Serial.print(Input);
    // Serial.print(" out ");
    // Serial.print(Output);
    // Serial.print(" run ");
    // Serial.print(running);
    // Serial.print(" motor avg ");
    // Serial.print(motorAvg);
    Serial.print(" | desired ");
    Serial.print(pressureDesired);
    // Serial.print(" P ");
    // Serial.print(pressure.value);
    // Serial.print(" Kp ");
    // Serial.print(Kp.value);
    // Serial.print(" Ki ");
    // Serial.print(Ki.value);
    // Serial.print(" Kd ");
    // Serial.print(Kd.value);
    Serial.println();
  }
  if (running && !isIntervalPaused) {
    myPID.SetTunings(Kp.value, Ki.value, Kd.value);
    myPID.Compute();
    if (Output <= MOTOR_MIN) {
      Output = 0;
    }
    motorAvg = (motorAvg * (NUM_AVG - 1) + Output) / NUM_AVG;
    analogWrite(enB, Output);
  } else {
    analogWrite(enB, 0);
  }
}
