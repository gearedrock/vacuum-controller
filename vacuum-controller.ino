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

#define enB 11
#define in4 12
#define in3 13
#define MOTOR_MIN 100
int speed = 0;
int lastButton = btnNONE;
unsigned long pressStart = 0;
unsigned long currentPressStart = 0;
bool wasPressed = false;
bool wasPressedLong = false;

// read the buttons
int read_LCD_buttons()
{
  adc_key_in = analogRead(0);      // read the value from the sensor
  // my buttons when read are centered at these valies: 0, 144, 329, 504, 741
  // we add approx 50 to those values and check to see if we are close
  if (adc_key_in > 1000) return btnNONE; // We make this the 1st option for speed reasons since it will be the most likely result

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
}

void loop()
{
  char buf[17] = "";
  // read sensor
  sensors_event_t temp_event, pressure_event;
  bmp_pressure->getEvent(&pressure_event);
  lcd.setCursor(0, 0);
  long pressure = (long)(pressure_event.pressure * 10);
  snprintf(buf, 17, "Press %04d.%1d hPa", pressure / 10, pressure % 10);
  lcd.print(buf);

  lcd.setCursor(9, 1);           // move cursor to second line "1" and 9 spaces over

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

  snprintf(buf, 16, "%2x %d %d", speed, isPressedShort, isPressedLong);
  lcd.print(buf);

  lcd.setCursor(0, 1);           // move to the begining of the second line
  switch (lcd_key)               // depending on which button was pushed, we perform an action
  {
    case btnRIGHT:
      {
        lcd.print("RIGHT ");
        break;
      }
    case btnLEFT:
      {
        lcd.print("LEFT   ");
        break;
      }
    case btnUP:
      {
        lcd.print("UP    ");
        if (isPressedLong) {
          speed += 10;
        } else if (isPressedShort) {
          speed += 1;
        }

        if (speed < MOTOR_MIN) {
          // motor will not start below this
          speed = MOTOR_MIN;
        }
        if (speed > 255) {
          speed = 255;
        }
        break;
      }
    case btnDOWN:
      {
        lcd.print("DOWN  ");
        if (isPressedLong) {
          speed -= 10;
        } else if (isPressedShort) {
          speed -= 1;
        }
        if (speed < MOTOR_MIN) {
          speed = 0;
        }
        break;
      }
    case btnSELECT:
      {
        lcd.print("SELECT");
        break;
      }
    case btnNONE:
      {
        lcd.print("NONE  ");
        break;
      }
  }
  analogWrite(enB, speed);

}
