/*
  LiquidCrystal Library - Hello World

 Demonstrates the use a 16x2 LCD display.  The LiquidCrystal
 library works with all LCD displays that are compatible with the
 Hitachi HD44780 driver. There are many of them out there, and you
 can usually tell them by the 16-pin interface.

 This sketch prints "Hello World!" to the LCD
 and shows the time.

  The circuit:
 * LCD RS pin to digital pin 12
 * LCD Enable pin to digital pin 11
 * LCD D4 pin to digital pin 5
 * LCD D5 pin to digital pin 4
 * LCD D6 pin to digital pin 3
 * LCD D7 pin to digital pin 2
 * LCD R/W pin to ground
 * LCD VSS pin to ground
 * LCD VCC pin to 5V
 * 10K resistor:
 * ends to +5V and ground
 * wiper to LCD VO pin (pin 3)

 Library originally added 18 Apr 2008
 by David A. Mellis
 library modified 5 Jul 2009
 by Limor Fried (http://www.ladyada.net)
 example added 9 Jul 2009
 by Tom Igoe
 modified 22 Nov 2010
 by Tom Igoe
 modified 7 Nov 2016
 by Arturo Guadalupi

 This example code is in the public domain.

 http://www.arduino.cc/en/Tutorial/LiquidCrystalHelloWorld

*/

// include the library code:
#include <LiquidCrystal.h>
#include <Wire.h>
#include <SHTSensor.h>

#define HUMIDITY_THRESHOLD 55
#define PIN_HW_ENABLE_n 8
#define SWITCH_PIN 9
#define LED_PIN LED_BUILTIN
#define CHILLER_SHUTOFF_COMMAND "CA@ 00000"

// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

SHTSensor sht;

int ledState = LOW;         // the current state of the output pin
int buttonState;             // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin

unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

void setup() {

  pinMode(PIN_HW_ENABLE_n, OUTPUT);
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  digitalWrite(LED_BUILTIN, ledState);

  Wire.begin();
  Serial.begin(9600);

  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
//  delay(1000); // let serial console settle

  lcd.display();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("   deftDevise");
  lcd.setCursor(0,1);
  lcd.print("Firmware ver 1.0");
  delay(4000);
//  lcd.display();
//  lcd.clear();
//  lcd.setCursor(0,0);
//  lcd.print("   deftDevise");
//  delay(1000);
//  lcd.display();
//  lcd.clear();
//  lcd.setCursor(0,0);
//  lcd.print("   deftDevise");
//  delay(1000);
//  lcd.display();
//  lcd.clear();
//  lcd.setCursor(0,0);
//  lcd.print("   deftDevise");
//  delay(1000);
//  lcd.display();
//  lcd.clear();
//  lcd.setCursor(0,0);
//  lcd.print("   deftDevise");
//  delay(1000);
 
  if (sht.init()) {
//      Serial.print("init(): success\n");
//      lcd.display();
//      lcd.clear();
//      lcd.print("init(): success");
  } else {
//      Serial.print("init(): failed\n");
      lcd.display();
      lcd.clear();
      lcd.print("init(): failed");
      while (1) {}
  }
  sht.setAccuracy(SHTSensor::SHT_ACCURACY_MEDIUM); // only supported by SHT3x

  displayInit();
}

float t, h;
boolean isHumidityOk=true, isHumidityOkPrevious, fault, hwEnableState;

void loop() {

  switchOps();
  
  if (sht.readSample()) {
    t = sht.getTemperature();
    h = sht.getHumidity();
    if (h > HUMIDITY_THRESHOLD) {
      isHumidityOk = false;
      fault = true;
      Serial.println(CHILLER_SHUTOFF_COMMAND);
      
//      hwEnableState = !buttonState && !fault;
//      ledState = buttonState;
//      digitalWrite(PIN_HW_ENABLE_n, hwEnableState);
    } //else isHumidityOk = true; IF FAULT KEEP ALERT MESSAGE UP
    if (isHumidityOk != isHumidityOkPrevious) {
      if (isHumidityOk) displayInit();
      else displayAlertInit();
    }
    displayUpdate();

    isHumidityOkPrevious = isHumidityOk;
    } else {
//      Serial.print("Error in readSample()\n");
      lcd.display();
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("*SENSOR FAILURE*");
  }

  delay(1000);

}

void displayChillerAlertInit() {
  lcd.display();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("* CHILLER PUMP *");
  lcd.setCursor(0,1);
  lcd.print("  MUST BE ON!!! ");
  delay(3000);
}

void displayInit() {
  lcd.display();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Humidity:      %");
  lcd.setCursor(0,1);
//  lcd.print(F("Temp          \xDF""C"));
  lcd.print("Threshold:     %");
  lcd.setCursor(13,1);
  lcd.print(HUMIDITY_THRESHOLD);
}

void displayAlertInit() {
  lcd.display();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("*** HUMIDITY ***");
  lcd.setCursor(0,1);
  lcd.print("**** ALERT *****");
}

void displayUpdate() {
  static boolean toggle = false;
  if (isHumidityOk) {
    lcd.setCursor(10, 0);
    lcd.print(h);
//    lcd.setCursor(9, 1);
//    lcd.print(t);
    if (fault) {
      lcd.setCursor(14, 0);
      if (toggle) {
        lcd.print("*");
      }
      else {
        lcd.print(" ");
      }
      toggle = !toggle;
    }
  } else {
    if (toggle) lcd.display();
    else lcd.noDisplay();
    toggle = !toggle;
  }
}


void switchOps() {
    // read the state of the switch into a local variable:
  int reading = digitalRead(SWITCH_PIN);

  // check to see if you just pressed the button
  // (i.e. the input went from LOW to HIGH), and you've waited long enough
  // since the last press to ignore any noise:

  // If the switch changed, due to noise or pressing:
  if (reading != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading != buttonState) {
      buttonState = reading;

      // only toggle the LED if the new button state is HIGH
      if (buttonState == HIGH) {
        ledState = HIGH;
      }
      else {
        ledState = LOW;
        displayChillerAlertInit();
        displayInit();
      }
    }
  }

  // set the LED:
//  digitalWrite(LED_PIN, ledState && !fault);
//  digitalWrite(PIN_HW_ENABLE_n, ledState && fault);
  digitalWrite(LED_PIN, ledState || fault);
  digitalWrite(PIN_HW_ENABLE_n, ledState || fault);

  // save the reading. Next time through the loop, it'll be the lastButtonState:
  lastButtonState = reading;
}
