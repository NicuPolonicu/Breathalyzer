#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 7
#define TEMP_THRESHOLD 28
#define TEMP_RESOLUTION 11

int start, end;

const byte interruptPin = 2;
LiquidCrystal_I2C lcd(0x27, 16, 2);
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

volatile bool manualTrigger = false;
volatile unsigned long lastInterruptTime = 0;
unsigned long breathCooldown = 3000;  // Start at 3000 to allow immediate first trigger



float lastTemp = 0;

void warmup() {
  for (int i = 0; i < 100; i++) {
    analogRead(A5);
    delay(5);
  }
}

void setup() {
  Serial.begin(9600);
  lcd.init();
  lcd.backlight();

  pinMode(4, OUTPUT);   // Green
  pinMode(10, OUTPUT);  // Yellow (PWM)
  pinMode(12, OUTPUT);  // Red

  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), buttonISR, FALLING);

  sensors.begin();
  sensors.setResolution(TEMP_RESOLUTION); // Set DS18B20 to 11-bit
  sensors.requestTemperatures();
  delay(200);
  lastTemp = sensors.getTempCByIndex(0);


  lcd.setCursor(0, 0);
  warmup();
  lcd.print("Astept...");
}

void buttonISR() {
  unsigned long currentTime = millis();
  if (currentTime - lastInterruptTime > 300) {
    manualTrigger = true;
    lastInterruptTime = currentTime;
  }
}

void measureAlcohol() {
  Serial.println(">>> Measuring alcohol...");
  
  tone(3, 2000, 1000);
  lcd.setCursor(0,0);
  lcd.clear();
  lcd.print("Sufla!");
  delay(500);
  int brightness = 255;

  unsigned long alcMax = 0;
  for (int i = 0; i < 400; i++) {
    unsigned long alc = analogRead(A5);
    float currentBrightness = brightness - (i * 255.0 / 400.0);
    if (currentBrightness < 0) currentBrightness = 0;
    analogWrite(10, currentBrightness);
    if (alcMax < alc) {
      alcMax = alc;
    }
    delay(10);
  }
  analogWrite(10, 0);

  Serial.print("Alcohol level: ");
  Serial.println(alcMax);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Alcohol:");
  lcd.setCursor(9, 0);
  lcd.print(alcMax);

  lcd.setCursor(0, 1);
  if (alcMax > 700) {
    lcd.print("Esti beat!");
    Serial.println("Esti beat!");
    digitalWrite(12, HIGH);
    tone(3, 2000, 3000);
    delay(3000);
    digitalWrite(12, LOW);
  } else {
    lcd.print("Esti curat!");
    Serial.println("Bravo, esti curat!");
    digitalWrite(4, HIGH);
    delay(1000);
    digitalWrite(4, LOW);
  }
  
}

void loop() {

  sensors.requestTemperatures();
  delay(375);
  float temperature = sensors.getTempCByIndex(0);
  Serial.println(temperature);


  // start = millis();
  // lcd.setCursor(0, 0);
  // lcd.print("Temp: ");
  // lcd.print(temperature, 2);
  // lcd.print(" C   ");
  
  // lcd.setCursor(0, 1);
  // lcd.print("Alcool: ...    ");
  // end= millis();
  // Serial.print("2 :");
  // Serial.println(end - start);

  breathCooldown += 300;

  if (manualTrigger || (breathCooldown >= 3000 && temperature - lastTemp > 0.2)) {
    breathCooldown = manualTrigger ? breathCooldown : 0;
    manualTrigger = false;
    measureAlcohol();
    delay(2000);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Astept...");
  }

  lastTemp = temperature;

  digitalWrite(4, LOW);
  digitalWrite(12, LOW);
  analogWrite(10, 0);

}
