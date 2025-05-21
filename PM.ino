#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 7
#define TEMP_THRESHOLD 28
#define TEMP_RESOLUTION 11
#define INTERRUPT_PIN 2

int start, end;

const byte interruptPin = 2;
LiquidCrystal_I2C lcd(0x27, 16, 2);
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

volatile bool manualTrigger = false;
volatile unsigned long lastInterruptTime = 0;

// Cooldown pentru detectie respiratie
unsigned long breathCooldown = 3000;

// Ultima temperatura inregistrata pentru a detecta expiratie
float lastTemp = 0;

// Incalzire senzor
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

  pinMode(4, OUTPUT);   // Verde
  pinMode(10, OUTPUT);  // Galben
  pinMode(12, OUTPUT);  // Rosu
  
  // Activare pull-up buton + intrerupere
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), buttonISR, FALLING);

  // Activare senzor temperatura + setare rezolutie ADC la 11 biti
  sensors.begin();
  sensors.setResolution(TEMP_RESOLUTION);
  sensors.requestTemperatures();
  delay(200);
  lastTemp = sensors.getTempCByIndex(0);

  // Incalzire senzor MQ-6
  warmup();
  // Print initial LCD
  lcd.setCursor(0, 0);
  lcd.print("Astept...");
}

// Intrerupere buton
void buttonISR() {
  unsigned long currentTime = millis();
  if (currentTime - lastInterruptTime > 300) {
    manualTrigger = true;
    lastInterruptTime = currentTime;
  }
}

// Masurare alcoolemie
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
  // Delay pt ADC temperatura
  delay(375);
  float temperature = sensors.getTempCByIndex(0);
  Serial.println(temperature);

  breathCooldown += 300;
  
  // Daca avem intrerupere cu buton sau detectare expiratie
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
