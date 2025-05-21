#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#include <DallasTemperature.h>

#define ONE_WIRE_BUS 7
#define TEMP_THRESHOLD 28
#define TEMP_RESOLUTION 11
#define INTERRUPT_PIN 2
#define MQ6_PIN 0

LiquidCrystal_I2C lcd(0x27, 16, 2);
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

volatile bool manualTrigger = false;
volatile unsigned long lastInterruptTime = 0;

// Cooldown pentru detectie respiratie
unsigned long breathCooldown = 3000;

// Ultima temperatura inregistrata pentru a detecta expiratie
float lastTemp = 0;

// Setare ADC pentru A0
void setupADC() {
  ADMUX = (1 << REFS0) | MQ6_PIN;
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1);
}

// Citire ADC
uint16_t readADC() {
  ADMUX = (ADMUX & 0xF0) | MQ6_PIN;
  ADCSRA |= (1 << ADSC);
  while (ADCSRA & (1 << ADSC));
  return ADC;
}

// Setare PWM pe Timer 1
void setupPWM() {
  DDRB |= (1 << DDB2); // PB2 = Pin 10 = OC1B output
  TCCR1A = (1 << COM1B1) | (1 << WGM10);         // Fast PWM 8-bit
  TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10); // Prescaler 64
}

// Setare duty cycle PWM
void setPWM(uint8_t duty) {
  OCR1B = duty; // 0–255
}

// Oprire completa PWM, ca altfel setPWM(0)
// lasa LED-ul aprins foarte stins
void stopPWM() {
  TCCR1A &= ~(1 << COM1B1);     // Disconnect OC1B (Pin 10) from PWM
  PORTB &= ~(1 << PORTB2);      // Drive it LOW
}


// Setare intrerupere buton
void setupInterrupt() {
  EICRA |= (1 << ISC01); // FALLING EDGE
  EIMSK |= (1 << INT0);  // INT0 pt pin 2
  sei();
}


// Incalzire senzor MQ-6
void warmup() {
  for (int i = 0; i < 100; i++) {
    readADC();
    delayMs(5);
  }
}

// Delay custom folosind Timer0
void delayMs(uint16_t ms) {
  TCCR0A = (1 << WGM01);
  TCCR0B = (1 << CS01) | (1 << CS00);
  OCR0A = 249;
  TCNT0 = 0;             

  for (uint16_t i = 0; i < ms; i++) {
    TIFR0 |= (1 << OCF0A);  // Reset flag
    while (!(TIFR0 & (1 << OCF0A)));
  }

  TCCR0B = 0;
}

void setup() {
  Serial.begin(9600);
  lcd.init();
  lcd.backlight();

  // Setare LED-uri ca output
  DDRD |= (1 << DDD4);   // Verde
  DDRB |= (1 << DDB2);  // Galben
  DDRB |= (1 << DDB4);  // Rosu

  // Activare pull-up buton + intrerupere
  DDRD &= ~(1 << DDD2);
  PORTD |= (1 << PORTD2);
  setupInterrupt();

  // Activare senzor temperatura + setare rezolutie ADC la 11 biti
  sensors.begin();
  sensors.setResolution(TEMP_RESOLUTION);
  sensors.requestTemperatures();
  delayMs(375);
  lastTemp = sensors.getTempCByIndex(0);

  // Incalzire senzor MQ-6 si setup ADC
  setupADC();
  warmup();

  // Print initial LCD
  lcd.setCursor(0, 0);
  lcd.print("Astept...");
}


// Intrerupere buton (cu debouncer)
ISR(INT0_vect) {
  unsigned long currentTime = millis();
  if (currentTime - lastInterruptTime > 300) {
    manualTrigger = true;
    lastInterruptTime = currentTime;
  }
}

// Masurare alcoolemie
void measureAlcohol() {
  Serial.println(">>> Measuring alcohol...");
  
  // Buzzer + print pe LCD
  tone(3, 2000, 800);
  lcd.setCursor(0,0);
  lcd.clear();
  delayMs(300);
  lcd.print("Sufla!");
  int brightness = 255;

  unsigned long alcMax = 0;
  for (int i = 0; i < 400; i++) {
    unsigned long alc = readADC();
    float currentBrightness = brightness - (i * 255.0 / 400.0);
    if (currentBrightness < 0) currentBrightness = 0;
    setupPWM();
    setPWM((uint8_t)currentBrightness);
    if (alcMax < alc) {
      alcMax = alc;
    }
    delayMs(10);
  }
  stopPWM();

  // Printare rezultate si verdict :)
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

    // LED rosu + buzz lung
    PORTB |= (1 << PORTB4);
    tone(3, 2000, 3000);
    delayMs(3000);
    PORTB &= ~(1 << PORTB4);

  } else {
    
    lcd.print("Esti curat!");
    Serial.println("Bravo, esti curat!");

    // LED verde
    PORTD |= (1 << PORTD4);
    delayMs(1000);
    PORTD &= ~(1 << PORTD4);
  }
  
}

void loop() {

  sensors.requestTemperatures();
  // Delay pt ADC temperatura
  delayMs(375);
  float temperature = sensors.getTempCByIndex(0);
  Serial.println(temperature);

  breathCooldown += 300;
  
  // Daca avem intrerupere cu buton sau detectare expiratie
  if (manualTrigger || (breathCooldown >= 3000 && temperature - lastTemp > 0.2)) {
    breathCooldown = manualTrigger ? breathCooldown : 0;
    manualTrigger = false;
    measureAlcohol();
    delayMs(2000);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Astept...");
  }

  lastTemp = temperature;
}
