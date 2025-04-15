#include <Wire.h>
#include "SPI.h"
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP280.h"
#include <DHT.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

Adafruit_BMP280 bmp;

#define DHTPIN A3
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

#define SOIL_MOISTURE_PIN A2

const int AirValue = 620;
const int WaterValue = 310;

int moisture_value;
int moisture_percent;
bool bmp_ok = false;
unsigned long startTime;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  delay(100);

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.print("Starting...");

  dht.begin();

  if (bmp.begin() || bmp.begin(0x76) || bmp.begin(0x77)) {
    bmp_ok = true;
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                     Adafruit_BMP280::SAMPLING_X2,
                     Adafruit_BMP280::SAMPLING_X16,
                     Adafruit_BMP280::FILTER_X16,
                     Adafruit_BMP280::STANDBY_MS_500);

    float test_pressure = bmp.readPressure();
    if (!(test_pressure >= 10 && test_pressure <= 120000)) {
      bmp_ok = false;
      Serial.println("BMP280 Invalid Reading");
    } else {
      Serial.println("BMP280 OK");
    }
  } else {
    Serial.println("BMP280 Not Found");
  }

  startTime = millis();
}

void renderLCD(float temperature, float humidity, int moisture_percent, float pressure) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("T:");
  lcd.print(temperature, 1);
  lcd.print("C P:");
  if (bmp_ok) {
    lcd.print((int)pressure);
    lcd.print("hPa");
  } else {
    lcd.print("N/A");
  }

  lcd.setCursor(0, 1);
  lcd.print("H:");
  lcd.print(humidity, 0);
  lcd.print("% M:");
  lcd.print(moisture_percent);
  lcd.print("%");
}

void loop() {
  float temp = dht.readTemperature();
  float hum = dht.readHumidity();

  moisture_value = analogRead(SOIL_MOISTURE_PIN);
  moisture_percent = map(moisture_value, AirValue, WaterValue, 0, 100);
  moisture_percent = constrain(moisture_percent, 0, 100);

  if (millis() - startTime < 3000) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Initializing...");
    delay(500);
    return;
  }

  float pressure = 0;
  if (bmp_ok) {
    pressure = bmp.readPressure() / 100.0;
    if (pressure < 800 || pressure > 1200) {
      pressure = 0;
    }
  }

  if (!isnan(temp) && !isnan(hum)) {
    Serial.print("Temp: ");
    Serial.print(temp);
    Serial.print("C  Hum: ");
    Serial.print(hum);
    Serial.print("%  Moist: ");
    Serial.print(moisture_percent);
    Serial.print("%");
    if (bmp_ok && pressure > 0) {
      Serial.print("  Press: ");
      Serial.print(pressure);
      Serial.println("hPa");
    } else {
      Serial.println("  Press: N/A");
    }
    renderLCD(temp, hum, moisture_percent, pressure);
  } else {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("DHT Error");
    Serial.println("DHT11 Read Error");
  }

  delay(2000);
}