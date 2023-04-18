/******************************************
Skecth to read from BME280 Sensor and send
per ESP-NOW to Receiver. Simple fire and 
forget, can't receive anything.
*******************************************/
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ESP8266WiFi.h>
#include <espnow.h>
//#include <ComStruct.h>

//define static vars
#define SEALEVELPRESSURE_HPA (1013.25)
#define BME_VCC 15
#define SAMPLING_COUNT 10
#define SAMPLING_DELAY 1000
#define SLEEP_TIME 10000//900e6
#define measurands { TEMPERATURE, PRESSURE, ALTITUDE, HUMIDTY }
#define MAC_SELF { 0x84, 0xF3, 0xEB, 0x05, 0x43, 0xE8 }
#define MAC_RECEIVER {0x84, 0x0D, 0x8E, 0xB7, 0xFE, 0x1B}

Adafruit_BME280 bme;  // I2C --> I2C only available on GPIO 4+5

struct bme280Struct {
  double t;
  double p;
  double alt;
  double hum;
};

/**
* Setup function
*/
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BME_VCC, OUTPUT);
  digitalWrite(BME_VCC, HIGH);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.print("MAC address:" );
  Serial.println(WiFi.macAddress());

  bool status;

  // default settings
  status = bme.begin(0x76);
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    exit(1);
  }
}

/**
* Loop function
*/
void loop() {
  bme280Struct meanVals;  //calc meanVals
  sampleData(&meanVals);

  Serial.print("\n---------------------\nTemperature: ");
  Serial.print(meanVals.t);
  Serial.print("\nPressure: ");
  Serial.print(meanVals.p);
  Serial.print("\nAltitude: ");
  Serial.print(meanVals.alt);
  Serial.print("\nHumidity: ");
  Serial.print(meanVals.hum);
  Serial.print("\n---------------------\nSleep");
  ESP.deepSleep(SLEEP_TIME);
  Serial.print("Wakeup\n---------------------\n");
}

/*
 * sample Data according to SAMPLING_COUNT and SAMPLING_DELAY
 * @param{meanVals} - bme280Struct*
 */
void sampleData(bme280Struct* meanVals) {
  //collect values from bme
  bme280Struct sensorVals[SAMPLING_COUNT];
  for (int i = 0; i < SAMPLING_COUNT; i++) {
    readBME280(&bme, &sensorVals[i]);
    delay(SAMPLING_DELAY);
  }

  //calc mean's from collected values
  for (int i = 0; i < SAMPLING_COUNT; i++) {
    meanVals->t += sensorVals[i].t;
    meanVals->p += sensorVals[i].p;
    meanVals->alt += sensorVals[i].alt;
    meanVals->hum += sensorVals[i].hum;
  }
  meanVals->t = meanVals->t / SAMPLING_COUNT;
  meanVals->p = meanVals->p / SAMPLING_COUNT;
  meanVals->alt = meanVals->alt / SAMPLING_COUNT;
  meanVals->hum = meanVals->hum / SAMPLING_COUNT;
}

/*
* Read data from bme280 sensor and write to struct
* @param{sens} - Adafruit_BME280*
* @param{vals} - bme280Struct*
*/
void readBME280(Adafruit_BME280* sens, bme280Struct* vals) {
  vals->t = sens->readTemperature();
  vals->p = sens->readPressure() / 100.0F;
  vals->alt = sens->readAltitude(SEALEVELPRESSURE_HPA);
  vals->hum = sens->readHumidity();
}