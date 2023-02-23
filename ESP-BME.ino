#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define SEALEVELPRESSURE_HPA (1013.25)
#define BME_VCC 15
#define SAMPLING_COUNT 10
#define SAMPLING_DELAY 1000
#define SLEEP_TIME 900e6

Adafruit_BME280 bme;  // I2C --> I2C only available on GPIO 4+5

struct bme280Struct {
  double t;
  double p;
  double alt;
  double hum;
};

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BME_VCC, OUTPUT);
  digitalWrite(BME_VCC, HIGH);
  digitalWrite(LED_BUILTIN, LOW);
  Serial.begin(115200);

  bool status;

  // default settings
  status = bme.begin(0x76);
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    exit(1);
  }
}

void loop() {
  struct bme280Struct meanVals;  //calc meanVals
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

void sampleData(bme280Struct* meanVals) {
  //init bmeStruct
  struct bme280Struct sensorVals[10];

  //read 10x from bme
  for (int i = 0; i < SAMPLING_COUNT; i++) {
    readBME280(&bme, &sensorVals[i]);
    delay(SAMPLING_DELAY);
  }

  //calc
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

void readBME280(Adafruit_BME280* sens, bme280Struct* vals) {
  vals->t = sens->readTemperature();
  vals->p = sens->readPressure() / 100.0F;
  vals->alt = sens->readAltitude(SEALEVELPRESSURE_HPA);
  vals->hum = sens->readHumidity();
}