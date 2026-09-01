/******************************************
Sketch to read from BME280 Sensor and send
per ESP-NOW to Receiver. Simple fire and
forget, can't receive anything.
*******************************************/
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
// #include <ESP8266WiFi.h>
// #include <espnow.h>
#include "ComStruct.h"
#include "SensorStruct.h"

//define static vars
#define SEALEVELPRESSURE_HPA (1013.25)
#define I2C_SDA 4   // ESP32-H2: GPIO4 (SDA)
#define I2C_SCL 5   // ESP32-H2: GPIO5 (SCL)
#define SAMPLING_COUNT 10
#define SAMPLING_DELAY 100
#define SLEEP_TIME 900e6
// #define LED_BUILTIN 1
#define ANEMOMETER ADC1_CH0  // GPIO36 on ESP32-H2 (ADC1_CH0)
// #define INTERRUPT_PIN 12

//Input voltage to analog pin
// ADC_MODE(ADC_VCC);
char measurands[][50] = { "TEMPERATURE", "PRESSURE", "ALTITUDE", "HUMIDITY", "VOLTAGE", "WINDSPEED", "RAIN" };

// "define" MAC's as uint8_t arrays
// uint8_t MAC_SELF[] = { 0x84, 0xF3, 0xEB, 0x05, 0x43, 0xE8 };
// uint8_t MAC_RECEIVER[] = {0x84, 0x0D, 0x8E, 0xB7, 0xFE, 0x1B};

TwoWire I2CBME = TwoWire(0);
Adafruit_BME280 bme;  // I2C --> I2C only available on GPIO 4+5
int dpID = 0;
volatile int rainSensorCounter = 0;

/**
 * Setup function
 */
void setup() {
  // put your setup code here, to run once:
  // pinMode(LED_BUILTIN, OUTPUT);
  // digitalWrite(LED_BUILTIN, LOW);

  I2CBME.begin(I2C_SDA, I2C_SCL, 100000);

  //Interrupt for Windspeed
  // attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), rainSensorInterrupt, FALLING);

  //Important: Start Serial com after activating outputs
  Serial.begin(115200);

  //Initialize WiFi
  // WiFi.mode(WIFI_STA);
  // Serial.print("MAC address:" );
  // Serial.println(WiFi.macAddress());

  //Initialize ESP_NOW
  // if (esp_now_init() != 0) {
  //   Serial.println("Error initializing ESP-NOW");
  //   return;
  // }
  // esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  // esp_now_register_send_cb(OnDataSent);
  // esp_now_add_peer(MAC_RECEIVER, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);

  //Initialize BME
  bool status;
  status = bme.begin(0x76, &I2CBME);
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1) delay(1000);  // halt instead of exit(1)
  }
}

/**
 * Loop function
 */
void loop() {
  //calc meanVals
  SensorStruct meanVals;
  sampleData(&meanVals);

  //Send all meanVals per ESPNow
  for (int i = 0; i < 4; i++) {
    comStruct send;
    send.datapointID = dpID++;
    sprintf(send.id, "%lu", bme.sensorID());
    strcpy(send.sensorType, "bme280");
    strcpy(send.key, measurands[i]);
    switch (i) {
      case 0:
        send.dValue = meanVals.t;
        break;
      case 1:
        send.dValue = meanVals.p;
        break;
      case 2:
        send.dValue = meanVals.alt;
        break;
      case 3:
        send.dValue = meanVals.hum;
        break;
    }

    //fill with empty string
    strcpy(send.sValue, "");
    //Send to Receiver
    //esp_now_send(MAC_RECEIVER, (uint8_t *) &send, sizeof(send));
  }

  //Send rain count
  comStruct rainAmount;
  rainAmount.datapointID = dpID++;
  strcpy(rainAmount.id, "ane");
  strcpy(rainAmount.sensorType, "raingauge");
  strcpy(rainAmount.key, measurands[6]);  // RAIN was measurands[5], now [6]
  strcpy(rainAmount.sValue, "");
  rainAmount.dValue = rainSensorCounter * 450;

  rainSensorCounter = 0;
  //esp_now_send(MAC_RECEIVER, (uint8_t *) &windSpeed, sizeof(windSpeed));

  //Send ESP voltage
  // comStruct voltage;
  // voltage.datapointID = dpID++;
  // strcpy(voltage.id, "vcc");
  // strcpy(voltage.sensorType, "ADC_VCC");
  // strcpy(voltage.key, measurands[4]);
  // strcpy(voltage.sValue, "");
  // voltage.dValue = ESP.getVcc();
  //esp_now_send(MAC_RECEIVER, (uint8_t *) &voltage, sizeof(voltage));


  //Log to console
  Serial.print("\n---------------------\nTemperature: ");
  Serial.print(meanVals.t);
  Serial.print("\nPressure: ");
  Serial.print(meanVals.p);
  Serial.print("\nAltitude: ");
  Serial.print(meanVals.alt);
  Serial.print("\nHumidity: ");
  Serial.print(meanVals.hum);
  Serial.print("\nWindSpeed (m/s): ");
  Serial.print(meanVals.wSpeed);
  Serial.print("\nWindSpeed (km/h): ");
  Serial.print((meanVals.wSpeed*360)/1000);
  Serial.print("\nRain: ");
  Serial.print(rainAmount.dValue);
  // Serial.print("\nVCC: ");
  // Serial.print(voltage.dValue);
  // Serial.print("\n---------------------\nSleep");

  //Deep Sleep
  // ESP.deepSleep(5000);
  delay(5000);
  Serial.print("\nWakeup\n---------------------\n");
}

/**
 *
 */
IRAM_ATTR void rainSensorInterrupt() {
  rainSensorCounter++;
}


/*
 * sample Data according to SAMPLING_COUNT and SAMPLING_DELAY
 * @param{meanVals} - SensorStruct*
 */
void sampleData(SensorStruct* meanVals) {
  //collect values from bme + anemometer
  SensorStruct sensorVals[SAMPLING_COUNT];
  for (int i = 0; i < SAMPLING_COUNT; i++) {
    readSensors(&bme, &sensorVals[i]);
    delay(SAMPLING_DELAY);
  }

  //calc mean's from collected values
  for (int i = 0; i < SAMPLING_COUNT; i++) {
    meanVals->t += sensorVals[i].t;
    meanVals->p += sensorVals[i].p;
    meanVals->alt += sensorVals[i].alt;
    meanVals->hum += sensorVals[i].hum;
    meanVals->wSpeed += sensorVals[i].wSpeed;
  }
  meanVals->t = meanVals->t / SAMPLING_COUNT;
  meanVals->p = meanVals->p / SAMPLING_COUNT;
  meanVals->alt = meanVals->alt / SAMPLING_COUNT;
  meanVals->hum = meanVals->hum / SAMPLING_COUNT;
  meanVals->wSpeed = meanVals->wSpeed / SAMPLING_COUNT;
}

/*
 * Read data from bme280 sensor and write to struct
 * @param{sens} - Adafruit_BME280*
 * @param{vals} - SensorStruct*
 */
void readSensors(Adafruit_BME280* sens, SensorStruct* vals) {
  vals->t = sens->readTemperature();
  vals->p = sens->readPressure() / 100.0F;
  vals->alt = sens->readAltitude(SEALEVELPRESSURE_HPA);
  vals->hum = sens->readHumidity();


  double wVoltage = analogRead(ANEMOMETER);
  // ESP32-H2: 12-bit ADC (0-4095), 3.3V reference
  // Assuming voltage divider: sensor outputs 0-2V into 3.3V max
  vals->wSpeed = (wVoltage / 4095.0) * 3.3 * 25;  // scaled to 3.3V ref
  Serial.print("\nwVoltage: ");
  Serial.println(wVoltage);
  //Serial.print("\nWind measured: ");
  // Serial.print(vals->wSpeed);
}

// Callback when data is sent
void OnDataSent(uint8_t* mac_addr, uint8_t sendStatus) {
  if (sendStatus != 0) {
    Serial.println("\nLast Packet Send Status: Delivery fail");
  }
}