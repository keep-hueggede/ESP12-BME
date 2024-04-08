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
#include "ComStruct.h"
#include "BME280Struct.h"

//define static vars
#define SEALEVELPRESSURE_HPA (1013.25)
#define BME_VCC 15
#define SAMPLING_COUNT 10
#define SAMPLING_DELAY 1000
#define SLEEP_TIME 900e6
#define LED_BUILTIN 1
#define INTERRUPT_PIN 12

//Input voltage to analog pin
ADC_MODE(ADC_VCC);
char measurands[][50] = { "TEMPERATURE", "PRESSURE", "ALTITUDE", "HUMIDTY", "VOLTAGE", "WINDSPEED" };

// "define" MAC's as uint8_t arrays
uint8_t MAC_SELF[] = { 0x84, 0xF3, 0xEB, 0x05, 0x43, 0xE8 };
uint8_t MAC_RECEIVER[] = {0x84, 0x0D, 0x8E, 0xB7, 0xFE, 0x1B};

Adafruit_BME280 bme;  // I2C --> I2C only available on GPIO 4+5
int dpID = 0;
int rainSensorCounter = 0;

/**
* Setup function
*/
void setup() {
  // put your setup code here, to run once:  
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BME_VCC, OUTPUT);
  digitalWrite(BME_VCC, HIGH);
  digitalWrite(LED_BUILTIN, LOW);  

  //Interrupt for Windspeed
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), rainSensorInterrupt, FALLING);  

  //Important: Start Serial com after activating outputs
  Serial.begin(115200);

  //Initialize WiFi
  WiFi.mode(WIFI_STA);
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
  //calc meanVals
  bme280Struct meanVals;
  sampleData(&meanVals);     

  //Send all meanVals per ESPNow
  for(int i = 0; i< 4; i++){
    comStruct send;
    send.datapointID = dpID++;
    sprintf(send.id, "%lu", bme.sensorID());        
    strcpy(send.sensorType, "bme280");
    strcpy(send.key, measurands[i]);
    switch(i){
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
  strcpy(rainAmount.sensorType, "anemometer");
  strcpy(rainAmount.key, measurands[5]);  
  strcpy(rainAmount.sValue, "");
  rainAmount.dValue = rainSensorCounter * 450;

  rainSensorCounter = 0;
  //esp_now_send(MAC_RECEIVER, (uint8_t *) &windSpeed, sizeof(windSpeed));   

  //Send ESP voltage
  comStruct voltage;
  voltage.datapointID = dpID++;
  strcpy(voltage.id, "vcc");
  strcpy(voltage.sensorType, "ADC_VCC");
  strcpy(voltage.key, measurands[4]);  
  strcpy(voltage.sValue, "");
  voltage.dValue = ESP.getVcc();
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
  Serial.print("\nRain: ");
  Serial.print(rainAmount.dValue);
  Serial.print("\nVCC: ");
  Serial.print(voltage.dValue);
  Serial.print("\n---------------------\nSleep");

  //Deep Sleep
  ESP.deepSleep(5000);
  //delay(15000);
  Serial.print("\nWakeup\n---------------------\n");
}

/**
 *
 */
ICACHE_RAM_ATTR void rainSensorInterrupt(){  
  rainSensorCounter++;
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

// Callback when data is sent
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  if (sendStatus != 0){    
    Serial.println("\nLast Packet Send Status: Delivery fail");
  }
}