#ifndef SENSORSTRUCT_H_
#define SENSORSTRUCT_H_
/**
 * Type definition for BME280 data struct
 */
typedef struct SensorStruct {
  double t; //Temperature
  double p; //Pressure
  double alt; //Altitude
  double hum; //Humidity
  double wSpeed; //Windspeed
} bme280Struct;

#endif
