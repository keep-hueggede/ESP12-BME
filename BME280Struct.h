#ifndef BME280STRUCT_H_
#define BME280STRUCT_H_
/**
* Type definition for BME280 data struct
*/
typedef struct bme280Struct {
  double t; //Temperature
  double p; //Pressure
  double alt; //Altitude
  double hum; //Humidity
} bme280Struct;

#endif