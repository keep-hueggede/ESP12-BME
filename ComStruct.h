/**
* Type definition for esp-now communication (use in sender and receiver)
*/
typedef struct comStruct {
  char id[50]; //sensor id
  char key[20]; //val key (e.g. temperature, pressure)
  char sensorType[50];
  double dValue;
  char[50] sValue;
  bool type = 0; //0 => double; 1 => string
}