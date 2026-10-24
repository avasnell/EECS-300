#include "Arduino.h"
#include <Wire.h>
#include "MLX_files/MLX90640_API.h"
#include "MLX_files/MLX90640_I2C_Driver.h"

void thermalsetup();
void thermalloop(void); 
void readTempValues();
void readTemp_and_PrintOutput();
void readTemp_and_PrintPixCt();
void Device_Scan();
void convertTempValuesToGrid(int tempValues[24][32], float tempValuesFloat[32*24]);
bool person_detected();
void readBlobs_from_Frame();