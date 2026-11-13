#include <Arduino.h>
#include "blob.h"
#include "thermal.h"
#include <Wire.h>
#include "MLX_files/MLX90640_API.h"
#include "MLX_files/MLX90640_I2C_Driver.h"

void setup() {
  // put your setup code here, to run once:
  thermalsetup();
}

void loop() {
  // put your main code here, to run repeatedly:
  thermalloop();
}

