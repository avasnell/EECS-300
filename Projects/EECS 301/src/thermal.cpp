#include "Arduino.h"
#include <Wire.h>
#include "MLX_files/MLX90640_API.h"
#include "MLX_files/MLX90640_I2C_Driver.h"
#include "thermal.h"
#include "blob.h"
#include <vector>
#include <iostream>
#include <fstream>
#include <queue>
#include <string>


#define EMMISIVITY 0.95
#define TA_SHIFT 8 
const int LED_BUILTIN = 2;

#define DOT_VALUE 28

//how many pixels need to be above threshold to count as a person
#define PIXEL_COUNT_THRESHOLD 20


bool already_found = false;
bool detected = false;
bool prev_detected = false;
int counter = 0;
int pixel_count=0;
bool person_thermal=false;
bool person_IR=false;

Blob blob_previous = {0, 0, 0, 0, 0, 0};
Blob blob_current = {0, 0, 0, 0, 0, 0};

paramsMLX90640 mlx90640;
const byte MLX90640_address = 0x33; //Default 7-bit unshifted address of the MLX90640
static float tempValues[32 * 24];

vector<Blob> blobs;
int (*matrix_just_blob)[24][32];

Frame current_frame = {0,blobs, 0, matrix_just_blob};
Prev_Frame previous_frame = {0,blobs, 0, matrix_just_blob};

void thermalsetup() {
    
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  Wire.begin();
  Wire.setClock(400000); 
  Wire.beginTransmission((uint8_t)MLX90640_address);
  if (Wire.endTransmission() != 0) {
    Serial.println("MLX90640 not detected at default I2C address. Starting scan the device addr...");
    Device_Scan();
  }
  else {
    Serial.println("MLX90640 online!");
  }
  int status;
  uint16_t eeMLX90640[832];
  status = MLX90640_DumpEE(MLX90640_address, eeMLX90640);
  if (status != 0) Serial.println("Failed to load system parameters");
  status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
  if (status != 0) Serial.println("Parameter extraction failed");
  MLX90640_SetRefreshRate(MLX90640_address, 0x01); 
  Wire.setClock(800000);
}

void thermalloop(void) {
  //readTempValues();
  //readTemp_and_PrintOutput();
  readBlobs_from_Frame();
  delay(20);
}

void readTempValues() {
  for (byte x = 0 ; x < 2 ; x++) 
  {
    uint16_t mlx90640Frame[834];
    int status = MLX90640_GetFrameData(MLX90640_address, mlx90640Frame);
    if (status < 0)
    {
      Serial.print("GetFrame Error: ");
      Serial.println(status);
    }

    float vdd = MLX90640_GetVdd(mlx90640Frame, &mlx90640);
    float Ta = MLX90640_GetTa(mlx90640Frame, &mlx90640);

    float tr = Ta - TA_SHIFT; 

    MLX90640_CalculateTo(mlx90640Frame, &mlx90640, EMMISIVITY, tr, tempValues);
  }
  Serial.println("\r\n===========================WaveShare MLX90640 Thermal Camera===============================");
  for (int i = 0; i < 768; i++) {
    if (((i % 32) == 0) && (i != 0)) {
      Serial.println(" ");
    }
    Serial.print((int)tempValues[i]);
    Serial.print(" ");

    if (tempValues[i]>30) {
      detected = true;
      if (already_found==false) { already_found=true; }
      digitalWrite(LED_BUILTIN, HIGH);
    }
    
  }
  /////////////////////////////////////////// IMPLEMENT COUNTER
  already_found=false;
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("\r\n===========================WaveShare MLX90640 Thermal Camera===============================");
}

void readBlobs_from_Frame() {
//make ID queue
queue<string> idQueue;
ifstream file("ids.txt");
string line;

while (getline(file, line)) {
    idQueue.push(line); 
}
  int pixel_count=0;
  for (byte x = 0 ; x < 2 ; x++) 
  {
    static uint16_t mlx90640Frame[834];
    int status = MLX90640_GetFrameData(MLX90640_address, mlx90640Frame);
    if (status < 0)
    {
      Serial.print("GetFrame Error: ");
      Serial.println(status);
    }

    float vdd = MLX90640_GetVdd(mlx90640Frame, &mlx90640);
    float Ta = MLX90640_GetTa(mlx90640Frame, &mlx90640);

    float tr = Ta - TA_SHIFT; 

    MLX90640_CalculateTo(mlx90640Frame, &mlx90640, EMMISIVITY, tr, tempValues);
  }
  Serial.println("\r\n===========================WaveShare MLX90640 Thermal Camera===============================");
  //create blob from temp_values and print out blob info
  static int temp_values[24][32];
  convertTempValuesToGrid(temp_values, tempValues);


  current_frame.matrix_of_temps = &temp_values;
  current_frame.hot_pixels_count = area(temp_values);
  current_frame.blobs = get_Blobs(current_frame);
  current_frame.number_of_blobs = current_frame.blobs.size();
  current_frame.previous_frame = previous_frame;

  vector<Blob> which_blobs;
  which_blobs = which_blob(current_frame.blobs, current_frame, previous_frame);

  print_vector_of_Blobs(current_frame.blobs);

  Serial.println();
  Serial.println();
  Serial.print("Number of blobs: ");
    Serial.println(current_frame.number_of_blobs);
  Serial.print("Hot pixels count: ");
    Serial.println(current_frame.hot_pixels_count);


  for (int i = 0; i < 768; i++) {
    
    if (((i % 32) == 0) && (i != 0)) {
      Serial.println(" ");
    }
    
    if(((int)tempValues[i]) > DOT_VALUE){
      Serial.print(("*"));
    } else {
      Serial.print(" ");
    }
    Serial.print(" ");
  }
  blob_previous = blob_current;
  previous_frame.prev_matrix_of_temps = current_frame.matrix_of_temps;
  previous_frame.prev_hot_pixels_count = current_frame.hot_pixels_count;
  previous_frame.prev_blobs = current_frame.blobs;
  previous_frame.prev_number_of_blobs = current_frame.number_of_blobs;
}

void readTemp_and_PrintOutput() {
  int pixel_count=0;
  for (byte x = 0 ; x < 2 ; x++) 
  {
    uint16_t mlx90640Frame[834];
    int status = MLX90640_GetFrameData(MLX90640_address, mlx90640Frame);
    if (status < 0)
    {
      Serial.print("GetFrame Error: ");
      Serial.println(status);
    }

    float vdd = MLX90640_GetVdd(mlx90640Frame, &mlx90640);
    float Ta = MLX90640_GetTa(mlx90640Frame, &mlx90640);

    float tr = Ta - TA_SHIFT; 

    MLX90640_CalculateTo(mlx90640Frame, &mlx90640, EMMISIVITY, tr, tempValues);
  }
  Serial.println("\r\n===========================WaveShare MLX90640 Thermal Camera===============================");
  //create blob from temp_values and print out blob info
  static int temp_values[24][32];
  convertTempValuesToGrid(temp_values, tempValues);
  float x_cent= x_centroid(temp_values);
  float y_cent = y_centroid(temp_values);
  Serial.print("X Centroid: ");
  Serial.println(x_cent);
  Serial.print("Y Centroid: ");
  Serial.println(y_cent);

  uint16_t area_val = area(temp_values);
  Serial.print("Area: ");
  Serial.println(area_val);

  float max_temp_val = max_temp(temp_values);
  Serial.print("Max Temp: ");
  Serial.println(max_temp_val);

  blob_current.centroid_x = x_cent;
  blob_current.centroid_y = y_cent;
  blob_current.area = area_val;
  blob_current.max_temp = max_temp_val;

  bool same_person_flag = same_person(blob_current, blob_previous);
  if ((blob_current.area > PIXEL_COUNT_THRESHOLD) && !same_person_flag) {
    Serial.println("New person detected");
  } else if ((blob_current.area > PIXEL_COUNT_THRESHOLD) && same_person_flag) {
    Serial.println("Same person detected");
  } else {
    Serial.println("No person detected");
  }
  
  for (int i = 0; i < 768; i++) {
    
    if (((i % 32) == 0) && (i != 0)) {
      Serial.println(" ");
    }
    
    if(((int)tempValues[i]) > DOT_VALUE){
      Serial.print(("*"));
    } else {
      Serial.print(" ");
    }
    Serial.print(" ");
  }
  blob_previous = blob_current;
}

void readTemp_and_PrintPixCt() {
  int pixel_count=0;
  for (byte x = 0 ; x < 2 ; x++) 
  {
    uint16_t mlx90640Frame[834];
    int status = MLX90640_GetFrameData(MLX90640_address, mlx90640Frame);
    if (status < 0)
    {
      Serial.print("GetFrame Error: ");
      Serial.println(status);
    }

    float vdd = MLX90640_GetVdd(mlx90640Frame, &mlx90640);
    float Ta = MLX90640_GetTa(mlx90640Frame, &mlx90640);

    float tr = Ta - TA_SHIFT; 

    MLX90640_CalculateTo(mlx90640Frame, &mlx90640, EMMISIVITY, tr, tempValues);
  }
  //Serial.println("\r\n===========================WaveShare MLX90640 Thermal Camera===============================");
  for (int i = 0; i < 768; i++) {
    
    /*if (((i % 32) == 0) && (i != 0)) {
      Serial.println(" ");
    }
    */
    if(((int)tempValues[i]) > DOT_VALUE){
      //Serial.print(("*"));
      pixel_count++;
    } else {
      //Serial.print("-");
    }
    //Serial.print(" ");
  }
  
  //Serial.println("pixel count: ");
  Serial.println(pixel_count);
  //Serial.println("\r\n===========================WaveShare MLX90640 Thermal Camera===============================");
}

void Device_Scan() {
  byte error, address;
  int nDevices;
  Serial.println("Scanning...");
  nDevices = 0;
  for (address = 1; address < 127; address++ )
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");
      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknow error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found");
  else
    Serial.println("done");

  if(pixel_count> PIXEL_COUNT_THRESHOLD) {
  person_thermal=true;
}
}

//blob functions take [32][24] array of temp values and tempValues from MLX90640 is an array of 768 values
//so need to convert tempValues to [32][24] array
void convertTempValuesToGrid(int tempValues[24][32], float tempValuesFloat[32*24]) {
  for (int row = 0; row < 24; row++) {
    for (int col = 0; col < 32; col++) {
        if (((int)tempValuesFloat[row * 32 + col]) > DOT_VALUE) {
            tempValues[row][col] = (int)tempValuesFloat[row * 32 + col];
        } else {
            tempValues[row][col] = 0;
        }
    }
  }
}

//convert [32][24] array of temp values to tempValues array of 768 values (used in Blob.cpp)
void convertTempValuestoArray(float tempValuesFloat[32*24], int tempValues[24][32]) {
  for (int row = 0; row < 24; row++) {
    for (int col = 0; col < 32; col++) {
        tempValuesFloat[row * 32 + col] = (float)tempValues[col][row];
    }
  }
}

bool person_detected() {
  return person_thermal;
}


