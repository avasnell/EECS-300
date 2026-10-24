#ifndef BLOB_H
#define BLOB_H
#include <Arduino.h>
#include <vector>


using namespace std;


typedef struct{
    int row;
    int col;
} Pixel;

typedef struct {
  uint8_t id; //tracker id
  float centroid_x; //center x position
  float centroid_y; //center y position
  float max_temp; //max temp of blob
  uint16_t area; //area of blob
  uint8_t age; //how many frames the blob has been there
  vector<Pixel> pixels; //pixels that make upS the blob
} Blob;

typedef struct{
    int number_of_blobs;
    vector<Blob> blobs;
    int hot_pixels_count;
    int (*matrix_of_temps)[24][32];
} Frame;

vector<Blob> get_Blobs(const Frame& frame);
Blob find_Blob(int startR, int startC, const Frame& frame);
uint16_t area(int tempValues[24][32]);
uint8_t max_temp(int tempValues[24][32]);
bool same_person(Blob blob1, Blob blob2);
float x_centroid(int tempValues[24][32]);
float y_centroid(int tempValues[24][32]);
#endif