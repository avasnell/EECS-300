#include "blob.h"
#include <vector>
#include <cstdint>
#include <queue>

using namespace std;

#define PIXEL_COUNT_THRESHOLD 40

vector<Blob> get_Blobs(const Frame& frame){
    vector<Blob> blobs;
    for (int r = 0; r < 24; r++){
        for (int c = 0; c < 32; c++){
            if ((*frame.matrix_of_temps)[r][c] > 0){
                Blob blob = find_Blob(r, c, frame);
                if (blob.area > PIXEL_COUNT_THRESHOLD){
                    blobs.push_back(blob);
                }
            }
        }
    }
    return blobs;
}


//Breadth-first search to find a blob starting from a given pixel
Blob find_Blob(int startR, int startC, const Frame& frame){
     Blob blob = {0, 0, 0, 0, 0, 0};

     //if empty, return empty blob
    if (frame.hot_pixels_count == 0){
        return blob;
    }
    
    //queue for BFS
    std::queue<Pixel> q;    
    Pixel start_pixel = {startR, startC}; 
    q.push(start_pixel);

    bool visited[24][32] = {false};
    visited[startR][startC] = true;

    while (!q.empty()){
        //make current pixel the front of queue and pop it
        Pixel current_pixel = q.front();
        q.pop();

        //add current pixel to blob and add to visited
        blob.pixels.push_back(current_pixel);
        visited[current_pixel.row][current_pixel.col] = true;

        //check diagonals and adjacent pixels
        const int dr[] = {-1, -1, -1,  0, 0,  1, 1, 1};
        const int dc[] = {-1,  0,  1, -1, 1, -1, 0, 1};
      
        for (int i = 0; i < 8; ++i){
            int newR = current_pixel.row + dr[i];
            int newC = current_pixel.col + dc[i];
            if (newR >= 0 && newR < 24 && newC >= 0 && newC < 32) { 
                if (!visited[newR][newC] && (*frame.matrix_of_temps)[newR][newC] > 0) {
                    visited[newR][newC] = true; 
                    q.push({newR, newC});
                }
            }
        }
    }
    int area = blob.pixels.size();
    float x_centroid = x_centroid(*frame.matrix_of_temps);
    float y_centroid = y_centroid(*frame.matrix_of_temps);
    float max_temp = max_temp(*frame.matrix_of_temps);
    blob.area = area;
    blob.centroid_x = x_centroid;
    blob.centroid_y = y_centroid;
    blob.max_temp = max_temp;
    return blob;
}

//find area of blob
uint16_t area(int tempValues[24][32]){
  uint16_t area = 0;
  for (int row = 0; row < 24; row++){
    for (int col = 0; col < 32; col++){
      if (tempValues[row][col] > 0){
        area++;
      }
    }
  }
  return area;
}

//find max temp of blob
uint8_t max_temp(int tempValues[24][32]){
  uint8_t max_temp = 0;
  for (int row = 0; row < 24; row++){
    for (int col = 0; col < 32 ; col++){
      if (tempValues[row][col] > max_temp){
        max_temp = tempValues[row][col];
      }
    }
  }
  return max_temp;
} 

//compare two blobs to see if they are the same person
//if the distance between the two blobs is less than 2, they are considered the same person
bool same_person(Blob blob1, Blob blob2){
  float distance = sqrt(pow(blob1.centroid_x - blob2.centroid_x, 2) + pow(blob1.centroid_y - blob2.centroid_y, 2));
  if (distance < 2.0){
    return true;
  }
  return false;
}

//take x_centroid of blob
// going through everything is inefficent, plan to fix
float x_centroid (int tempValues[24][32]){
  float sum_weight_x = 0;
  float sum_mass = 0;

  for (int row = 0; row < 24; row++){
    for (int col = 0; col < 32; col++){
      int val = tempValues[row][col];
      sum_weight_x += val *row;
      sum_mass += val;
    }
  }
  if (sum_mass == 0) return 0;
  return sum_weight_x / sum_mass;
}

//y_centroid
float y_centroid (int tempValues[24][32]){
  float sum_weight_y = 0;
  float sum_mass = 0;
  for (int row = 0; row < 24; row++){
    for (int col = 0; col < 32; col++){
      int val = tempValues[row][col];
      sum_weight_y += val *col;
      sum_mass += val;
    }
  }
  if (sum_mass == 0) return 0;
  return sum_weight_y / sum_mass;
}