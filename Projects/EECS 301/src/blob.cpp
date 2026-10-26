#include "blob.h"
#include <vector>
#include <cstdint>
#include <queue>


using namespace std;

#define PIXEL_COUNT_THRESHOLD 20

//determine if blob is headiing in enter direction or exit direction
//1 is enter and 0 is exit
bool track_blob_direction (int previous_x_cent, int current_x_cent) {
  //determine if blob is moving in enter direction or exit direction
  if (current_x_cent > previous_x_cent) {
    return true; //enter direction
  } else {
    return false; //exit direction
  }
  
}

vector<Blob> get_Blobs(const Frame& frame){
  bool visited[24][32] = {false}; //initialize visited array to false
    vector<Blob> blobs;
    for (int r = 0; r < 24; r++){
        for (int c = 0; c < 32; c++){
            if ((*frame.matrix_of_temps)[r][c] > 0){
                Blob blob = find_Blob(visited, r, c, frame); 
                if (blob.area > PIXEL_COUNT_THRESHOLD){
                    blobs.push_back(blob);
                }
            }
        }
    }
    return blobs;
}


//Breadth-first search to find a blob starting from a given pixel
Blob find_Blob(bool (&visited)[24][32], int startR, int startC, const Frame& frame){
     Blob blob = {0, 0, 0, 0, 0, 0};

     //if empty, return empty blob
    if (frame.hot_pixels_count == 0){
        return blob;
    }
    
    //queue for BFS
    std::queue<Pixel> q;    
    Pixel start_pixel = {startR, startC}; 
    q.push(start_pixel);

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
          //Serial.println("hey guys");
            int newR = current_pixel.row + dr[i];
            int newC = current_pixel.col + dc[i];
            if (newR >= 0 && newR < 24 && newC >= 0 && newC < 32) { 
                if (!visited[newR][newC] && (*frame.matrix_of_temps)[newR][newC] > 0) {
                    visited[newR][newC] = true; 
                    q.push({newR, newC});
                }
            }
        }
       // Serial.println("hey guyssgfdoxjhkfdjghod");
    }
    int area = blob.pixels.size();
    float x_centroidd = x_centroid(*frame.matrix_of_temps);
    float y_centroidd = y_centroid(*frame.matrix_of_temps);
    float max_tempp = max_temp(*frame.matrix_of_temps);
    blob.area = area;
    blob.centroid_x = x_centroidd;
    blob.centroid_y = y_centroidd;
    blob.max_temp = max_tempp;
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

// take in a blob and identify based on closest centroid of previous frame
Blob which_blob (Blob blob) {
  Frame cur_frame;
  Prev_Frame prev_frame;
  float current_x = blob.centroid_x;
  float current_y = blob.centroid_y;

  // if there's only one blob in both current and previous, assume they are the same
  if (cur_frame.number_of_blobs == 1 && prev_frame.prev_number_of_blobs == 1) {
    blob.age++;
    return cur_frame.blobs[0]; // verify that the only blob is in index 0
    // verify that this vector has the updated blob age
  }

  ///////////////////////////////////////////////////// this is highkey incorrect. keep in mind we only take in ONE blob. or adjust to take in a vector of blobs.
  // case where there are more blobs in prev than cur
  else if (prev_frame.prev_number_of_blobs>cur_frame.number_of_blobs) {
    vector<Blob> compare_queue = cur_frame.blobs;
    for (int ind=0; ind<cur_frame.number_of_blobs; ind++) {
      vector<float> distances;

      // compare to each old blob
      for(int prev_ind=0; ind<prev_frame.prev_number_of_blobs; ind++) {
        distances[ind] = sqrt(pow(compare_queue[ind].centroid_x - prev_frame.prev_blobs[prev_ind].centroid_x, 2) + pow(compare_queue[ind].centroid_y - prev_frame.prev_blobs[prev_ind].centroid_y, 2));
      }

      closest = min_element(distances.begin(), distances.end()); //////// return a pointer to min here
      ///// do the assignment here
    }
  }

  // case where they are equally numbered
  else if(prev_frame.prev_number_of_blobs==cur_frame.number_of_blobs){
    
  }

  // case where there are more blobs in cur than prev
}