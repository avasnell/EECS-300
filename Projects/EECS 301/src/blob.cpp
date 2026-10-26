#include "blob.h"
#include <vector>
#include <cstdint>
#include <queue>
#include <map>


using namespace std;

#define PIXEL_COUNT_THRESHOLD 20
#define PIXEL_HEAT_THRESHOLD 30



vector<Blob> get_Blobs(const Frame& frame){
  //id number gets defined when creating the vector of blobs
  int id_number = 1;
  bool visited[24][32] = {false}; //initialize visited array to false
    vector<Blob> blobs;
    for (int r = 0; r < 24; r++){
        for (int c = 0; c < 32; c++){
            if ((*frame.matrix_of_temps)[r][c] > 0){
                Blob blob = find_Blob(visited, r, c, frame); 
                if (blob.area > PIXEL_COUNT_THRESHOLD){
                    blob.id = id_number;
                    id_number++;
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
    
    blob.matrix_just_blob = map_blob_pixels_to_matrix(blob.pixels);
    int area = blob.pixels.size();
    float x_centroidd = x_centroid(*blob.matrix_just_blob);
    float y_centroidd = y_centroid(*blob.matrix_just_blob);
    float max_tempp = max_temp(*blob.matrix_just_blob);
    blob.area = area;
    blob.centroid_x = x_centroidd;
    blob.centroid_y = y_centroidd;
    blob.max_temp = max_tempp;
    return blob;
}

//
auto map_blob_pixels_to_matrix(std::vector<Pixel> pixels) -> int (*)[24][32] {
  int (*mapped_matrix)[24][32] = {0};
  for (int r = 0; r < 24; r++){
    for (int c = 0; c < 32; c++){
      for (int i = 0; i < pixels.size(); i++){
        if (pixels[i].row ==  r && pixels[i].col == c){
          (*mapped_matrix)[r][c] = PIXEL_HEAT_THRESHOLD;
        } else {
          (*mapped_matrix)[r][c] = 0;
        }
      }
    }
  }
  return mapped_matrix;
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

vector<Blob> which_blob(vector<Blob> blobs, const Frame& cur_frame, const Prev_Frame& prev_frame){
   
    //map to store ID value of blobs in previous frame and their corresponding distances to blobs in current frame
    std::map<uint8_t, std::vector<float>> distances;

    //if there is only one blob in both current and previous frame, assume they are the same blob
    if (cur_frame.number_of_blobs == 1 && prev_frame.prev_number_of_blobs == 1) {
      Blob blob = cur_frame.blobs[0];
      blob.age++;
      return blobs;
    } else {
      
      //store centroids of current and previous blobs in vectors
      vector<std::pair<float,float>> current_centroids;
      for (int i = 0; i < blobs.size(); i++){
            Blob current_blob = blobs[i];
            std::pair<float, float> centroids;
            centroids.first = current_blob.centroid_x;
            centroids.second = current_blob.centroid_y;
            current_centroids.push_back(centroids);
          }
      
      vector<std::pair<float,float>> past_centroids;
      for(int i = 0; i < prev_frame.prev_blobs.size(); i++){
        Blob current_blob = prev_frame.prev_blobs[i];
        std::pair<float, float> centroids;
        centroids.first = current_blob.centroid_x;
        centroids.second =current_blob.centroid_y;
        past_centroids.push_back(centroids);
      }

      //calculate distances between current and previous centroids and store in map
      for(int i = 0; i < current_centroids.size(); i++){
        for(int j = 0; j < past_centroids.size(); j++){
          float distance = sqrt(pow(current_centroids[i].first - past_centroids[j].first, 2) + pow(current_centroids[i].second - past_centroids[j].second, 2));
          distances[i].push_back(distance);
        }
      }

      //go through and find the closest current blob to each previous blob and assign the IDs of the previous blobs to the current blobs
      vector<Blob> return_Blobs;
      for (int i = 0; i < distances.size(); i++){
        float min_distance = distances[i][0];
        int min_index = 0;
        for (int j = 1; j <distances[i].size(); j++){
           float distance_compare = distances[i][j];
           if(distance_compare < min_distance){
            min_distance = distance_compare;
            min_index = j;
           }
        
        Blob push_back_Blob;
        push_back_Blob.id = blobs[min_index].id;

        push_back_Blob.age = blobs[min_index].age++;
        
        return_Blobs.push_back(push_back_Blob);  
        }

      }
      
      
      return return_Blobs;
    }
}

//determine if blob is headiing in enter direction or exit direction
//1 is enter and 0 is exit
bool track_blob_direction_and_count (vector<Blob> blobs, const Frame& cur_frame, const Prev_Frame& prev_frame, Counter increment) {
  vector<Blob> current_blobs = which_blob(blobs, cur_frame, prev_frame);
  vector<Blob> past_blobs = prev_frame.prev_blobs;

  uint8_t usable_size = 0;

  if(current_blobs.size() < past_blobs.size()){
    usable_size = current_blobs.size();
  } else if (current_blobs.size() > past_blobs.size()) {
    usable_size = past_blobs.size();
  } else {
    usable_size = past_blobs.size();
  }

  for(int i = 0; i < usable_size; i++){
    float y_centroid_past = past_blobs[i].centroid_y;
    float y_centroid_current = current_blobs[i].centroid_y;
    int count = increment.count;
    if (y_centroid_current > y_centroid_past){
      if(count > 0){
         count -=1;
      } 
    }

    if(y_centroid_current < y_centroid_past){
      count += 1;
    }
  }
  
  
  
  
  
}








































/*
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
  */