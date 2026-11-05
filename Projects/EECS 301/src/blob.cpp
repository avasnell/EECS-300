#include "blob.h"
#include <vector>
#include <cstdint>
#include <queue>
#include <map>
#include <algorithm>


using namespace std;

#define PIXEL_COUNT_THRESHOLD 40
#define PIXEL_HEAT_THRESHOLD 2
#define THRESHOLD_DISTANCE_BOTTOM 20

#define THRESHOLD_DISTANCE_TOP 12
#define MAX_TRACKING_DISTANCE 17.0f

#define FRAME_PERSISTENCE_LIMIT 1 //Keep a lost Blob alive for _ frames


void print_vector_of_Blobs(vector<Blob> vector_blobs){
  for(int i = 0; i < vector_blobs.size(); i++){
    Serial.print("------------Blob_");
    Serial.print(vector_blobs[i].id);
    Serial.print("------------");
    Serial.println();
    Serial.print("id: ");
    Serial.println(vector_blobs[i].id);
    Serial.print(vector_blobs[i].id);
    Serial.print("_age: ");
    Serial.println(vector_blobs[i].age);
    Serial.print(vector_blobs[i].id);
    Serial.print("_x centroid: ");
    Serial.println(vector_blobs[i].centroid_x);
    Serial.print(vector_blobs[i].id);
    Serial.print("_y centroid: ");
    Serial.println(vector_blobs[i].centroid_y);
    Serial.print(vector_blobs[i].id);
    Serial.print("_max temp: ");
    Serial.println(vector_blobs[i].max_temp);
    Serial.print(vector_blobs[i].id);
    Serial.print("_area: ");
    Serial.println(vector_blobs[i].area);
    Serial.print(vector_blobs[i].id);
    Serial.print("_first_x centroid: ");
    Serial.println(vector_blobs[i].first_centroid_x);
    Serial.print(vector_blobs[i].id);
    Serial.print("_first_y centroid: ");
    Serial.println(vector_blobs[i].first_centroid_y);
    Serial.print(vector_blobs[i].id);
    Serial.print("_frames_lost: ");
    Serial.println(vector_blobs[i].frames_lost);
  }
}

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
// Fixed BFS Blob Finder without stack-pointer bugs
Blob find_Blob(bool (&visited)[24][32], int startR, int startC, const Frame& frame){
    Blob blob;
    blob.area = 0;
    blob.frames_lost = 0;
    
    if (frame.hot_pixels_count == 0) return blob;
    
    std::queue<Pixel> q;    
    q.push({startR, startC});
    visited[startR][startC] = true;

    float sum_weight_x = 0;
    float sum_weight_y = 0;
    float sum_mass = 0;
    uint8_t max_t = 0;

    while (!q.empty()){
        Pixel current_pixel = q.front();
        q.pop();

        blob.pixels.push_back(current_pixel);
        blob.area++;

        // Calculate centroids and max temperature inline during BFS to prevent 
        // secondary loop over-allocation and dangling stack matrix bugs.
        int val = (*frame.matrix_of_temps)[current_pixel.row][current_pixel.col];
        if (val > max_t) max_t = val;
        
        sum_weight_x += val * current_pixel.row;
        sum_weight_y += val * current_pixel.col;
        sum_mass += val;

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

    if (sum_mass > 0) {
        blob.centroid_x = sum_weight_x / sum_mass;
        blob.centroid_y = sum_weight_y / sum_mass;
    } else {
        blob.centroid_x = 0;
        blob.centroid_y = 0;
    }
    blob.max_temp = max_t;
    return blob;
}

//
void map_blob_pixels_to_matrix(std::vector<Pixel> pixels, int (&mapped_matrix)[24][32]){
  for (int r = 0; r < 24; r++) {
    for (int c = 0; c < 32; c++) {
      mapped_matrix[r][c] = 0;
    }
  }
  for (int r = 0; r < 24; r++){
    for (int c = 0; c < 32; c++){
      for (int i = 0; i < pixels.size(); i++){
        if (pixels[i].row ==  r && pixels[i].col == c){
          (mapped_matrix)[r][c] = PIXEL_HEAT_THRESHOLD;
        } else {
          (mapped_matrix)[r][c] = 0;
        }
      }
    }
  }
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

static vector<Blob> tracked_history;
static int id_value = 1;
vector<Blob> which_blob(vector<Blob> detected_blobs, const Frame& cur_frame, const Prev_Frame& prev_frame){
  //create list of currently tracked blobs
  vector<Blob> current_tracked_list;
  // people have found a seat
  //has blob been linked to old ID value 
  vector<bool> detected_matched(detected_blobs.size(), false);
  // seats are taken
  //has historical blob been matched up with blob (prevents multiple current blobs from matching to same historical blob)
  vector<bool> history_matched(tracked_history.size(), false);


  //start iterating through detected blobs
  for (size_t i = 0; i < detected_blobs.size(); i++) {
    float min_dist = 999999.0f;
    int best_history_idx = -1;

    // current blobs look for matches in historical blobs
    for (size_t j = 0; j < tracked_history.size(); j++) {
      //skip already matched history blobs
      if (history_matched[j]) continue;

      float dist = sqrt(pow(detected_blobs[i].centroid_x - tracked_history[j].centroid_x, 2) + 
                        pow(detected_blobs[i].centroid_y - tracked_history[j].centroid_y, 2));
      
      if (dist < min_dist) {
        min_dist = dist;
        best_history_idx = j;
      }
    }

  
    // if a match was found in historical blobs and distance is less than max_tracking to be considred a blob, transfer info
    if (best_history_idx != -1 && min_dist < MAX_TRACKING_DISTANCE) {
      Blob matched_blob = detected_blobs[i];
      matched_blob.id = tracked_history[best_history_idx].id;
      matched_blob.age = tracked_history[best_history_idx].age + 1;
      matched_blob.frames_lost = 0; //reset timer
      
      matched_blob.first_centroid_x = tracked_history[best_history_idx].first_centroid_x;
      matched_blob.first_centroid_y = tracked_history[best_history_idx].first_centroid_y;

      current_tracked_list.push_back(matched_blob);
      //mark that detected blob has been matched and that histroical blob has been matched
      detected_matched[i] = true;
      history_matched[best_history_idx] = true;
      
      // Update history values in place
      tracked_history[best_history_idx] = matched_blob;
    }
  }

  //For and additional blobs that didn't get matched
  for (size_t j = 0; j < tracked_history.size(); j++) {
  if (!history_matched[j]) {
    //keep track of how many frames the blob has been missing for
    tracked_history[j].frames_lost++;
    
    //if under the limit, still push to list of current blobs
    if (tracked_history[j].frames_lost <= FRAME_PERSISTENCE_LIMIT) {
      current_tracked_list.push_back(tracked_history[j]);
    }
  }
}

for (size_t i = 0; i < detected_blobs.size(); i++) {
  if (!detected_matched[i]) {
    Blob new_blob = detected_blobs[i];
    new_blob.id = id_value++; 
    new_blob.age = 0;
    new_blob.frames_lost = 0;
    
    // Anchor their starting gate position permanently right now
    new_blob.first_centroid_x = new_blob.centroid_x;
    new_blob.first_centroid_y = new_blob.centroid_y;

    current_tracked_list.push_back(new_blob);
    tracked_history.push_back(new_blob); 
  }
}

//remove hsitroical blobs that have been lost for too long from the tracked history
tracked_history.erase(
    remove_if(tracked_history.begin(), tracked_history.end(), 
              [](const Blob& b) { return b.frames_lost > FRAME_PERSISTENCE_LIMIT; }), 
    tracked_history.end()
  );

  return current_tracked_list;
}

int count_people_entered(vector<Blob> current_blobs, vector<Blob> past_blobs, int current_count){
  for(size_t j = 0; j < tracked_history.size(); j++) {
    // process crossing checks right before a blob is about to expire from frame dropout
    if (tracked_history[j].frames_lost == FRAME_PERSISTENCE_LIMIT) {
       float start_y = tracked_history[j].first_centroid_y;
       float end_y = tracked_history[j].centroid_y;

  
       if (start_y <= THRESHOLD_DISTANCE_TOP && end_y >= THRESHOLD_DISTANCE_BOTTOM) {
           current_count += 1; // Top-to-Bottom Entry
           Serial.println("-> Person Entered Frame!");
       } 
       else if (start_y >= THRESHOLD_DISTANCE_BOTTOM && end_y <= THRESHOLD_DISTANCE_TOP) {
           current_count -= 1; // Bottom-to-Top Exit
           Serial.println("<- Person Exited Frame!");
       }
    }
  }
  return current_count;
}


/*
vector<Blob> which_blob(vector<Blob> blobs, const Frame& cur_frame, const Prev_Frame& prev_frame){
  vector<Blob> return_Blobs;

  // Safety fallback: if there are no current blobs, return immediately
  if (blobs.empty()) {
      return return_Blobs;
  }

  // If there is exactly 1 blob now and 1 blob before, link them directly
  if (cur_frame.number_of_blobs == 1 && prev_frame.prev_number_of_blobs == 1) {
    blobs[0].id = prev_frame.prev_blobs[0].id;
    blobs[0].age = prev_frame.prev_blobs[0].age + 1; 
    blobs[0].first_centroid_x = prev_frame.prev_blobs[0].first_centroid_x;
    blobs[0].first_centroid_y = prev_frame.prev_blobs[0].first_centroid_y;
    return blobs;
  } 
    
  // Store current centroids
  vector<std::pair<float,float>> current_centroids;
  for (size_t i = 0; i < blobs.size(); i++){
      current_centroids.push_back({blobs[i].centroid_x, blobs[i].centroid_y});
  }
    
  // Store past centroids
  vector<std::pair<float,float>> past_centroids;
  for(size_t i = 0; i < prev_frame.prev_blobs.size(); i++){
      past_centroids.push_back({prev_frame.prev_blobs[i].centroid_x, prev_frame.prev_blobs[i].centroid_y});
  }

  // Calculate distance matrix
  std::map<uint8_t, std::vector<float>> distances;
  for(size_t i = 0; i < current_centroids.size(); i++){
      for(size_t j = 0; j < past_centroids.size(); j++){
        float distance = sqrt(pow(current_centroids[i].first - past_centroids[j].first, 2) + 
                              pow(current_centroids[i].second - past_centroids[j].second, 2));
        distances[i].push_back(distance);
      }
  }

  std::vector<uint8_t> assigned_indices;

  // Find the next available new ID
  int max_id = 0;
  for (size_t i = 0; i < prev_frame.prev_blobs.size(); i++){
      if (prev_frame.prev_blobs[i].id > max_id){
        max_id = prev_frame.prev_blobs[i].id;
      }
  }
  int next_new_id = max_id + 1;

  // Loop through each current blob
  for (size_t i = 0; i < current_centroids.size(); i++){
      float min_distance = 999999.0f;
      int min_index = -1;

      // only calculate tracking matches if history elements actually exist
      if (!past_centroids.empty() && !distances[i].empty()) {
          min_distance = distances[i][0];
          min_index = 0;

          for (size_t j = 1; j < distances[i].size(); j++){
              float distance_compare = distances[i][j];
              bool already_assigned = (find(assigned_indices.begin(), assigned_indices.end(), j) != assigned_indices.end());

              if (!already_assigned && distance_compare < min_distance) {
                  min_distance = distance_compare;
                  min_index = j;
              }
          }
      }
      
      // push back the blob with the appropriate ID and age
      Blob push_back_Blob = blobs[i];

      if (min_index != -1 && min_distance < MIN_DISTANCE_TO_BE_SAME_PERSON) { 
          push_back_Blob.id = prev_frame.prev_blobs[min_index].id;
          push_back_Blob.age = prev_frame.prev_blobs[min_index].age + 1;
          assigned_indices.push_back(min_index);
          push_back_Blob.first_centroid_x = prev_frame.prev_blobs[min_index].first_centroid_x;
          push_back_Blob.first_centroid_y = prev_frame.prev_blobs[min_index].first_centroid_y;
      } else {
        //new person entering the frame
          push_back_Blob.id = next_new_id++;
          push_back_Blob.age = 0; 
          push_back_Blob.first_centroid_x = push_back_Blob.centroid_x;
          push_back_Blob.first_centroid_y = push_back_Blob.centroid_y;

      }
      
      return_Blobs.push_back(push_back_Blob);  
  }
    
  return return_Blobs;
}*/

/*
int count_people_entered(vector<Blob> current_blobs, vector<Blob> past_blobs, int count){
  //Loop through past blobs to see who disappeared
  for(size_t i = 0; i < past_blobs.size(); i++){
    int target_id = past_blobs[i].id;
    
    //distance a centroid must hit to consider exit or entering, this is a threshold to avoid counting people who are just moving around in the frame
  int threshold_distance_1 = THRESHOLD_DISTANCE_1;
    int threshold_distance_2 = THRESHOLD_DISTANCE_2;
    
    //Check if this past person is still present in the current frame
    bool found_in_current = (std::find_if(current_blobs.begin(), current_blobs.end(), 
                             [target_id](const Blob& b) { return b.id == target_id; }) != current_blobs.end());
    
    //If they were there before, but are gone now, they exited or entered completely
    if (!found_in_current) {
      Serial.println("Looking at person with ID: " + String(past_blobs[i].id));
      if((past_blobs[i].centroid_y >= threshold_distance_1) && (past_blobs[i].first_centroid_y <= threshold_distance_2)){
        count -= 1; //Person disappeared near the top (Exited)
      } else if((past_blobs[i].centroid_y <= threshold_distance_2) && (past_blobs[i].first_centroid_y >= threshold_distance_1)) {
        count += 1; //Person disappeared near the bottom (Entered)
      } else {
        count =+ 0;
      }
    }
  }
  
  // Return the total count only AFTER examining every single blob
  return count;
}*/


/*
//determine if blob is heading in enter direction or exit direction
//1 is enter, 0 is exit, 0.5 is stagnant
int track_blob_direction_and_count (vector<Blob> blobs, const Frame& cur_frame, const Prev_Frame& prev_frame, Counter counter) {
  vector<Blob> current_blobs = which_blob(blobs, cur_frame, prev_frame);
  print_vector_of_Blobs(current_blobs);
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
    int count = counter.count;
    if (y_centroid_current > y_centroid_past){
      if(count > 0){
         count -=1;
      } 
    }

    if (y_centroid_current < y_centroid_past){
      count += 1;
    }
  }
  Serial.println("Count: " + String(counter.count));
  return counter.count;
}
  */
