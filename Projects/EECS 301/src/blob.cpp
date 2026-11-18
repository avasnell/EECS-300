#include "blob.h"
#include <vector>
#include <cstdint>
#include <queue>
#include <map>
#include <algorithm>


using namespace std;

#define PIXEL_COUNT_THRESHOLD 40
#define THRESHOLD_DISTANCE_BOTTOM 20

#define THRESHOLD_DISTANCE_TOP 12
#define MAX_TRACKING_DISTANCE 17.0f

//only used for single person trials
#define SINGLE_BLOB_TRACKING_DISTANCE 22.0f

#define FRAME_PERSISTENCE_LIMIT 1 //Keep a lost Blob alive for _ frames

#define PIXEL_COUNT_THRESHOLD_UPPER 220
#define MIN_PEAK_SEPARATION 20.0f//peaks closer than this many pixels count as one person

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

//returns true if the blob has two hot spots far enough apart to be two people
bool split_blob_by_maxima(const Blob& big, const Frame& frame){
  const auto& T = *frame.matrix_of_temps;

  //find all local peaks(pixels that are as hot or hotter than all 8 neighbors)
  vector<Pixel> peaks;//stores coordinates of every pixel that is a peak
  for (size_t k = 0;k < big.pixels.size();k++){//loops through every pixel in blob
    int r= big.pixels[k].row;
    int c=big.pixels[k].col;
    int v=T[r][c];//temp at that pixel
    
    bool is_peak= true;//assumes pixel is peak until proven otherwise

    //checks the 8 surrounding pixels
    for(int dr = -1; dr <= 1; dr++){//one row up, one down
      for(int dc = -1; dc <= 1; dc++){//one column left, one right
        if(dr == 0 && dc == 0) continue;//does not check pixel against itself
        int nr = r + dr;//row of a neighbour(surrounding) pixel
        int nc = c + dc;
        if(nr >= 0 && nr < 24 && nc >= 0 && nc < 32){//boundary check to make sure not accidentally looking outside 24x32 camera boundart
          int nv = T[nr][nc];//temp of neighbour pixel

          //if neighbor is hotter or same temperature 
          if(nv> v ||(nv == v && (nr * 32 + nc) < (r * 32 + c))){
            //(nr * 32 + nc) is index so (nr * 32 + nc) < (r * 32 + c) checks that neighbour index less than pixel index
            is_peak= false;
          }
        }
      }
    }
    if(is_peak) {
      peaks.push_back(big.pixels[k]);
    }
  }

  //if 1 or fewer peaks found then just keep the original blob,,,so only one person
  if(peaks.size() <= 1){
    return false;
  }

  //find the absolute hottest peak first
  int i1 = 0;//i1 is index number of hottest peak, assume first peak is hottest until proven otherwise
  for (size_t i = 1;i< peaks.size(); i++){
    if(T[peaks[i].row][peaks[i].col] > T[peaks[i1].row][peaks[i1].col]) {//if current peak hotter than i1
      i1= i;
    }
  }
  Pixel peak1=peaks[i1];//stores coordinates of hottest peak in blob

  //find the second peak that is at least MIN_PEAK_SEPARATION away from peak1
  int i2 = -1;//set it as invalid number because index of second peak not found yet
  for (size_t i = 0; i < peaks.size(); i++){
    if (i == i1) continue;//don't compare to first peak
    float d = sqrt(pow((float)(peaks[i].row - peak1.row), 2) + pow((float)(peaks[i].col - peak1.col), 2));//distance between the two peaks
    if (d < MIN_PEAK_SEPARATION) continue;
    
    if (i2 == -1 || T[peaks[i].row][peaks[i].col] > T[peaks[i2].row][peaks[i2].col]) {//if we have not found a second peak yet or we have found one and this new peak is hotter than our current best
      i2 = i;//set index of second peak as current index
    }
  }

  //two hot spots at least min_peak separation away means two peoplee
  return i2 != -1;
}
static std::map<int, int> two_hot_spots;//key is blob id in this frame, value is 1 if it is a big blob with two hot spots
static std::map<int, int> two_people;//key is track id,value is 1 if this track was ever a big blob with two hot spots

vector<Blob> get_Blobs(const Frame& frame){
  two_hot_spots.clear();
  //id number gets defined when creating the vector of blobs
  int id_number = 1;
  bool visited[24][32] = {false}; //initialize visited array to false
    vector<Blob> blobs;
    for (int r = 0; r < 24; r++){
        for (int c = 0; c < 32; c++){
            if ((*frame.matrix_of_temps)[r][c] > 0){
                Blob blob = find_Blob(visited, r, c, frame); 
                if(blob.area > PIXEL_COUNT_THRESHOLD){
                  if (blob.area > PIXEL_COUNT_THRESHOLD_UPPER){
                    // big blob: local maxima check. two hot spots = two people.
                    // the blob is kept in one piece and only marked, so tracking stays stable
                    if(split_blob_by_maxima(blob,frame)){
                      two_hot_spots[id_number] = 1;
                    }
                    blob.id=id_number;
                    id_number++;
                    blobs.push_back(blob);
                    }else{
                      //unchanged area based path
                      blob.id = id_number;
                      id_number++;
                      blobs.push_back(blob);
                    }
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
    q.push({startR,startC});
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
    //if (best_history_idx != -1 && min_dist < MAX_TRACKING_DISTANCE) {

    //fast walker- one blob seen last frame, one new blob and the jump continues
    //in the direction the blob was heading (started left, moving right and vice versa)
    float tracking_distance = MAX_TRACKING_DISTANCE;
    if(best_history_idx != -1 && detected_blobs.size() == 1 && tracked_history.size() == 1){
      const Blob& h = tracked_history[best_history_idx];
      float moved = detected_blobs[i].centroid_y - h.centroid_y;
      float heading = 16.0f - h.first_centroid_y;//16 is frame centre since 32 columns
      if(h.frames_lost == 0 && moved * heading > 0){
        tracking_distance = SINGLE_BLOB_TRACKING_DISTANCE;
      }
    }
    if (best_history_idx != -1 && min_dist < tracking_distance) {
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
      if(two_hot_spots[detected_blobs[i].id] == 1){
        two_people[matched_blob.id] = 1;
      }
    }
  }

  //For and additional blobs that didn't get matched
  for(size_t j = 0; j < tracked_history.size(); j++){
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
    
    new_blob.first_centroid_x = new_blob.centroid_x;
    new_blob.first_centroid_y = new_blob.centroid_y;

    current_tracked_list.push_back(new_blob);
    tracked_history.push_back(new_blob); 
    two_people[new_blob.id]= two_hot_spots[detected_blobs[i].id];
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
           if(two_people[tracked_history[j].id] == 1){ 
            current_count += 1; 
            Serial.println("-> Person Entered Frame!"); //print for second person
          }
       } 
       else if (start_y >= THRESHOLD_DISTANCE_BOTTOM && end_y <= THRESHOLD_DISTANCE_TOP) {
           current_count -= 1; // Bottom-to-Top Exit
           Serial.println("<- Person Exited Frame!");
           if(two_people[tracked_history[j].id] == 1){ 
            current_count -= 1; 
            Serial.println("<- Person Exited Frame!"); 
           }
       }
    }
  }
  return current_count;
}
