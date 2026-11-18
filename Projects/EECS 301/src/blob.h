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
  int id; //tracker id
  float centroid_x; //center x position
  float centroid_y; //center y position
  float max_temp; //max temp of blob
  uint16_t area; //area of blob
  uint16_t max_area=0; //max area of blob in its history
  uint8_t age; //how many frames the blob has been there
  vector<Pixel> pixels; //pixels that make upS the blob
  uint8_t frames_lost;
  int (*matrix_just_blob)[24][32];
  double first_centroid_x = -1; //first x position of blob
  double first_centroid_y = -1; //first y position of blob
  std::map<int, pair<float, float>> ids_inside_to_centroid; //ids to first centroid values
  bool is_merged;
} Blob;


typedef struct{
    int prev_number_of_blobs;
    vector<Blob> prev_blobs;
    int prev_hot_pixels_count;
    int (*prev_matrix_of_temps)[24][32];
    int count;
} Prev_Frame;

typedef struct{
    int number_of_blobs;
    vector<Blob> blobs;
    int hot_pixels_count;
    int (*matrix_of_temps)[24][32];
    int count;
    Prev_Frame previous_frame;
} Frame;

typedef struct{
    int count;
} Counter;

void print_vector_of_Blobs(vector<Blob> vector_blobs);
bool track_blob_direction (int previous_x_cent, int current_x_cent);
vector<Blob> get_Blobs(const Frame& frame);
Blob find_Blob(bool (&visited)[24][32], int startR, int startC, const Frame& frame);
uint16_t area(int tempValues[24][32]);
uint8_t max_temp(int tempValues[24][32]);
bool same_person(Blob blob1, Blob blob2);
float x_centroid(int tempValues[24][32]);
float y_centroid(int tempValues[24][32]);
vector<Blob> which_blob(vector<Blob> blobs, const Frame& cur_frame, const Prev_Frame& prev_frame);
void map_blob_pixels_to_matrix(std::vector<Pixel> pixels, int (&mapped_matrix)[24][32]);
//int track_blob_direction_and_count (vector<Blob> blobs, const Frame& cur_frame, const Prev_Frame& prev_frame, Counter counter);
int count_people_entered(vector<Blob> current_blobs, vector<Blob> past_blobs, int count);
void split_blob_by_maxima(const Blob& big, const Frame& frame, const Prev_Frame& prev_frame);
#endif












void split_blob_by_maxima(const Blob& big, const Frame& frame, const Prev_Frame& prev_frame){
  const auto& T = *frame.matrix_of_temps;
  vector<Blob> result;//empty vector for final separated blobs

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
    result.push_back(big);
    return result;
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

  //if no second peak is far enough away, keep the blob as one person
  if (i2 == -1){
    result.push_back(big);
    return result;
  }
  Pixel peak2 = peaks[i2];

  
  //otherwise, assume blob is two people
  //big blob will have an id, but if is_merged is true, don't count. Instead, just count non-is_merged blobs. A lot big blob variables in the struct will
  //be useless, but it shouldn't matter because all the important information is stored in the smaller blobs that make up the big blob. We will know this info
  //through the map of ids to first centroids.
  //We want to consistently update the first centroid values of the ids found in the vector of for the big blob, based on the hotspot values found in the big blob.
  //split into two cases
    // case 1: blob is two people together (always have been together)
    // case 2: blob is two people that merged together (they were seperate at one time)

 //case 1
 //assume blobs walked into together at the same time, therefore, their centroid values are going to be determined by their hotspots
 //create two new blobs based on hot spot values

 //don't worry about area here, since we know there are two people in the blob, there's no point of diving the area, espcially beacause we don't know if it's two
 //large people, one large one small, etc

  //now iterate through the peaks vector, and create new blobs based on that (mapping will occur in which blob)
  //this is just for get_blobs() to see what is present
  //probably shouldn't even care about mapping blobs in big blob? If centroid values are just based on hotspots, merging doesn't really matter
  //anymore

  for (int i = 0; i < peaks.size(); i++){
    Blob new_person;
    new_person.centroid_x = peaks[i].row;
    new_person.centroid_y = peaks[i].column;
    result.push_back(new_person);
  }

  
  return result;//sends back to get_Blobs()
}



//when looking at the frame, there will be 3 blobs present, but the actual amount of 
//people present will only be 2
void assign_merged_to_ids(Blob big, const Frame& frame, const Prev_Frame& prev_frame){
  
  //assume all previous blobs in previous frame have merged
  // identify which blobs are inside big by looking at previous blob hotspots
  for (int i = 0; i <prev_frame.prev_blobs.size(); i++){

    // track first centroids for counting purposes
    pair<float, float> first_cent;
    first_cent.first = prev_frame.prev_blobs[i].first_centroid_x;
    first_cent.second = prev_frame.prev_blobs[i].first_centroid_y;

    //look at prev_frame[i] blob and map to first cent identified
    big.ids_inside_to_centroid[prev_frame.prev_blobs[i].id] = first_cent; 

    frame.blobs
  }


}