#include "ndt_grid.h"

#include "FS.h"
#include "SD.h"

#define MAX_LINE 14000
unsigned char buff[MAX_LINE];  

float ranges[1440];

int readLine( File *file, float *scan )
{
  uint8_t c;
  int len = 0;
  while( 1 ){
    file->read( &c, 1 );
    if( c == '\n' ){
      break;
    }
    else{
      buff[len] = c;
      len ++; 
    }
  }
  buff[len] = '\0';
  
  //Serial.printf("%s\n", buff);

  char str[20];
  int slow = 0, fast = 0;
  float dist = 0;
  int count = 0;
  while (fast != len - 1) {
    if (buff[fast] != ' ') {
      fast++;
    }
    else {
      memset( str, 0, sizeof( str ) );
      memcpy( str, &buff[slow], fast - slow );
      str[fast - slow] = '\0';

      if (strcmp("laser", str) == 0) {
          //printf("%s\n", str);
      }
      else if (strcmp("inf", str) == 0) {
        dist = 65536;
        //printf("dist = %f\n", dist);
        scan[count ++] = dist;
        //count ++;
      }
      else {
        dist = atof(str);
        //printf("dist = %f\n", dist);
        scan[count ++] = dist;
        //count ++;
      }

      slow = fast + 1;
      fast++;
    }
  }

  return count;
}

void laserData2Container( float *ranges, slam::ScanContainer &container )
{
        float angle = -3.14159f;
        container.clear();

        for( int i = 0; i < 1440; i ++ ){
                float dist = ranges[ i ];
                if( dist >= 0.0099999998f && dist <= 14.0000000000f ){
                        Point point;
                        point.x = cos(angle) * dist;
                        point.y = sin(angle) * dist;
                        container.addData( point );
                }

                angle += 0.0043633231f;
        }
        //std::cout<<"Scan Container Size: "<<container.getSize()<<std::endl;
}


void setup() {
  Serial.begin(115200);
  ndt::NDT ndt;

  initSD();       

  listDir(SD, "/", 0);

  File file = SD.open("/laser_data.txt", FILE_READ);
  if(!file){
    Serial.println("无法打开文件进行读取 1");
    return;
  }
  
  Pose robot_pose;
  robot_pose.x = 0;
  robot_pose.y = 0;
  robot_pose.theta = 0;
  
  slam::ScanContainer pre_scan_container;
  slam::ScanContainer cur_scan_container;
  int count = 0;
  bool is_init = false;
  while( count < 1400 ){
    
    memset( ranges, 0, sizeof( ranges ) );
    int line = readLine( &file, ranges );
    //std::cout<<"read : "<<line<<", count : "<<count<<std::endl;
    laserData2Container( ranges, cur_scan_container );  
    
    if( !is_init ){
      pre_scan_container = cur_scan_container;

      is_init = true;
    }
    else {                    
      Pose pose_delta;
      pose_delta.x = 0;
      pose_delta.y = 0;
      pose_delta.theta = 0;

      uint64_t start_time = millis();
      ndt.ndtProcess( pre_scan_container, cur_scan_container, pose_delta, 5 );
      //std::cout<<"pose delta = "<<std::endl<<pose_delta.x<<" "<<pose_delta.y<<" "<<pose_delta.theta<<std::endl;
      if( pose_delta.x < -1 || pose_delta.x > 1 || pose_delta.y < -1 || pose_delta.y > -1 ){
              
        robot_pose.x += pose_delta.x;
        robot_pose.y += pose_delta.y;
        robot_pose.theta += pose_delta.theta;
      }
      uint64_t end_time = millis();
      std::cout<<count<<" "<<end_time - start_time<<std::endl;

      //std::cout<<std::endl<<robot_pose.x<<" "<<robot_pose.y<<" "<<robot_pose.theta;
      pre_scan_container = cur_scan_container;
    }

    count ++;
  }
}
 
void loop() {

}

void initSD()
{
  if(!SD.begin()){
    Serial.println("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD.cardType();

  if(cardType == CARD_NONE){
      Serial.println("No SD card attached");
      return;
  }

  Serial.print("SD Card Type: ");
  if(cardType == CARD_MMC){
      Serial.println("MMC");
  } else if(cardType == CARD_SD){
      Serial.println("SDSC");
  } else if(cardType == CARD_SDHC){
      Serial.println("SDHC");
  } else {
      Serial.println("UNKNOWN");
  }

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);
}

void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
    Serial.printf("Listing directory: %s\n", dirname);

    File root = fs.open(dirname);
    if(!root){
        Serial.println("Failed to open directory");
        return;
    }
    if(!root.isDirectory()){
        Serial.println("Not a directory");
        return;
    }

    File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if(levels){
                listDir(fs, file.name(), levels -1);
            }
        } else {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("  SIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}
