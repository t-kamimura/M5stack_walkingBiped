// test Logger
// 2019/12/22
#define M5STACK_MPU6886 
#include <M5Stack.h>
#define SerialMonitor Serial    // ハードウェアシリアル

////////////////////////////
// ファイル名の設定
////////////////////////////
// "acclogXX.csv"という名前のファイルが生成されるが，"acclog99.csv"が限界
#define LOG_FILE_PREFIX "acclog"    // ファイル名
#define MAX_LOG_FILES 100           // 作成できるログファイル数の上限
#define LOG_FILE_SUFFIX "csv"       // ログファイルの拡張子
char logFileName[13]; // ログファイル名を入れるchar型変数．半角13文字．
// 保存されるデータ
#define LOG_COLUMN_COUNT 4
char * log_col_names[LOG_COLUMN_COUNT] = {
 "time", "accX", "accY", "accZ"
}; // これはファイルの１行目に入る

////////////////////////////
// ログ周期の設定
////////////////////////////
#define LOG_RATE 10 // Log every 5 milliseconds
unsigned long lastLog = 0;

bool fOK = false;

////////////////////////////
// ログ用変数
////////////////////////////
File logFile;
unsigned int time_cur = 0;
float ax = 0.0F;
float ay = 0.0F;
float az = 0.0F;
//
//byte logAccData()
//{
//  logFile = SD.open(logFileName, FILE_WRITE); // Open the log file
//  if (!logFile) {
//    M5.Lcd.println("ERROR: OPEN FILE");
//    while (1) ;    
//  }
// if (logFile)
// { // Print longitude, latitude, altitude (in feet), speed (in mph), course
//   // in (degrees), date, time, and number of satellites.
//   time_cur = millis();
//   M5.IMU.getAccelData(&ax,&ay,&az);
//   logFile.print(time_cur);
//   logFile.print(',');
//   logFile.print(ax,4);
//   logFile.print(',');
//   logFile.print(ay,4);
//   logFile.print(',');
//   logFile.print(az,4);
//   logFile.println();
//   logFile.close();
//
//   return 1; // Return success
// }
//
//}

// printHeader() - prints our eight column names to the top of our log file
void printHeader()
{
 logFile = SD.open(logFileName, FILE_WRITE); // Open the log file
 if (!logFile) {
    M5.Lcd.println("ERROR: OPEN FILE_inPrintHeader");
    while (1) ;    
 }
 if (logFile) // If the log file opened, print our column names to the file
 {
   int i = 0;
   for (; i < LOG_COLUMN_COUNT; i++)
   {
     logFile.print(log_col_names[i]);
     if (i < LOG_COLUMN_COUNT - 1) // If it's anything but the last column
       logFile.print(','); // print a comma
     else // If it's the last column
       logFile.println(); // print a new line
   }
   logFile.close(); // close the file
 }
}

// updateFileName() - Looks through the log files already present on a card,
// and creates a new file with an incremented file index.
void updateFileName()
{
 int i = 0;
 for (; i < MAX_LOG_FILES; i++)
 {
   memset(logFileName, 0, strlen(logFileName)); // Clear logFileName string
   // Set logFileName to "acclogXX.csv":
   sprintf(logFileName, "/%s%d.%s", LOG_FILE_PREFIX, i, LOG_FILE_SUFFIX);
   if (!SD.exists(logFileName)) // If a file doesn't exist
   {
     break; // Break out of this loop. We found our index
   }
   else // Otherwise:
   {
     SerialMonitor.print(logFileName);
     SerialMonitor.println(" exists"); // Print a debug statement
   }
 }
 SerialMonitor.print("File name: ");
 SerialMonitor.println(logFileName); // Debug print the file name
}

void setup() {
  M5.begin();

  // Start SD card
  if (!SD.begin()) {
    M5.Lcd.println("ERROR: SD CARD");
    while (1) ;
  }
  
  M5.IMU.Init();
  
  updateFileName(); // プログラムを始めるたびに新しいファイルを作成
  printHeader();    // ファイルの１行目を書き込む
  
  // Greeting Message
  M5.Lcd.println("testLogger");
}

void loop() {
  String dataString = "";
  
  M5.update();
  
  if (M5.BtnA.wasPressed()) {
      M5.Lcd.println("Start>");
      logFile = SD.open(logFileName, FILE_APPEND); // Open the log file
      if (!logFile) {
          M5.Lcd.println("ERROR: OPEN FILE");
          while (1) ;    
        }
      fOK = true;
  }
  if (M5.BtnB.wasPressed()) {
      fOK = false;
      logFile.close(); // close the file
      M5.Lcd.println("End");
      SerialMonitor.println("data log stopped.");
  }
  
  if (fOK) {
    if ((lastLog + LOG_RATE) <= millis())  //前回のデータからちょっと時間がたっていれば
     {
       time_cur = millis();
       M5.IMU.getAccelData(&ax,&ay,&az);
       logFile.print(time_cur);
       logFile.print(',');
       logFile.print(ax,6);
       logFile.print(',');
       logFile.print(ay,6);
       logFile.print(',');
       logFile.print(az,6);
       logFile.println();
       SerialMonitor.println("ACC logged."); // Print a debug message
       lastLog = millis(); // Update the lastLog variable
     
     }
  }
}
