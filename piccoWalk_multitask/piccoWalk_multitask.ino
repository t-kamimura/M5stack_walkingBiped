
#define M5STACK_MPU6886
#include <M5Stack.h>
#include <Wire.h>
#include "Adafruit_PWMServoDriver.h"

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

///////////////////////////////////////////////////
// サーボ関連定数(1)
// 変更しないでください
///////////////////////////////////////////////////

#define SERVOMIN   128  // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX   512  // This is the 'maximum' pulse length count (out of 4096)
#define USMIN      500  // This is the rounded 'minimum' microsecond length based on the minimum pulse of 102
#define USMAX      2500 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 512
#define SERVO_FREQ 50   // Analog servos run at ~50 Hz updates

///////////////////////////////////////////////////
// サーボ関連定数(2)
// 必要に応じて変更してください
///////////////////////////////////////////////////

// 4つのサーボのアドレスを指定
#define ADDR_SERVO_NECK  1
#define ADDR_SERVO_WAIST 2
#define ADDR_SERVO_LEFT  3
#define ADDR_SERVO_RIGHT 4

// サーボの中心位置調整
#define CENTER_SERVO_NECK  90
#define CENTER_SERVO_WAIST 97
#define CENTER_SERVO_LEFT  90
#define CENTER_SERVO_RIGHT 90

///////////////////////////////////////////////////
// 制御関連変数
// 必要に応じて変更してください
///////////////////////////////////////////////////
// // 歩行パラメータ
int twistAngle = 20;
int tiltAngle = 18;
int twistDelay = 150;
int tiltDelay = 180;


///////////////////////////////////////////////////
// 加速度センサー関連
// 変更しないでしてください
///////////////////////////////////////////////////

// 加速度モニター用
float accX = 0.0F;
float accY = 0.0F;
float accZ = 0.0F;
float ax = 0.0F;
float ay = 0.0F;
float az = 0.0F;
int depth = 200;
int circle_x = 160;
int circle_y = 120;

// 保存ファイル名の設定
#define LOG_FILE_PREFIX "acclog"
#define MAX_LOG_FILES 100
#define LOG_FILE_SUFFIX "csv"
char logFileName[13];

// File header
#define LOG_COLUMN_COUNT 4
char * log_col_names[LOG_COLUMN_COUNT] = {"time", "accX", "accY", "accZ"};

///////////////////////////////////////////////////
// 加速度ログ関連
// 変更しないでください
///////////////////////////////////////////////////

// ログ周期の設定
#define LOG_RATE 10 // Log every 10 milliseconds
unsigned long lastLog = 0;

// ファイル書き込み許可
bool fOK = false;

File logFile;
unsigned int time_cur = 0;


///////////////////////////////////////////////////
// サーボ制御関連関数
// 変更しないでください
///////////////////////////////////////////////////

void setServoPulse(uint8_t n, double pulse) {
   double pulselength;
   pulselength = 1000000;   // 1,000,000 us per second
   pulselength /= 50;   // 50 Hz
   pulselength /= 4096;  // 12 bits of resolution
   pulse *= 1000;
   pulse /= pulselength;
   pwm.setPWM(n, 0, pulse);
}

void servo_angle_write(uint8_t n, int Angle) {
  double pulse = Angle;
  pulse = pulse/90 + 0.5;
  setServoPulse(n, pulse);
}

void twist(int angle){
  servo_angle_write(ADDR_SERVO_NECK, CENTER_SERVO_NECK - angle);
  servo_angle_write(ADDR_SERVO_LEFT, CENTER_SERVO_LEFT + angle);
  servo_angle_write(ADDR_SERVO_RIGHT, CENTER_SERVO_RIGHT + angle);
}
void tilt(int angle){
  servo_angle_write(ADDR_SERVO_WAIST, CENTER_SERVO_WAIST - angle);
}

///////////////////////////////////////////////////
// 加速度ログ関連関数
// 変更しないでください
///////////////////////////////////////////////////

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
      Serial.print(logFileName);
      Serial.println(" exists"); // Print a debug statement
    }
  }
  Serial.print("File name: ");
  Serial.println(logFileName); // Debug print the file name
}

void acc_log(void * pvParameters) {
  while(1){  // マルチタスクは別個にループに入れる
    M5.update();

    if (M5.BtnA.wasPressed()) {
        M5.Lcd.setCursor(0, 30);
        M5.Lcd.println("Start>");
        M5.Lcd.setCursor(0, 50);
        M5.Lcd.println("Press B button to end");

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
        M5.Lcd.setCursor(0, 70);
        M5.Lcd.println("End");
        Serial.println("data log stopped.");
    }

    if (fOK) {
      if ((lastLog + LOG_RATE) <= millis())  //前回のデータからちょっと時間がたっていれば
      {
        time_cur = millis();
        M5.IMU.getAccelData(&ax,&ay,&az);
        // ローパスフィルタ
        float ratio = 0.5;  // 大きいとフィルタ強いが，遅れが大きくなる
        accX = ratio * accX + (1-ratio)*ax;
        accY = ratio * accY + (1-ratio)*ay;
        accZ = ratio * accZ + (1-ratio)*az;

        logFile.print(time_cur);
        logFile.print(',');
        logFile.print(accX,6);
        logFile.print(',');
        logFile.print(accY,6);
        logFile.print(',');
        logFile.print(accZ,6);
        logFile.println();
        // Serial.println("ACC logged."); // Print a debug message
        lastLog = millis(); // Update the lastLog variable
      }
    }
  }
}

void acc_monitor(void * pvParameters) {
  while(1){  // マルチタスクは別個にループに入れる

    // M5.IMU.getAccelData(&accX,&accY,&accZ);

    // ローパスフィルタ
    float ratio = 0.5;  // 大きいとフィルタ強いが，遅れが大きくなる
    circle_x = ratio*circle_x + (1-ratio)*(-depth*accX/accZ + 160);
    circle_y = ratio*circle_y + (1-ratio)*( depth*accY/accZ + 120);

    // 画面内に入れる
    if(circle_x <  0){circle_x =   0;}
    if(circle_x >320){circle_x = 320;}
    if(circle_y <  0){circle_y =   0;}
    if(circle_y >240){circle_y = 240;}

    M5.Lcd.setCursor(0, 20);
    M5.Lcd.printf(" Gx:%5.2f  Gy:%5.2f  Gz:%5.2f", accX, accY, accZ);
    M5.Lcd.setCursor(0, 40);
    M5.Lcd.printf(" X:%d  Y:%d", circle_x, circle_y);

    M5.Lcd.drawCircle(160, 120, 20, WHITE); //枠だけ center-x, center-y, radius
    M5.Lcd.drawRect(40, 60, 80, 120, WHITE); //枠だけ left, top, witdh, height
    M5.Lcd.drawRect(200, 60, 80, 120, WHITE); //枠だけ left, top, witdh, height
    M5.Lcd.fillCircle(circle_x, circle_y, 15, GREEN); //塗りつぶし center-x, center-y, radius
    delay(50);
    M5.Lcd.fillCircle(circle_x, circle_y, 15, BLACK); //塗りつぶし center-x, center-y, radius

  }
}

///////////////////////////////////////////////////
// setupは一度だけ実行され，初期設定を行う
// 変更しないでください
///////////////////////////////////////////////////

void setup() {
  M5.begin(true, true, true, true);
  pwm.begin();
  pwm.setPWMFreq(50);
  M5.IMU.Init();

  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(GREEN , BLACK);
  M5.Lcd.setTextSize(2);

  // 姿勢を全部初期位置に戻す
  servo_angle_write(ADDR_SERVO_NECK, CENTER_SERVO_NECK);
  servo_angle_write(ADDR_SERVO_WAIST, CENTER_SERVO_WAIST);
  servo_angle_write(ADDR_SERVO_LEFT, CENTER_SERVO_LEFT);
  servo_angle_write(ADDR_SERVO_RIGHT, CENTER_SERVO_RIGHT);
  delay(2000);

  // ログファイルの作成
  if (!SD.begin()) {
    M5.Lcd.println("ERROR: SD CARD");
    while (1) ;
  }
  updateFileName();
  printHeader();

  // 加速度センサーの値を並列処理で取り続ける
  xTaskCreatePinnedToCore(acc_log,"acc_log",4096,NULL,1,NULL,1);
  // xTaskCreatePinnedToCore(acc_monitor,"acc_monitor",4096,NULL,1,NULL,1);

  M5.Lcd.setCursor(0, 10);
  M5.Lcd.println("Press A button to start");

  M5.Lcd.setCursor(0, 210);
  M5.Lcd.println("     A       B       C");

  delay(1000);
}

///////////////////////////////////////////////////
// loopの中身は電源を切るまで繰り返す
// 指示された部分を変更してください
///////////////////////////////////////////////////

void loop() {
  // M5.update();
  if (fOK)
  {
    // 以下にモーション作成
    servo_angle_write(ADDR_SERVO_NECK,  80);
    servo_angle_write(ADDR_SERVO_WAIST, 100);
    servo_angle_write(ADDR_SERVO_LEFT,  80);
    servo_angle_write(ADDR_SERVO_RIGHT, 100);
    delay(1000);

    // モーションはここまで
  }
  else
  {
    servo_angle_write(ADDR_SERVO_NECK,  CENTER_SERVO_NECK);
    servo_angle_write(ADDR_SERVO_WAIST, CENTER_SERVO_WAIST);
    servo_angle_write(ADDR_SERVO_LEFT,  CENTER_SERVO_LEFT);
    servo_angle_write(ADDR_SERVO_RIGHT, CENTER_SERVO_RIGHT);
    delay(100);
  }

}
