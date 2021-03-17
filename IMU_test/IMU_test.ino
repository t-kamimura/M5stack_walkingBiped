#define M5STACK_MPU6886 
#include <M5Stack.h>

float accX = 0.0F;
float accY = 0.0F;
float accZ = 0.0F;

int d = 200;
int x = 160;
int y = 120;
int x_= 160;
int y_= 120;

float data_x;
float data_y;
float data_z;

void setup(){

  // Initialize the M5Stack object
  M5.begin();
  M5.Power.begin();
    
  M5.IMU.Init();

  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(GREEN , BLACK);
  M5.Lcd.setTextSize(1);
}

void loop() {
//  M5.IMU.getGyroData(&gyroX,&gyroY,&gyroZ);
  M5.IMU.getAccelData(&accX,&accY,&accZ);
//  M5.IMU.getAhrsData(&pitch,&roll,&yaw);

  x = -d*accX/accZ + 160;
  y =  d*accY/accZ + 120;

  // ローパスフィルタ
  float ratio = 0.5;
  x_ = ratio*x_ + (1-ratio)*(-d*accX/accZ + 160);
  y_ = ratio*y_ + (1-ratio)*( d*accY/accZ + 120);
  
  M5.Lcd.setCursor(0, 20);
  M5.Lcd.printf(" Gx:%6.3f  Gy:%6.3f  Gz:%6.3f", accX, accY, accZ);
//  M5.Lcd.setCursor(0, 40);
//  M5.Lcd.printf(" X:%d  Y:%d", x, y);
  
  M5.Lcd.drawCircle(160, 120, 30, WHITE); //枠だけ center-x, center-y, radius
//  M5.Lcd.drawCircle(160, 120, 60, WHITE); //枠だけ center-x, center-y, radius
//  M5.Lcd.drawCircle(160, 120, 90, WHITE); //枠だけ center-x, center-y, radius
  M5.Lcd.fillCircle(x_, y_, 8, GREEN); //塗りつぶし center-x, center-y, radius 
  M5.Lcd.fillCircle(x, y, 5, RED); //塗りつぶし center-x, center-y, radius
  delay(50);
  M5.Lcd.fillCircle(x_, y_, 8, BLACK); //塗りつぶし center-x, center-y, radius 
  M5.Lcd.fillCircle(x, y, 5, BLACK); //塗りつぶし center-x, center-y, radius 
  
//  M5.Lcd.fillScreen(BLACK);
}
