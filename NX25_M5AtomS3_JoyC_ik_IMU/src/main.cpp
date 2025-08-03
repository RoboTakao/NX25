#include <SPIFFS.h>
#include "M5AtomS3.h"
#include <Wire.h>          // I2C setting
#include <PCA9685.h>       //for PCA9685
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <FS.h>

#include <M5GFX.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h> //別途「Adafruit BusIO」ライブラリ必要

#define SERVICE_UUID "1010"
#define CHRX_UUID "1012"
#define CHTX_UUID "1011"

#define LED_WIRE 8 //LED_WIREにポートG8 定義

//JojC設定
byte joyLX=100, joyLY=100, joyRX=100, joyRY=100, joyLSW, joyRSW, joyLDistance, joyRDistance, joyPitch;

BLEServer* pServer = NULL;
BLECharacteristic* pCharTx = NULL;
BLECharacteristic* pCharRx = NULL;

bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t value = 0;

// OLED設定
#define SCREEN_WIDTH 128  //OLED 幅指定
#define SCREEN_HEIGHT 64  //OLED 高さ指定
#define OLED_RESET -1     //リセット端子（未使用-1）

int angZero[] = {100,84,101,86,90,91,88,97,62,90,46,125,94,125,80,83};
int angHome[] = {-3,28,-40,-3,3,-28,40,3,-40,-20,60,40,20,-60,0,0};
int ang0[16];
int ang1[16];
int ang_b[16];
char ang_c[16];
float ts=40;  //10msごとに次のステップに移る
float td=4;   //2回で分割

//PCA9685チャンネル
const uint8_t srv_CH0 = 9; //Right leg1
const uint8_t srv_CH1 = 10; //Right leg2
const uint8_t srv_CH2 = 11; //Right leg3
const uint8_t srv_CH3 = 12; //Right leg4
const uint8_t srv_CH4 = 3; //Left leg1
const uint8_t srv_CH5 = 4; //Left leg2
const uint8_t srv_CH6 = 5; //Left leg3
const uint8_t srv_CH7 = 6; //Left leg4
const uint8_t srv_CH8 = 13; //Right hand1
const uint8_t srv_CH9 = 14; //Right hand2
const uint8_t srv_CH10 = 15; //Right hand3
const uint8_t srv_CH11 = 0; //Left hand1
const uint8_t srv_CH12 = 1; //Left hand2
const uint8_t srv_CH13 = 2; //Left hand3
const uint8_t srv_CH14 = 7; //Head
const uint8_t srv_CH15 = 8; //Hip

const float Pi = 3.141593;

// eye
int Sw = SCREEN_WIDTH;
int Sh = SCREEN_HEIGHT;
int Pe = 60;
int We = 20;
int He = 26;
int Hec = 8;
int ReX = Sw/2 - Pe/2 - We/2;
int LeX = Sw/2 + Pe/2 - We/2;
int eY = Sh/2 - He/2;
int ecY = Sh/2 - Hec/2;
int Re = 6;
int eC = 0;

float L1 = 28.7;
float L2 = 28.7;
float H1 = 13.54;
float H2 = 25.92;
float H3 = 13.54;

float angleX, angleZ;
float acc_angle_x, acc_angle_z;

float preAngleX = 0;
float preAngleZ = 0;

float interval_pid, preInterval_pid, T;

float bZ = 0;

int walk_mode = 0;

// I2Cに接続されたSSD1306用「display」の宣言
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Base_angle
float Theta[3]={0.0,0.0,0.0};

// Forward Step
int base_s[22][16];
  
// Left Step
int l_s[4][16]={
  {-9,27,-38,-9,9,-27,38,9,-40,-20,60,40,20,-60,0,0},
  {-9,27,-38,-29,9,-27,38,29,-40,-20,60,40,20,-60,-10,50},
  {-9,27,-38,-29,9,-27,38,29,-40,-20,60,40,20,-60,0,0},
  {-9,27,-38,-9,9,-27,38,9,-40,-20,60,40,20,-60,0,0}};
// Right Step
int r_s[4][16]={
  {-9,27,-38,-9,9,-27,38,9,-40,-20,60,40,20,-60,0,0},
  {-9,27,-38,-29,9,-27,38,29,-40,-20,60,40,20,-60,10,-50},
  {-9,27,-38,-29,9,-27,38,29,-40,-20,60,40,20,-60,0,0},
  {-9,27,-38,-9,9,-27,38,9,-40,-20,60,40,20,-60,0,0}};
// Left Side Step
int ls_s[13][16]={
  {-4,43,-58,-4,3,-43,58,3,-40,-20,60,40,20,-60,0,0},
  {3,43,-58,3,11,-41,56,11,-40,-20,60,40,20,-60,0,0},
  {9,42,-57,9,16,-39,53,16,-40,-20,60,40,20,-60,0,0},
  {11,41,-56,11,18,-38,52,18,-40,-20,60,40,20,-60,0,0},
  {11,41,-56,11,23,-54,80,23,-40,-20,60,40,20,-60,0,0},
  {11,41,-56,11,42,-38,52,42,-40,-20,60,40,20,-60,0,0},
  {11,41,-56,11,35,-7,18,35,-40,-20,60,40,20,-60,0,0},
  {-15,39,-54,-15,14,-40,55,14,-40,-20,60,40,20,-60,0,0},
  {-36,-0,-10,-36,-12,-41,56,-12,-40,-20,60,40,20,-60,0,0},
  {-43,36,-50,-63,-12,-41,56,-12,-40,-20,60,40,20,-60,0,0},
  {-24,54,-79,-24,-12,-41,56,-12,-40,-20,60,40,20,-60,0,0},
  {-19,37,-51,-19,-12,-41,56,-12,-40,-20,60,40,20,-60,0,0},
  {-4,43,-58,-4,3,-43,58,3,-40,-20,60,40,20,-60,0,0}};
// Right Side Step
int rs_s[13][16]={
  {-3,43,-58,-3,4,-43,58,4,-40,-20,60,40,20,-60,0,0},
  {-11,41,-56,-11,-3,-43,58,-3,-40,-20,60,40,20,-60,0,0},
  {-16,39,-53,-16,-9,-42,57,-9,-40,-20,60,40,20,-60,0,0},
  {-18,38,-52,-18,-11,-41,56,-11,-40,-20,60,40,20,-60,0,0},
  {-23,54,-80,-23,-11,-41,56,-11,-40,-20,60,40,20,-60,0,0},
  {-42,38,-52,-42,-11,-41,56,-11,-40,-20,60,40,20,-60,0,0},
  {-35,7,-18,-35,-11,-41,56,-11,-40,-20,60,40,20,-60,0,0},
  {-14,40,-55,-14,15,-39,54,15,-40,-20,60,40,20,-60,0,0},
  {12,41,-56,12,36,0,10,36,-40,-20,60,40,20,-60,0,0},
  {12,41,-56,12,43,-36,50,63,-40,-20,60,40,20,-60,0,0},
  {12,41,-56,12,24,-54,79,24,-40,-20,60,40,20,-60,0,0},
  {12,41,-56,12,19,-37,51,19,-40,-20,60,40,20,-60,0,0},
  {-3,43,-58,-3,4,-43,58,4,-40,-20,60,40,20,-60,0,0}};
// Left Head Step
int lh_s[16]={-4,50,-2,-4,4,17,47,4,-40,-20,100,-90,20,-40,30,-30};
// Right Head Step
int rh_s[16]={-4,-17,-47,-4,4,-50,2,4,90,-20,40,40,20,-100,-30,30};
// Center Step
int c_s[16]={-3,28,-40,-3,3,-28,40,3,-40,-20,60,40,20,-60,0,0};
// Center Step
int h_s[16]={-3,28,-40,-3,3,-28,40,3,-40,-20,60,40,20,-60,0,0};

//PCA9685のアドレス指定
PCA9685 pwm = PCA9685(0x40);

#define SERVOMIN 104            //Min pulse width (12bit 500μs) 
#define SERVOMAX 512            //Max pulse width (12bit 2500μs) 

void servo_write(int ch, int ang){
  ang = map(ang, 0, 180, SERVOMIN, SERVOMAX); //angle（0～180）-> pulse width（150～500）
  pwm.setPWM(ch, 0, ang);
}

void servo_set()
{
  int a[16],b[16];

  for (int j=0; j <=15 ; j++){
      a[j] = ang1[j] - ang0[j];
      b[j] = ang0[j];
      ang0[j] = ang1[j];
  }
  
  for (int k=0; k <= td; k++){

      servo_write(srv_CH0,a[0]*float(k)/td+b[0]);
      servo_write(srv_CH1,a[1]*float(k)/td+b[1]);
      servo_write(srv_CH2,a[2]*float(k)/td+b[2]);
      servo_write(srv_CH3,a[3]*float(k)/td+b[3]);
      servo_write(srv_CH4,a[4]*float(k)/td+b[4]);
      servo_write(srv_CH5,a[5]*float(k)/td+b[5]);
      servo_write(srv_CH6,a[6]*float(k)/td+b[6]);
      servo_write(srv_CH7,a[7]*float(k)/td+b[7]);
      servo_write(srv_CH8,a[8]*float(k)/td+b[8]);
      servo_write(srv_CH9,a[9]*float(k)/td+b[9]);
      servo_write(srv_CH10,a[10]*float(k)/td+b[10]);
      servo_write(srv_CH11,a[11]*float(k)/td+b[11]);
      servo_write(srv_CH12,a[12]*float(k)/td+b[12]);
      servo_write(srv_CH13,a[13]*float(k)/td+b[13]);
      servo_write(srv_CH14,a[14]*float(k)/td+b[14]);
      servo_write(srv_CH15,a[15]*float(k)/td+b[15]);

      delay(ts/td);
  }
}

void face_clear(){
  display.clearDisplay();     //表示クリア
}

void face(){
  face_clear();
  display.fillRoundRect(ReX + eC, eY, We, He, Re,WHITE);      //Rounded Rectangle（Filled）
  display.fillRoundRect(LeX + eC, eY, We, He, Re,WHITE);      //Rounded Rectangle（Filled）
  display.display();  //表示実行
}

void face_center_eye(void *pvParameters){
  while(1)
  {
    face();
    delay(2500);
    face_clear();
    display.fillRect(ReX + eC, ecY, We, Hec,WHITE);      //Rectangle（Filled）
    display.fillRect(LeX + eC, ecY, We, Hec,WHITE);      //Rectangle（Filled）
    display.display();  //表示実行
    delay(200);
  }
}
void ik(float *X3,float *Y3,float *Z3,float *Theta1_deg,float *Theta2_deg,float *Theta3_deg)
{
	float Theta3_rad = atan2(*Z3 , *Y3);

	float Y2 = *Y3 / cos(Theta3_rad) - (H1 + H2 + H3);

  float Theta1_rad = -acos((pow(*X3,2) + pow(Y2,2) + pow(L1,2) - pow(L2,2))/(2*L1*sqrt(pow(*X3,2) + pow(Y2,2)))) + atan2(Y2,*X3);
	float Theta2_rad = atan2(Y2 - L1 * sin(Theta1_rad),*X3 - L1 * cos(Theta1_rad)) - Theta1_rad;

	*Theta1_deg = Theta1_rad / Pi * 180;
	*Theta2_deg = Theta2_rad / Pi * 180;
	*Theta3_deg = Theta3_rad / Pi * 180;
}

void angle_cul_RF(float *Theta_1, float *Theta_2, float *Theta_3, int *cs_0, int *cs_1, int *cs_2, int *cs_3)
{
  *cs_0 = int(*Theta_3);
  *cs_1 = 90 - int(*Theta_1);
  *cs_2 = 90 - int(*Theta_1 + *Theta_2);
  *cs_3 = *cs_0;
}

void angle_cul_LF(float *Theta_1, float *Theta_2, float *Theta_3, int *cs_0, int *cs_1, int *cs_2, int *cs_3)
{
  *cs_0 = int(*Theta_3);
  *cs_1 = - 90 + int(*Theta_1);
  *cs_2 = - 90 + int(*Theta_1 + *Theta_2);
  *cs_3 = *cs_0;
}

void IMU_data(){
  float fK = 0.5;
  auto imu_update = AtomS3.Imu.update();
    if (imu_update) {
        auto data = AtomS3.Imu.getImuData();

        acc_angle_x = atan2(-data.accel.y, data.accel.z) * 180 / Pi -90;
        acc_angle_z = atan2(data.accel.x, -data.accel.y) * 180 / Pi;

        //ローパスフィルター
        angleX = fK*acc_angle_x+(1.0-fK)*preAngleX;
        angleZ = fK*acc_angle_z+(1.0-fK)*preAngleZ;

        preAngleX = angleX;
        preAngleZ = angleZ;
    }
}

void fwd_bk_step(float Fws,float ArmSwg)
{
  float SideS = 20;
  float CenterX = -8;
  float Stance = 0;
  float CenterZ = 0;
  float H0 = 100;
  float Up = 20;
  float ArmStt = 20;
  float ArmAng = 20;
  float ElbAng = 60;

  float eZ, eZ_pre = 0;
  float deZ;
  float ieZ = 0;
  float uZ;

  float Kp = 0.05;
  float Kd = 0.0;
  float Ki = 0.0;

  ts=15;  //15msごとに次のステップに移る
  td=1;   //1回で分割
  
  for (int i=0; i <=21 ; i++){

    // For PID but it's not woriking
    //interval_pid = millis() - preInterval_pid;
    //preInterval_pid = millis();
    //T = interval_pid * 0.001;

    //IMU_data();

    //eZ = angleZ;
    //deZ = (eZ - eZ_pre)/T;
    //ieZ = ieZ + (eZ + eZ_pre)*T/2;
    //uZ = Kp*eZ + Ki*ieZ + Kd*deZ;
    //eZ_pre = eZ;

    //bZ = bZ + uZ;

    //bZ = map(angleZ, -90, 90, -100, 100);
    //if (bZ >= 10) bZ = 10;
    //if (bZ <= -10) bZ = -10;

    bZ = 0;

    float ankle = 30;

    float Step_base[22][14]={
    {Fws+CenterX     ,H0   ,-Stance-CenterZ-bZ                ,-Fws+CenterX    ,H0   ,Stance-CenterZ-bZ                 ,-ArmStt-ArmSwg    ,-ArmAng,ElbAng,ArmStt-ArmSwg    ,ArmAng,-ElbAng,0,0},
    {Fws/3*2+CenterX ,H0   ,SideS*sin(Pi/6)-Stance-CenterZ-bZ ,-Fws/3*4+CenterX,H0   ,SideS*sin(Pi/6)+Stance-CenterZ-bZ ,-ArmStt-ArmSwg/6*5,-ArmAng,ElbAng,ArmStt-ArmSwg/6*5,ArmAng,-ElbAng,0,0},
    {Fws/3+CenterX   ,H0   ,SideS*sin(Pi/3)-Stance-CenterZ-bZ ,-Fws/3*5+CenterX,H0   ,SideS*sin(Pi/3)+Stance-CenterZ-bZ ,-ArmStt-ArmSwg/6*4,-ArmAng,ElbAng,ArmStt-ArmSwg/6*4,ArmAng,-ElbAng,0,0},
    {CenterX         ,H0   ,SideS-Stance-CenterZ-bZ           ,-2*Fws+CenterX  ,H0   ,SideS+Stance-CenterZ-bZ           ,-ArmStt-ArmSwg/6*3,-ArmAng,ElbAng,ArmStt-ArmSwg/6*3,ArmAng,-ElbAng,0,0},
    {CenterX         ,H0   ,SideS-Stance-CenterZ-bZ           ,-2*Fws+CenterX  ,H0-Up,SideS+Stance-CenterZ-bZ           ,-ArmStt-ArmSwg/6*2,-ArmAng,ElbAng,ArmStt-ArmSwg/6*2,ArmAng,-ElbAng,0,0},
    {CenterX         ,H0   ,SideS-Stance-CenterZ-bZ           ,-2*Fws+CenterX  ,H0-Up,SideS+Stance-CenterZ-bZ           ,-ArmStt-ArmSwg/6  ,-ArmAng,ElbAng,ArmStt-ArmSwg/6  ,ArmAng,-ElbAng,0,0},
    {CenterX         ,H0   ,SideS-Stance-CenterZ-bZ           ,CenterX         ,H0-Up,SideS+Stance-CenterZ-bZ           ,-ArmStt           ,-ArmAng,ElbAng,ArmStt           ,ArmAng,-ElbAng,0,0},
    {CenterX         ,H0   ,SideS-Stance-CenterZ-bZ           ,2*Fws+CenterX   ,H0-Up,SideS+Stance-CenterZ-bZ           ,-ArmStt+ArmSwg/5  ,-ArmAng,ElbAng,ArmStt+ArmSwg/5  ,ArmAng,-ElbAng,0,0},
    {CenterX         ,H0   ,SideS-Stance-CenterZ-bZ           ,2*Fws+CenterX   ,H0   ,SideS+Stance-CenterZ-bZ           ,-ArmStt+ArmSwg/5*2,-ArmAng,ElbAng,ArmStt+ArmSwg/5*2,ArmAng,-ElbAng,0,0},
    {-Fws/3+CenterX  ,H0   ,SideS*sin(Pi/3)-Stance-CenterZ-bZ ,Fws/3*5+CenterX ,H0   ,SideS*sin(Pi/3)+Stance-CenterZ-bZ ,-ArmStt+ArmSwg/5*3,-ArmAng,ElbAng,ArmStt+ArmSwg/5*3,ArmAng,-ElbAng,0,0},
    {-Fws/3*2+CenterX,H0   ,SideS*sin(Pi/6)-Stance-CenterZ-bZ ,Fws/3*4+CenterX ,H0   ,SideS*sin(Pi/6)+Stance-CenterZ-bZ ,-ArmStt+ArmSwg/5*4,-ArmAng,ElbAng,ArmStt+ArmSwg/5*4,ArmAng,-ElbAng,0,0},
    {-Fws+CenterX    ,H0   ,-Stance-CenterZ-bZ                ,Fws+CenterX     ,H0   ,Stance-CenterZ-bZ                 ,-ArmStt+ArmSwg    ,-ArmAng,ElbAng,ArmStt+ArmSwg    ,ArmAng,-ElbAng,0,0},
    {-Fws/3*4+CenterX,H0   ,-SideS*sin(Pi/6)-Stance-CenterZ-bZ,Fws/3*2+CenterX ,H0   ,-SideS*sin(Pi/6)+Stance-CenterZ-bZ,-ArmStt+ArmSwg/6*5,-ArmAng,ElbAng,ArmStt+ArmSwg/6*5,ArmAng,-ElbAng,0,0},
    {-Fws/3*5+CenterX,H0   ,-SideS*sin(Pi/3)-Stance-CenterZ-bZ,Fws/3+CenterX   ,H0   ,-SideS*sin(Pi/3)+Stance-CenterZ-bZ,-ArmStt+ArmSwg/6*4,-ArmAng,ElbAng,ArmStt+ArmSwg/6*4,ArmAng,-ElbAng,0,0},
    {-2*Fws+CenterX  ,H0   ,-SideS-Stance-CenterZ-bZ          ,CenterX         ,H0   ,-SideS+Stance-CenterZ-bZ          ,-ArmStt+ArmSwg/6*3,-ArmAng,ElbAng,ArmStt+ArmSwg/6*3,ArmAng,-ElbAng,0,0},
    {-2*Fws+CenterX  ,H0-Up,-SideS-Stance-CenterZ-bZ          ,CenterX         ,H0   ,-SideS+Stance-CenterZ-bZ          ,-ArmStt+ArmSwg/6*2,-ArmAng,ElbAng,ArmStt+ArmSwg/6*2,ArmAng,-ElbAng,0,0},
    {-2*Fws+CenterX  ,H0-Up,-SideS-Stance-CenterZ-bZ          ,CenterX         ,H0   ,-SideS+Stance-CenterZ-bZ          ,-ArmStt+ArmSwg/6  ,-ArmAng,ElbAng,ArmStt+ArmSwg/6  ,ArmAng,-ElbAng,0,0},
    {CenterX         ,H0-Up,-SideS-Stance-CenterZ-bZ          ,CenterX         ,H0   ,-SideS+Stance-CenterZ-bZ          ,-ArmStt           ,-ArmAng,ElbAng,ArmStt           ,ArmAng,-ElbAng,0,0},
    {2*Fws+CenterX   ,H0-Up,-SideS-Stance-CenterZ-bZ          ,CenterX         ,H0   ,-SideS+Stance-CenterZ-bZ          ,-ArmStt-ArmSwg/5  ,-ArmAng,ElbAng,ArmStt-ArmSwg/5  ,ArmAng,-ElbAng,0,0},
    {2*Fws+CenterX   ,H0   ,-SideS-Stance-CenterZ-bZ          ,CenterX         ,H0      ,-SideS+Stance-CenterZ-bZ          ,-ArmStt-ArmSwg/5*2,-ArmAng,ElbAng,ArmStt-ArmSwg/5*2,ArmAng,-ElbAng,0,0},
    {Fws/3*5+CenterX ,H0   ,-SideS*sin(Pi/3)-Stance-CenterZ-bZ,-Fws/3+CenterX  ,H0   ,-SideS*sin(Pi/3)+Stance-CenterZ-bZ,-ArmStt-ArmSwg/5*3,-ArmAng,ElbAng,ArmStt-ArmSwg/5*3,ArmAng,-ElbAng,0,0},
    {Fws/3*4+CenterX ,H0   ,-SideS*sin(Pi/6)-Stance-CenterZ-bZ,-Fws/3*2+CenterX,H0   ,-SideS*sin(Pi/6)+Stance-CenterZ-bZ,-ArmStt-ArmSwg/5*4,-ArmAng,ElbAng,ArmStt-ArmSwg/5*4,ArmAng,-ElbAng,0,0}};

    ik(&Step_base[i][0],&Step_base[i][1],&Step_base[i][2],&Theta[0],&Theta[1],&Theta[2]); //Right Leg
    angle_cul_RF(&Theta[0], &Theta[1], &Theta[2], &base_s[i][0], &base_s[i][1], &base_s[i][2], &base_s[i][3]);

    ik(&Step_base[i][3],&Step_base[i][4],&Step_base[i][5],&Theta[0],&Theta[1],&Theta[2]); //Left Leg
    angle_cul_LF(&Theta[0], &Theta[1], &Theta[2], &base_s[i][4], &base_s[i][5], &base_s[i][6], &base_s[i][7]);

    base_s[4][7] = base_s[4][7] + ankle;
    base_s[15][3] = base_s[15][3] - ankle;

    for (int k=6; k <=13 ; k++){
      base_s[i][k+2] = Step_base[i][k];
    }

    for (int j=0; j <=15 ; j++){
      ang1[j] = angZero[j] + base_s[i][j];
    }
    servo_set();
  }
}

void forward_step(){
  fwd_bk_step(10,20);  //Fwd Step, Arm Swing
}

void back_step(){
  fwd_bk_step(-10,-20);  //Fwd Step, Arm Swing
}

void left_step()
{
  ts=40;  //40msごとに次のステップに移る
  td=4;   //4回で分割

  for (int i=0; i <=3 ; i++){
    for (int j=0; j <=15 ; j++){
      ang1[j] = angZero[j] + l_s[i][j];
    }
  servo_set();
  }
}

void right_step()
{
  ts=40;  //40msごとに次のステップに移る
  td=4;   //4回で分割

  for (int i=0; i <=3 ; i++){
    for (int j=0; j <=15; j++){
      ang1[j] = angZero[j] + r_s[i][j];
    }
  servo_set();
  }
}

void left_side_step()
{
  ts=40;  //40msごとに次のステップに移る
  td=4;   //4回で分割

  for (int i=0; i <=12 ; i++){
    for (int j=0; j <=15 ; j++){
      ang1[j] = angZero[j] + ls_s[i][j];
    }
  servo_set();
  }
}

void right_side_step()
{
  ts=40;  //40msごとに次のステップに移る
  td=4;   //4回で分割

  for (int i=0; i <=12 ; i++){
    for (int j=0; j <=15 ; j++){
      ang1[j] = angZero[j] + rs_s[i][j];
    }
  servo_set();
  }
}

void left_head_step()
{
  ts=40;  //40msごとに次のステップに移る
  td=4;   //4回で分割

  for (int j=0; j <=15 ; j++){
    ang1[j] = angZero[j] + lh_s[j];
  }
  servo_set();
}

void right_head_step()
{
  ts=40;  //40msごとに次のステップに移る
  td=4;   //4回で分割

  for (int j=0; j <=15 ; j++){
    ang1[j] = angZero[j] + rh_s[j];
  }
  servo_set();
}

void center_step()
{
  ts=40;  //40msごとに次のステップに移る
  td=4;   //4回で分割

  for (int j=0; j <=15 ; j++){
    ang1[j] = angZero[j] + c_s[j];
  }
  servo_set();
}

void hand_step(float hsRH, float shRH, float esRH, float hsLH, float shLH, float esLH, float hAng)
{
  ts=40;  //40msごとに次のステップに移る
  td=4;   //4回で分割

  h_s[8] = hsRH;
  h_s[9] = shRH;
  h_s[10] = esRH;
  h_s[11] = hsLH;
  h_s[12] = shLH;
  h_s[13] = esLH;
  h_s[14] = -hAng;
  h_s[15] = hAng;
  for (int j=0; j <=15 ; j++){
    ang1[j] = angZero[j] + h_s[j];
  }
  servo_set();
}

class MyCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pChar) {
    std::string value = pChar->getValue();
    if (value.length()>0) {
      joyLX=value[0];
      joyLY=value[1];
      joyRX=value[2];
      joyRY=value[3];
      //joyLDistance=value[4];
      //joyRDistance=value[5];
      joyLSW=value[4];
      joyRSW=value[5];
      joyPitch=value[6];
    }
  }
};

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

void setupBLE() {
  BLEDevice::init("NX25_M5CoreS3");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharTx = pService->createCharacteristic(CHTX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pCharRx = pService->createCharacteristic(CHRX_UUID, BLECharacteristic::PROPERTY_WRITE_NR);
  pCharRx ->setCallbacks(new MyCallbacks());
  pCharTx->addDescriptor(new BLE2902());
  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");
}

void checkBLE() {
  // notify changed value
  if (deviceConnected) {
      pCharTx->setValue((uint8_t*)&value, 6);
      pCharTx->notify();
  }
  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
      delay(500); // give the bluetooth stack the chance to get things ready
      pServer->startAdvertising(); // restart advertising
      Serial.println("start advertising");
      oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
      // do stuff here on connecting
      oldDeviceConnected = deviceConnected;
  }
}

void StartUpScreen()  //Start Up Screen
{
  int screen_step = 100;
  M5.Lcd.fillScreen(BLUE);
  M5.Lcd.drawBmpFile(SPIFFS, "/NX25.bmp", 0, 0);
  delay(500);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setTextColor(GREEN, BLACK);
  M5.Lcd.setFont(&fonts::FreeMono9pt7b);
  M5.Lcd.drawString("Robo Tskao", 5, 20);
  M5.Lcd.drawString("NX25 v0.2", 5, 40);
  M5.Lcd.drawRect(5, 60, 115, 10, GREEN);
  for(int i=0; i <=5 ; i++){
    M5.Lcd.fillRect(5, 60, i*23, 10, GREEN);
    delay(100);
  }
  delay(200);
  M5.Lcd.fillScreen(BLACK);
  delay(200);
  M5.Lcd.fillRoundRect(5, 85, 118, 40, 7, RED);
  M5.Lcd.setTextColor(CYAN, BLACK);
  M5.Lcd.setFont(&fonts::FreeMono9pt7b);
  M5.Lcd.drawString("BLE", 5, 2);
  M5.Lcd.setTextColor(BLACK, RED);
  M5.Lcd.setFont(&fonts::FreeMonoBold9pt7b);
  M5.Lcd.drawString(" NA ", 75, 2);
  M5.Lcd.setTextColor(CYAN, BLACK);
  M5.Lcd.setFont(&fonts::FreeMono9pt7b);
  M5.Lcd.drawString("FACE", 5, 22);
  M5.Lcd.setTextColor(BLACK, RED);
  M5.Lcd.setFont(&fonts::FreeMonoBold9pt7b);
  M5.Lcd.drawString(" NA ", 75, 22);
  M5.Lcd.setTextColor(CYAN, BLACK);
  M5.Lcd.setFont(&fonts::FreeMono9pt7b);
  M5.Lcd.drawString("IMU", 5, 42);
  M5.Lcd.setTextColor(BLACK, RED);
  M5.Lcd.setFont(&fonts::FreeMonoBold9pt7b);
  M5.Lcd.drawString(" NA ", 75, 42);
  M5.Lcd.setTextColor(CYAN, BLACK);
  M5.Lcd.setFont(&fonts::FreeMono9pt7b);
  M5.Lcd.drawString("SERVO", 5, 62);
  M5.Lcd.setTextColor(BLACK, RED);
  M5.Lcd.setFont(&fonts::FreeMonoBold9pt7b);
  M5.Lcd.drawString(" NA ", 75, 62);
  M5.Lcd.fillRoundRect(5, 85, 118, 40, 7, RED);
  delay(200);
  M5.Lcd.setTextColor(BLACK, GREEN);
  M5.Lcd.setFont(&fonts::FreeMonoBold9pt7b);
  M5.Lcd.drawString(" OK ", 75, 2);
  delay(screen_step);
  M5.Lcd.setTextColor(BLACK, GREEN);
  M5.Lcd.setFont(&fonts::FreeMonoBold9pt7b);
  M5.Lcd.drawString(" OK ", 75, 22);
  delay(screen_step);
  M5.Lcd.setTextColor(BLACK, YELLOW);
  M5.Lcd.setFont(&fonts::FreeMonoBold9pt7b);
  M5.Lcd.drawString(" NA ", 75, 42);
  delay(screen_step);
  M5.Lcd.setTextColor(BLACK, GREEN);
  M5.Lcd.setFont(&fonts::FreeMonoBold9pt7b);
  M5.Lcd.drawString(" OK ", 75, 62);
  delay(screen_step);
  M5.Lcd.fillRoundRect(5, 85, 118, 40, 7, BLUE);
}

void setup() 
{ 
  auto cfg = M5.config();
  AtomS3.begin(cfg);
  Serial.begin(115200);
  M5.Lcd.setRotation(2);
  setupBLE();

  IMU_data();

  // SPIFFSの初期化
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS mount failed");
    return;
  }

  pinMode(LED_WIRE, OUTPUT); //LED_WIRE（ポートG8）を出力に設定
  digitalWrite(LED_WIRE, LOW);   // turn the LED on (HIGH is the voltage level)

  Wire.begin(6, 5); //SDA-6, SCL-5
  
  pwm.begin();                   //initial setting (for 0x40) (PCA9685)
  pwm.setPWMFreq(50);            //PWM 50Hz (for 0x40) (PCA9685)

  // OLED初期設定
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306:0 allocation failed"));
    for (;;); //エラーなら無限ループ
  }
  // OLED表示設定
  display.setTextColor(SSD1306_WHITE);  //文字色

  //initial servo angle
  for (int j=0; j <=15 ; j++){
      ang0[j] = angZero[j] + angHome[j];
  }
  for (int j=0; j <=15 ; j++){
      ang1[j] = angZero[j] + angHome[j];
  }

  StartUpScreen();
  
  digitalWrite(LED_WIRE, HIGH);   // turn the LED on (HIGH is the voltage level)
  servo_set();

  xTaskCreatePinnedToCore(face_center_eye, "face_center_eye", 4096, NULL, 1, NULL, 1); //priority 1 , CORE1
} 
 
void loop() 
{   
  checkBLE();

  if (joyRSW == 1)
    {
      walk_mode = walk_mode + 1;
      if(walk_mode >=3) walk_mode = 0;
      delay(500);
    }

  if(walk_mode == 0)
    {
      M5.Lcd.fillRoundRect(5, 85, 118, 40, 7, BLUE);
      M5.Lcd.setTextColor(BLACK, BLUE);
      M5.Lcd.setFont(&fonts::FreeMonoBold12pt7b);
      M5.Lcd.drawString("WALK", 10, 95);

      if (joyRY > 150)
      {
       forward_step();
      }

      if (joyRY < 50)
      {
        back_step();
      }

      if (joyRX > 150)
      {
        left_step();
      }

      if (joyRX < 50)
      {
        right_step();
      }

      if (joyLX > 150)
      {
        left_side_step();
      }

      if (joyLX < 50)
      {
        right_side_step();
      }

      if (((joyRY <= 150) && (joyRY >= 50)) && ((joyRX <= 150) && (joyRX >= 50)))
      {
        center_step();
      }
    }

    if(walk_mode == 1)
    {
      M5.Lcd.fillRoundRect(5, 85, 118, 40, 7, ORANGE);
      M5.Lcd.setTextColor(BLACK, ORANGE);
      M5.Lcd.setFont(&fonts::FreeMonoBold12pt7b);
      M5.Lcd.drawString("FIGHT", 10, 95);

      if (joyRX > 150)
      {
        left_head_step();
      }

      if (joyRX < 50)
      {
        right_head_step();
      }

      if (((joyRY <= 150) && (joyRY >= 50)) && ((joyRX <= 150) && (joyRX >= 50)))
      {
        center_step();
      }
    }

    if(walk_mode == 2)
    {
      M5.Lcd.fillRoundRect(5, 85, 118, 40, 7, PURPLE);
      M5.Lcd.setTextColor(BLACK, PURPLE);
      M5.Lcd.setFont(&fonts::FreeMonoBold12pt7b);
      M5.Lcd.drawString("HAND", 10, 95);

      float ArmSwgLH = map(joyRY,0,200,60,-120);
      float ElbowSwgLH = map(joyRY,0,200,0,-80);
      float SholSwgLH = map(joyRX,0,100,90,10);
      float ArmSwgRH = map(joyLY,0,200,-60,120);
      float ElbowSwgRH = map(joyLY,0,200,0,80);
      float SholSwgRH = map(joyLX,100,200,-10,-90);
      float hipAngle = map(joyPitch,0,200,-45,45);
      hand_step(ArmSwgRH,SholSwgRH,ElbowSwgRH,ArmSwgLH,SholSwgLH,ElbowSwgLH,hipAngle);
    }
} 