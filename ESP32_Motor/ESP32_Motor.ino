#include <ros.h>


/* Left_motorctrl pin define */
#define PWM1 27
#define INA1 26
#define INB1 25
/* Right_motorctrl pin define */
#define PWM2 13
#define INA2 12
#define INB2 14
/* Encoder pin define */
#define L_ENC_A 35
#define L_ENC_B 34
#define R_ENC_A 23
#define R_ENC_B 22

portMUX_TYPE mux0 = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE mux1 = portMUX_INITIALIZER_UNLOCKED;

/* 建立計時器物件 */
hw_timer_t * timer_R = NULL;
hw_timer_t * timer_L = NULL;

/* 宣告任務變數 */
TaskHandle_t Task_Motorcontrol;

ros::NodeHandle nh;

volatile long Encoder_R, Encoder_L;
double LeftMotor_RPM;
double RightMotor_RPM;
int Vel_Right, Vel_Left;
int PPR = 13;               // Encoder 1圈pulse數
int N = 71;


void Motor_control(void * pvParameters){
  // Serial.print("Task_Motorcontrol running on core");
  // Serial.println(xPortGetCoreID());
  int i;
  for(;;){
    digitalWrite(INA1, LOW);
    digitalWrite(INB1, HIGH);
    digitalWrite(INA2, LOW);
    digitalWrite(INB2, HIGH);
    // for(i=80 ; i<=255 ; i++){
      ledcWrite(0, 255);
      ledcWrite(1, 255);
      // Serial.print("Motor PWM: ");
      // Serial.println(i);
      vTaskDelay(200);
    // }
  }
}

/* 轉速計算： 
  轉速(RPM) = trigger_count * 擷取頻率的時間轉換至1s的倍數 * 分鐘轉換 / (Encoder線數 * 減速比 * 中斷觸發方式(RISING, FALLING為1, CHANGE為2))
*/
void IRAM_ATTR R_EncoderISR(){
  portENTER_CRITICAL(&mux0);
  Vel_Right = Encoder_R;
  RightMotor_RPM = Vel_Right * 0.065;  
  Encoder_R = 0;
  portEXIT_CRITICAL(&mux0);
}

void IRAM_ATTR L_EncoderISR(){
  portENTER_CRITICAL(&mux1);
  Vel_Left = Encoder_L;
  LeftMotor_RPM = Vel_Left * 0.065;
  Encoder_L = 0;
  portEXIT_CRITICAL(&mux1);
}

void READ_ENC_R(){
  if(digitalRead(R_ENC_A) == LOW){
    if(digitalRead(R_ENC_B) == LOW)
      Encoder_R--;
    else
      Encoder_R++;
  } else{
    if(digitalRead(R_ENC_B) == LOW)
      Encoder_R++;
    else
      Encoder_R--;
  }
}

void READ_ENC_L(){
  if(digitalRead(L_ENC_A) == LOW){
    if(digitalRead(L_ENC_B) == LOW)
      Encoder_L--;
    else
      Encoder_L++;
  } else{
    if(digitalRead(L_ENC_B) == LOW)
      Encoder_L++;
    else
      Encoder_L--;
  }
}

void Controller_Pinsetup(){
  pinMode(PWM1, OUTPUT);
  pinMode(INA1, OUTPUT);
  pinMode(INB1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(INA2, OUTPUT);
  pinMode(INB2, OUTPUT);
  pinMode(L_ENC_A, INPUT);
  pinMode(L_ENC_B, INPUT);
  pinMode(R_ENC_A, INPUT);
  pinMode(L_ENC_B, INPUT);

  digitalWrite(INA1, HIGH);
  digitalWrite(INB1, HIGH);
  digitalWrite(PWM1, LOW);
  digitalWrite(INA2, HIGH);
  digitalWrite(INB2, HIGH);
  digitalWrite(PWM2, LOW);

  ledcSetup(0, 5000, 8);
  ledcSetup(1, 5000, 8);
  ledcAttachPin(PWM1, 0);
  ledcAttachPin(PWM2, 1);

}

void setup() {
  Serial.begin(115200);
  setCpuFrequencyMhz(240);

  Controller_Pinsetup();

  nh.initNode();

  xTaskCreatePinnedToCore(Motor_control, "Motor Control", 10000, NULL, 1, &Task_Motorcontrol, 1);
  vTaskDelay(500);

  attachInterrupt(R_ENC_A, READ_ENC_R, CHANGE);
  timer_R = timerBegin(0, 80, true);                  // timer為80MHz的頻率，因此需要分頻成1MHz，計時器的step為1us(1*10^-6Hz)
  
  timerAttachInterrupt(timer_R, &R_EncoderISR, true);
  timerAlarmWrite(timer_R, 500000, true);             // 500000 = 500 ms
  timerAlarmEnable(timer_R);

  attachInterrupt(L_ENC_A, READ_ENC_L, CHANGE);
  timer_L = timerBegin(1, 80, true);
  timerAttachInterrupt(timer_L, &L_EncoderISR, true);
  timerAlarmWrite(timer_L, 500000, true);
  timerAlarmEnable(timer_L);
}

void loop() {
  // Serial.print("Right motor:");
  // Serial.println(RightMotor_RPM);
  // Serial.print("Left motor:");
  // Serial.println(LeftMotor_RPM);
  vTaskDelay(200);
}
