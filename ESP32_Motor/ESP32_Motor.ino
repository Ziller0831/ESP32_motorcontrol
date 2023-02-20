// Left_motorctrl pin define
#define PWM1 27
#define INA1 26
#define INB1 25
// Right_motorctrl pin define
#define PWM2 13
#define INA2 12
#define INB2 14
// Encoder pin define
#define L_ENC_A 32
#define L_ENC_B 33
#define R_ENC_A 23
#define R_ENC_B 22

hw_timer_t *timer = NULL; // 建立計時器物件
/* 宣告任務變數 */
TaskHandle_t Task_Motorcontrol;

volatile long Encoder_R;
long RightMotor_RPM, Left;
int Vel_Right, Vel_Left;
int PPR = 13;               // Encoder 1圈pulse數


void Motor_control(void * pvParameters){
  Serial.print("Task_Motorcontrol running on core");
  Serial.println(xPortGetCoreID());
  int i;
  for(;;){
    digitalWrite(INA1, LOW);
    digitalWrite(INB1, HIGH);
    digitalWrite(INA2, LOW);
    digitalWrite(INB2, HIGH);
    for(i=80 ; i<=255 ; i++){
      ledcWrite(0, 255);
      ledcWrite(1, 255);
      // Serial.print("Motor PWM: ");
      // Serial.println(i);
      vTaskDelay(200);
    }
  }
}

void setup() {
  Serial.begin(115200);
  setCpuFrequencyMhz(240);

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
  delay(200);

  digitalWrite(INA1, HIGH);
  digitalWrite(INB1, HIGH);
  digitalWrite(PWM1, LOW);
  digitalWrite(INA2, HIGH);
  digitalWrite(INB2, HIGH);
  digitalWrite(PWM2, LOW);
  delay(100);

  ledcSetup(0, 5000, 8);
  ledcSetup(1, 5000, 8);
  ledcAttachPin(PWM1, 0);
  ledcAttachPin(PWM2, 1);

  xTaskCreatePinnedToCore(Motor_control, "Motor Control", 10000, NULL, 1, &Task_Motorcontrol, 1);
  delay(500);

  attachInterrupt(L_ENC_A, READ_ENC_R, CHANGE);
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &EncoderISR, true);
  timerAlarmWrite(timer, 10000, true);
  timerAlarmEnable(timer);
}

void loop() {
  Serial.print("Right motor:");
  Serial.println(Vel_Right);
  vTaskDelay(200);
}
