#define PWM1 13
#define INA1 12
#define INB1 14
#define PWM2
#define INA2
#define INB2
#define Encoder_A 32
#define Encoder_B 33

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
    // for(i=80 ; i<=255 ; i++){
      ledcWrite(0, 255);
      // Serial.print("Motor PWM: ");
      // Serial.println(i);
      vTaskDelay(200);
    // }
  }
}

void setup() {
  Serial.begin(115200);
  setCpuFrequencyMhz(240);

  pinMode(PWM1, OUTPUT);
  pinMode(INA1, OUTPUT);
  pinMode(INB1, OUTPUT);
  pinMode(Encoder_A, INPUT);
  pinMode(Encoder_B, INPUT);
  delay(200);

  digitalWrite(INA1, HIGH);
  digitalWrite(INB1, HIGH);
  digitalWrite(PWM1, LOW);
  delay(100);
  ledcSetup(0, 5000, 8);
  ledcAttachPin(PWM1, 0);

  xTaskCreatePinnedToCore(Motor_control, "Motor Control", 10000, NULL, 1, &Task_Motorcontrol, 1);
  delay(500);

  attachInterrupt(Encoder_A, READ_ENCODER_R, CHANGE);
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
