#define PWM1 13
#define INA1 12
#define INB1 14
#define Encoder_A 32
#define Encoder_B 33

hw_timer_t *timer = NULL; // 建立計時器物件

volatile long Encoder_R;
int Vel_Right;
unsigned long RightMotor_Speed;
void EncoderISR(){
  sei();
  Vel_Right = Encoder_R;
  RightMotor_Speed = Vel_Right / 13 * 60;
  Encoder_R = 0;
}

void READ_ENCODER_R(){
  if(digitalRead(Encoder_A) == LOW){
    if(digitalRead(Encoder_B) == LOW)
      Encoder_R++;
    else
      Encoder_R--;
  } else{
    if(digitalRead(Encoder_B) == LOW)
      Encoder_R--;
    else
      Encoder_R++;
  }
}

void setup() {
  Serial.begin(115200);
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

  attachInterrupt(Encoder_A, READ_ENCODER_R, CHANGE);
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &EncoderISR, true);
  timerAlarmWrite(timer, 5000, true);
  timerAlarmEnable(timer);
}

void loop() {
  digitalWrite(INA1, LOW);
  digitalWrite(INB1, HIGH);
  for(int i = 80 ; i <= 255 ; i++){
    ledcWrite(0, i);
    Serial.print("Right motor:");
    Serial.println(RightMotor_Speed);
  }
}
