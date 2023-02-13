#define PWM1 13
#define INA1 12
#define INB1 14
#define Encoder_A 32
#define Encoder_B 33

bool EA_state;
bool EA_Laststate;
bool EB_state;
bool EB_Laststate;
volatile double counter = 0;
volatile double circounter = 0;

void Encode(){
  EA_state = digitalRead(Encoder_A);
  if(EA_state != EA_Laststate){
    EB_state = digitalRead(Encoder_B);
    if(EB_state != EA_state)
      counter += 1;
    else
      counter -= 1;
  }
  EA_Laststate = EA_state;
}

void setup() {
  Serial.begin(115200);
  pinMode(PWM1, OUTPUT);
  pinMode(INA1, OUTPUT);
  pinMode(INB1, OUTPUT);
  pinMode(Encoder_A, INPUT);
  pinMode(Encoder_B, INPUT);
  attachInterrupt(Encoder_A, Encode, CHANGE);

  digitalWrite(INA1, HIGH);
  digitalWrite(INB1, HIGH);
  digitalWrite(PWM1, LOW);
  delay(100);

  ledcSetup(0, 5000, 8);

  ledcAttachPin(PWM1, 0);

  EA_Laststate = digitalRead(EA_state);
}

void loop() {
  digitalWrite(INA1, LOW);
  digitalWrite(INB1, HIGH);
  ledcWrite(0, 255);

  int HTime = pulseIn(Encoder_A, HIGH);
  int LTime = pulseIn(Encoder_A, LOW);
  float freq = 1 / (HTime + LTime);
  Serial.println(HTime);

  Serial.print("Position: "); //透過serial印出字串 Position:
  Serial.print(circounter); //透過serial印出 counter 值
  Serial.print("rev,");
  Serial.print(counter*360/71000); //透過serial印出 counter 值
  Serial.println("deg.");
}
