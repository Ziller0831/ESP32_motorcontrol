#include <ros.h>
#include <std_msgs/Int16.h>

#define L_ENC_A 35
#define L_ENC_B 34
#define R_ENC_A 23
#define R_ENC_B 22

ros::NodeHandle nh;

std_msgs::Int16 right_wheel_tick;
ros::Publisher rightPub("right_ENC", &right_wheel_tick);

std_msgs::Int16 left_wheel_tick;
ros::Publisher leftPub("left_ENC", &left_wheel_tick);

portMUX_TYPE mux0 = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE mux1 = portMUX_INITIALIZER_UNLOCKED;

hw_timer_t * timer_R = NULL;
hw_timer_t * timer_L = NULL;

volatile long Encoder_R;
volatile long Encoder_L;
double LeftMotor_RPM;
double RightMotor_RPM;
int RightENC_Vel;
int LeftENC_Vel;
const int interval = 100;
long previousMillis = 0;
long currentMillis = 0;

void IRAM_ATTR R_EncoderISR(){
    portENTER_CRITICAL(&mux0);
    // RightENC_Vel = Encoder_R;
    // right_wheel_speed.data = RightENC_Vel * 0.065;
    right_wheel_tick.data = Encoder_R;
    Encoder_R = 0;
    portEXIT_CRITICAL(&mux0);
}

void IRAM_ATTR L_EncoderISR(){
    portENTER_CRITICAL(&mux1);
    // LeftENC_Vel = Encoder_L;
    // left_wheel_speed.data = LeftENC_Vel * 0.065;
    left_wheel_tick.data = Encoder_L;
        Encoder_L = 0;
    portEXIT_CRITICAL(&mux1);
}
void READ_ENC_Right(){
  if(digitalRead(R_ENC_A) == LOW){    // 正轉
        if(digitalRead(R_ENC_B) == LOW)
            Encoder_R--;
        else
            Encoder_R++;
  } else{                             // 反轉
        if(digitalRead(R_ENC_B) == LOW)
            Encoder_R++;
        else
            Encoder_R--;
    }
}

void READ_ENC_Left(){
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

void Controller_Pinsetup()
{
    pinMode(L_ENC_A, INPUT);
    pinMode(L_ENC_B, INPUT);
    pinMode(R_ENC_A, INPUT);
    pinMode(L_ENC_B, INPUT);
}

void setup(){
    Serial.begin(115200);
    setCpuFrequencyMhz(240);

    Controller_Pinsetup();

    attachInterrupt(R_ENC_A, READ_ENC_Right, CHANGE);
    timer_R = timerBegin(0, 80, true); // timer為80MHz的頻率，因此需要分頻成1MHz，計時器的step為1us(1*10^-6Hz)

    timerAttachInterrupt(timer_R, &R_EncoderISR, true);
    timerAlarmWrite(timer_R, 10000, true); // 1000 = 1 ms
    timerAlarmEnable(timer_R);

    attachInterrupt(L_ENC_A, READ_ENC_Left, CHANGE);
    timer_L = timerBegin(1, 80, true);

    timerAttachInterrupt(timer_L, &L_EncoderISR, true);
    timerAlarmWrite(timer_L, 10000, true);
    timerAlarmEnable(timer_L);

    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.advertise(rightPub);
    nh.advertise(leftPub);
}

void loop(){
//   Serial.print("Right motor:");
//   Serial.println(RightMotor_RPM);
//   Serial.print("Left motor:");
//   Serial.println(LeftMotor_RPM);

    // rightPub.publish(&right_wheel_speed);
    // leftPub.publish(&left_wheel_speed);
    currentMillis = millis();

    // If 100ms have passed, print the number of ticks
    if (currentMillis - previousMillis > interval)
    {

        previousMillis = currentMillis;

        rightPub.publish(&right_wheel_tick);
        leftPub.publish(&left_wheel_tick);
        nh.spinOnce();
    }
}