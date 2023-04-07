#include <ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>

/* Left_motor control pin define */
#define PWM1 18
#define INA1 19
#define INB1 21
/* Right_motor control pin define */
#define PWM2 25
#define INA2 33
#define INB2 32
/* Encoder pin define */
#define L_ENC_A 35
#define L_ENC_B 34
#define R_ENC_A 23
#define R_ENC_B 22

ros::NodeHandle nh;

std_msgs::Int16 right_wheel_tick;
ros::Publisher rightPub("right_ENC", &right_wheel_tick);

std_msgs::Int16 left_wheel_tick;
ros::Publisher leftPub("left_ENC", &left_wheel_tick);

std_msgs::Int16 test_message;
ros::Publisher testPub("test", &test_message);

portMUX_TYPE mux0 = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE mux1 = portMUX_INITIALIZER_UNLOCKED;

hw_timer_t *timer_R = NULL;
hw_timer_t *timer_L = NULL;
hw_timer_t *timer2publish = NULL;

TaskHandle_t Task_Motorcontrol;
TaskHandle_t Task_MessageReceive;

// Encoder Variables
volatile long Encoder_R;
volatile long Encoder_L;
int RightENC_Vel;
int LeftENC_Vel;

// Motor variables
double Left_PWM_Req = 0;
double Right_PWM_Req = 0;
double Left_MT_Vel;
double Right_MT_Vel;

// PID parameters
const int Kp = 1;

const int b = 20;   // 輪子到車體中心距
const int PWM_turn = 80; // 轉彎PWM值
const int Drift_muliplier = 1;  // Correction muliplier for drift. from the experimentation
const int PWM_MIN = 80; // about ?? m/s
const int PWM_MAX = 200; // about ?? m/s
const int PWM_Increment = 1; // PWM增加量
double lastCmdVelRecivTime = 0; // 上次接收命令的時間


/* ----- Encoder ----------------------------*/
void IRAM_ATTR R_EncoderISR()
{
    portENTER_CRITICAL(&mux0);
    // RightENC_Vel = Encoder_R;
    // right_wheel_speed.data = RightENC_Vel * 0.065;
    right_wheel_tick.data = Encoder_R;
    Encoder_R = 0;
    portEXIT_CRITICAL(&mux0);
}

void IRAM_ATTR L_EncoderISR()
{
    portENTER_CRITICAL(&mux1);
    // LeftENC_Vel = Encoder_L;
    // left_wheel_speed.data = LeftENC_Vel * 0.065;
    left_wheel_tick.data = Encoder_L;
    Encoder_L = 0;
    portEXIT_CRITICAL(&mux1);
}

void IRAM_ATTR Publish()
{
    rightPub.publish(&right_wheel_tick);
    leftPub.publish(&left_wheel_tick);
    // if (test_message.data == nullptr)
    //     test_message.data = 0;
    testPub.publish(&test_message);
}

void READ_ENC_Right()
{
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

void READ_ENC_Left()
{
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

/* ----- Motor control -------------------------*/
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

void ROS_messageRecivTask(void *pvParam)
{
    while(1){
        nh.spinOnce();
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

void cmd_velocity_receiv(const geometry_msgs::Twist &cmdVel)
{
    lastCmdVelRecivTime = (millis() / 1000);
    Left_PWM_Req = Kp * cmdVel.linear.x + b;
    Right_PWM_Req = Kp * cmdVel.linear.x + b;
    test_message.data = cmdVel.linear.x;
    if (cmdVel.angular.z != 0.0){
        if (cmdVel.angular.z > 0.0){
            Left_PWM_Req = -PWM_turn;
            Right_PWM_Req = PWM_turn;
        } else{
            Left_PWM_Req = PWM_turn;
            Right_PWM_Req = -PWM_turn;
        }
    } else{
        static double PrevDiff = 0;     // 前一次速差
        static double Dub_prevDiff = 0; // 前前次速差
        double currDiff = Left_MT_Vel - Right_MT_Vel;
        double avgDiff = (PrevDiff + Dub_prevDiff + currDiff) / 3;
        Dub_prevDiff = PrevDiff;
        PrevDiff = currDiff;
        Left_PWM_Req -= (int)(avgDiff * Drift_muliplier);
        Right_PWM_Req += (int)(avgDiff * Drift_muliplier);
    }
    if (abs(Left_PWM_Req) < PWM_MIN)
        Left_PWM_Req = 0;
    if (abs(Right_PWM_Req) < PWM_MIN)
        Right_PWM_Req = 0;
    
    xTaskCreatePinnedToCore(
        Motor_control,
        "Core 0: Motor control",
        1024,
        NULL,
        10,
        &Task_Motorcontrol,
        0
    );

}

void Motor_control(void *pvParam)
{
    static int L_PWM_out = 0;
    static int R_PWM_out = 0;

    
    if ((Left_PWM_Req * Left_MT_Vel < 0 && L_PWM_out != 0) ||
        (Right_PWM_Req * Right_MT_Vel < 0 && R_PWM_out != 0)){
        Left_PWM_Req = 0;
        Right_PWM_Req = 0;
    }

    if (Left_PWM_Req > 0){
        digitalWrite(INA1, HIGH);
        digitalWrite(INB1, LOW);
    } else if (Left_PWM_Req < 0){
        digitalWrite(INA1, LOW);
        digitalWrite(INB1, HIGH);
    } else if (Right_PWM_Req == 0 && L_PWM_out == 0){
        digitalWrite(INA1, LOW);
        digitalWrite(INB1, LOW);
    } else{
        digitalWrite(INA1, LOW);
        digitalWrite(INB1, LOW);
    }

    if (Right_PWM_Req > 0){
        digitalWrite(INA2, HIGH);
        digitalWrite(INB2, LOW);
    } else if (Right_PWM_Req < 0){
        digitalWrite(INA2, LOW);
        digitalWrite(INB2, HIGH);
    } else if (Right_PWM_Req == 0 && R_PWM_out == 0){
        digitalWrite(INA2, LOW);
        digitalWrite(INB2, LOW);
    } else{
        digitalWrite(INA2, LOW);
        digitalWrite(INB2, LOW);
    }

    if (Left_PWM_Req != 0 && Left_MT_Vel == 0)
        Left_PWM_Req *= 1.5;
    if (Right_PWM_Req != 0 && Right_MT_Vel == 0)
        ight_PWM_Req *= 1.5;

    if (abs(Left_PWM_Req) > L_PWM_out)
        L_PWM_out += PWM_Increment;
    else if (abs(Left_PWM_Req) < L_PWM_out)
        L_PWM_out -= PWM_Increment;

    if (abs(Right_PWM_Req) > R_PWM_out)
        R_PWM_out += PWM_Increment;
    else if (abs(Right_PWM_Req) < L_PWM_out)
        R_PWM_out -= PWM_Increment;

    L_PWM_out = (L_PWM_out > PWM_MAX) ? PWM_MAX : L_PWM_out;
    R_PWM_out = (R_PWM_out > PWM_MAX) ? PWM_MAX : R_PWM_out;

    L_PWM_out = (L_PWM_out < 0) ? 0 : L_PWM_out;
    R_PWM_out = (R_PWM_out < 0) ? 0 : R_PWM_out;

    ledcWrite(0, L_PWM_out);
    ledcWrite(1, R_PWM_out);
    vTaskDelay(1000/portTICK_PERIOD_MS);

    vTaskDelete(NULL);
}

/* 
void SimpleMotor_control(void *pvPara){
    // int i;
    while(1){
        digitalWrite(INA1, LOW);
        digitalWrite(INB1, HIGH);
        digitalWrite(INA2, LOW);
        digitalWrite(INB2, HIGH);
        // for (i = 80; i <= 255; i++){
            // for(i=80 ; i<=255 ; i++){
            ledcWrite(0, 255);
            ledcWrite(1, 255);
            // Serial.print("Motor PWM: ");
            // Serial.println(i);
            vTaskDelay(200);
        // }
        // }
    }
}
*/

ros::Subscriber<geometry_msgs::Twist> subCmdVel("cmd_vel", &cmd_velocity_receiv);

void setup()
{
    Serial.begin(57600);
    setCpuFrequencyMhz(240);

    Controller_Pinsetup();

    xTaskCreatePinnedToCore(
        ROS_messageRecivTask,
        "Core 0: ROS message recieve",
        10000,
        NULL,
        10,
        &Task_MessageReceive,
        0
    );


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

    timer2publish = timerBegin(2, 80, true);
    timerAttachInterrupt(timer2publish, &Publish, true);
    timerAlarmWrite(timer2publish, 10000, true);
    timerAlarmEnable(timer2publish);

    // nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.advertise(rightPub);
    nh.advertise(leftPub);
    nh.advertise(testPub);
    nh.subscribe(subCmdVel);
}

void loop(){
}
