void EncoderISR(){
  Vel_Right = Encoder_R;
  RightMotor_RPM = ((Vel_Right/PPR)*(1000/10) * 60);
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