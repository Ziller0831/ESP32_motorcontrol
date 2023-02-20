void EncoderISR(){
  Vel_Right = Encoder_R;
  RightMotor_RPM = ((Vel_Right/PPR)*(1000/10) * 60);
  Encoder_R = 0;
}

void READ_ENC_R(){
  if(digitalRead(L_ENC_A) == LOW){
    if(digitalRead(L_ENC_B) == LOW)
      Encoder_R++;
    else
      Encoder_R--;
  } else{
    if(digitalRead(L_ENC_B) == LOW)
      Encoder_R--;
    else
      Encoder_R++;
  }
}
void READ_ENC_L(){
  
}