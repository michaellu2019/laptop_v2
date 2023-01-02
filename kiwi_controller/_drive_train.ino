void drive_robot(float vx, float vy, float w, float dt) {
  float motor_speeds[NUM_DRIVE_MOTORS];
  for (int i = 0; i < NUM_DRIVE_MOTORS; i++) {
    motor_speeds[i] = ik_drive_matrix[i][0] * vx + ik_drive_matrix[i][1] * vy + ik_drive_matrix[i][2] * w;
  }

/*
  Serial.print(vx);
  Serial.print(", ");
  Serial.print(vy);
  Serial.print(" --> ");
  Serial.print(motor_speeds[0]);
  // Serial.print(", ");
  // Serial.print(motor_speeds[1]);
  // Serial.print(", ");
  // Serial.print(motor_speeds[2]);
  Serial.print(" --> ");
  Serial.print(motor_speeds[0] * MAX_DRIVE_MOTOR_PWM);
  // Serial.print(", ");
  // Serial.print(motor_speeds[1] * MAX_DRIVE_MOTOR_PWM);
  // Serial.print(", ");
  // Serial.print(motor_speeds[2] * MAX_DRIVE_MOTOR_PWM);
  Serial.print(" --> ");
  Serial.print(motor_speeds[0] * left_drive_motor.MAX_ANGULAR_VELOCITY);
  Serial.print(", ");
  Serial.print(motor_speeds[1] * right_drive_motor.MAX_ANGULAR_VELOCITY);
  Serial.print(", ");
  Serial.print(motor_speeds[2] * back_drive_motor.MAX_ANGULAR_VELOCITY);
  Serial.print(" ");
*/
  // left_drive_motor.write_pwm(motor_speeds[0] * MAX_DRIVE_MOTOR_PWM);
  // right_drive_motor.write_pwm(motor_speeds[1] * MAX_DRIVE_MOTOR_PWM);
  // back_drive_motor.write_pwm(motor_speeds[2] * MAX_DRIVE_MOTOR_PWM);

  left_drive_motor.set_drive_speed(motor_speeds[0] * left_drive_motor.MAX_ANGULAR_VELOCITY, dt);
  right_drive_motor.set_drive_speed(motor_speeds[1] * right_drive_motor.MAX_ANGULAR_VELOCITY, dt);
  back_drive_motor.set_drive_speed(motor_speeds[2] * back_drive_motor.MAX_ANGULAR_VELOCITY, dt);

  // float t = millis()/1000.0;
  // float period = 3.0;
  // float step = 1.0;
  // float peak = left_drive_motor.MAX_ANGULAR_VELOCITY;
  // float speed = min(peak, step * ((int) (t * 1/period)));

  // float speed = peak/2.0 * sin(t * 2 * M_PI * 1/period) + peak/2.0;

  // float speed = min(2 * M_PI * 30.0/10, step * t);

  // left_drive_motor.set_drive_speed(speed, dt);
  // right_drive_motor.set_drive_speed(speed, dt);
  // back_drive_motor.set_drive_speed(speed, dt);
  
  Serial.println();
}

template <int j>
void read_drive_motor_encoder() {
  int encb_val = digitalRead(drive_motors[j]->ENCB);
  if (encb_val > 0) {
    drive_motors[j]->enc_posi--;
  } else {
    drive_motors[j]->enc_posi++;
  }

  long curr_enc_time_micros = micros();
  float encoder_dt = ((float) (curr_enc_time_micros - drive_motors[j]->prev_enc_time_micros))/1.0e6;
  drive_motors[j]->enc_velocityi = (encb_val > 0 ? -1 : 1)/encoder_dt;
  drive_motors[j]->prev_enc_time_micros = curr_enc_time_micros;

//  Serial.println(drive_motors[j]->enc_posi);
//
//  if (j == LEFT_DRIVE_MOTOR) {
//    left_drive_motor.enc_posi = drive_motors[j]->enc_posi;
//  } else if (j == RIGHT_DRIVE_MOTOR) {
//    right_drive_motor.enc_posi = drive_motors[j]->enc_posi;
//  } 
}