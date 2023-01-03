/**
 * drives the robot with a certain translational and rotational velocity
 * 
 * @param {float} vx the target x velocity for the robot
 * @param {float} vy the target y velocity for the robot
 * @param {float} dt the change in time since the last update
 */
void drive_robot(float vx, float vy, float w, float dt) {
  // multiply the inverse kinematics matrix by the desired velocities to calculate the individual motor speeds
  float motor_speeds[NUM_DRIVE_MOTORS];
  for (int i = 0; i < NUM_DRIVE_MOTORS; i++) {
    motor_speeds[i] = ik_drive_matrix[i][0] * vx + ik_drive_matrix[i][1] * vy + ik_drive_matrix[i][2] * w;
  }

  // set the drive motor speeds to the calculated inverse kinematics speeds scaled up by the max motor velocity
  left_drive_motor.set_drive_speed(motor_speeds[0] * left_drive_motor.MAX_ANGULAR_VELOCITY, dt);
  right_drive_motor.set_drive_speed(motor_speeds[1] * right_drive_motor.MAX_ANGULAR_VELOCITY, dt);
  back_drive_motor.set_drive_speed(motor_speeds[2] * back_drive_motor.MAX_ANGULAR_VELOCITY, dt);
  
  if (DEBUG) {
    Serial.print(" hi:");
    Serial.print(left_drive_motor.MAX_ANGULAR_VELOCITY + 1);
    Serial.print(" lo:");
    Serial.print(left_drive_motor.MAX_ANGULAR_VELOCITY - 1);
    Serial.println();
  }
}

/**
 * run a drive motor at a specific time-based function, to test the PID controller response to a given signal
 * 
 * @param {int} id the ID of the drive motor to be tested
 * @param {int} mode the type of signal to be given to the drive motor, 0 for sine wave, 1 for step function
 * @param {float} period the time in seconds for each interval of the test signal
 * @param {float} dt the change in time since the last update
 */
void test_drive_motor_pid(int id, int mode, float period, float dt) {
  float t = millis()/1000.0;
  float step = 1.0;
  float peak = left_drive_motor.MAX_ANGULAR_VELOCITY;
  float speed = 0.0;

  if (mode == 0) {
    speed = peak/2.0 * sin(t * 2 * M_PI * 1/period) + peak/2.0;
  } else {
    speed = min(peak, step * ((int) (t * 1/period)));
  }

  drive_motors[id]->set_drive_speed(speed, dt);
}

/**
 * interrupt function to read the encoders of a specific drive motor
 */
template <int j>
void read_drive_motor_encoder() {
  // read the encoder value and update the encoder count
  int encb_val = digitalRead(drive_motors[j]->ENCB);
  if (encb_val > 0) {
    drive_motors[j]->enc_posi--;
  } else {
    drive_motors[j]->enc_posi++;
  }

  // calculate the velocity of the wheel by dividing one update by the time it takes in seconds
  long curr_enc_time_micros = micros();
  float encoder_dt = ((float) (curr_enc_time_micros - drive_motors[j]->prev_enc_time_micros))/1.0e6;
  drive_motors[j]->enc_velocityi = (encb_val > 0 ? -1 : 1)/encoder_dt;
  drive_motors[j]->prev_enc_time_micros = curr_enc_time_micros;
}