#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>

#define M_PI 3.14159265
#define DEG_TO_RAD M_PI/180.0
#define EPSILON 0.00001

// radio
RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00001";
int data[5];

// drive motors
#define NUM_DRIVE_MOTORS 3
#define MAX_DRIVE_MOTOR_PWM 255

#define LEFT_DRIVE_MOTOR 0
#define RIGHT_DRIVE_MOTOR 1
#define BACK_DRIVE_MOTOR 2

long curr_time_micros = 0;
long prev_time_micros = 0;

// rough theory explanation: https://youtu.be/wwQQnSWqB7A
/**
  Define the coordinates system in the robot frame to be the origin at the center of the robot, with
  +x going to the right and +y going up looking down at the origin from the sky. Motor 3 (back motor
  from the robot's perspective) will be the motor along the y axis, motor 1 (left motor) will be the 
  motor in the fourth quadrant, and motor 2 (right motor) will be the motor in the third quadrant.
  Each motor has a positive angular velocity m1, m2, m3 defined as counterclockwise rotation when looking
  at the motor, the shaft extending towards you.

  Let fk_drive_matrix be the matrix where [vx, vy, ωz]^ = fk_drive_matrix [m1, m2, m3]^T
  that is, the forward kinematics matrix multiplied by the vector of 3 motor speeds m1, m2, m3 
  to yield the vector containing the translational and angular velocities of the robot vx, vy, ωz

  The first row of fk_drive_matrix corresponds to the coefficients multiplied by m1, m2, m3 to yield a particular vx.
  Starting with the coefficient for m3 for vx, i.e. the third element in the first row of fk_drive_matrix, consider m3 
  as a positive angular velocity, thus producing a positive force/velocity vector for the robot along the +x direction.
  Motor 3 is fully on the y axis, thus the shaft is oriented 90° away from the +x axis, which orients its produced 
  force/velocity vector along 90° - 90°, which must be perpendicular to how the motor's shaft is oriented. Thus, the 
  horizontal component is cos(90° - 90°). This logic extends to motor 2 and 1, which are oriented 120° and 240° away from 
  motor 1, respectively, and thus have horizontal force/velocity vector components of cos(90° - 90° + 120°) and 
  cos(90° - 90° + 240°), respectively. 

  The second row of fk_drive_matrix corresponds to the coefficients multiplied by m1, m2, m3 to yield a particular vy, 
  which is all the vertical components of the force/velocity vectors produced by the motors. Those can be obtained by 
  taking the sins of the aforementioned angles of the force/velocity vectors.

  The third row of fk_drive_matrix corresponds to the coefficients multiplied by m1, m2, m3 to yield a particular ωz. 
  This row is simpler, since all motors need to be running equally with positive angular velocity to rotate the robot 
  by a positive angular velocity, so all entries are 1.

  Thus, to solve the needed motor speeds m1, m2, m3 given desired angular velocities vx, vy, ωz,
  invert fk_drive_matrix to find ik_drive_matrix where [m1, m2, m3]^T = ik_drive_matrix [vx, vy, ωz]^T.

  Sorry that was wordy, good luck nerds.
*/
float fk_drive_matrix[NUM_DRIVE_MOTORS][NUM_DRIVE_MOTORS] = {{ cos((90 + 2 * 120 - 90) * DEG_TO_RAD), cos((90 + 120 - 90) * DEG_TO_RAD), cos((90 - 90) * DEG_TO_RAD) }, 
                                                             { sin((90 + 2 * 120 - 90) * DEG_TO_RAD), sin((90 + 120 - 90) * DEG_TO_RAD), sin((90 - 90) * DEG_TO_RAD) }, 
                                                             { 1.0, 1.0, 1.0 }};

float ik_drive_matrix[NUM_DRIVE_MOTORS][NUM_DRIVE_MOTORS] = {{ -1.0/3.0, -sqrt(3.0)/3.0, 1.0/3.0 }, 
                                                             { -1.0/3.0, sqrt(3.0)/3.0, 1.0/3.0 }, 
                                                             { 2.0/3.0, 0, 1.0/3.0 }};

class PIDController {
  private:
    float kp;
    float ki;
    float kd;
    
    float error;
    float prev_error;
    float d_error;
    float sum_error;
    
    float setpoint;
    float value;
    float dt;
    float output;

  public:
    PIDController(float kp, float ki, float kd) {
      this->kp = kp;
      this->ki = ki;
      this->kd = kd;
    }

    float get_controller_output(float setpoint, float value, float dt) {
      this->setpoint = setpoint;
      this->value = value;
      this->dt = dt;
      
      this->error = this->setpoint - this->value;
      this->sum_error = this->sum_error + this->error * this->dt;
      this->sum_error = 0;
      this->d_error = (this->error - this->prev_error)/this->dt;
      this->prev_error = this->error;

      this->output = this->kp * this->error + this->ki * this->sum_error + this->kd * this->d_error;

      // hack, pls fix
      // if (abs(this->error) < 10) {
      //   return 0.0;  
      // }

      // Serial.print(" e:");
      // Serial.print(this->error);

      return this->output;
    }
};

// this robot uses Polulu 50:1 200RPM 12V DC Gearmotors with 64 CPR Encoders: https://www.pololu.com/product/4753
class DriveMotor {
  public:
    const int MIN_DRIVE_PWM = 30;
    const int MAX_DRIVE_PWM = 255;
    const float MAX_ANGULAR_VELOCITY = 20.943951;

    int ENCA;
    int ENCB;
    volatile long enc_posi;
    long enc_pos;
    long prev_enc_pos;
    long target_enc_pos;
    volatile float enc_velocityi;
    float enc_velocity;
    volatile long prev_enc_time_micros;

    float v1f;
    float prev_v1;
    float v2f;
    float prev_v2;
    const float af = 0.893;
    const float bf = (1.0 - af)/2.0; 
    
  private:
    int id;
    int EN;
    int IN1;
    int IN2;

    float bal;
    PIDController* pid_controller;

    float pulses_per_rotation;
    float wheel_circumference;
    float target_angular_velocity; // min 0.125, max 3.3 rot/s

  public:
    DriveMotor(int id, int EN, int IN1, int IN2, int ENCA, int ENCB, 
                /*float kp, float ki, float kd,*/float bal, PIDController* pid_controller) {
      this->id = id;
      this->EN = EN;
      this->IN1 = IN1;
      this->IN2 = IN2;
      this->ENCA = ENCA;
      this->ENCB = ENCB;

      this->pid_controller = pid_controller;
      this->bal = bal;

      this->pulses_per_rotation = 64/4 * 50;
      this->wheel_circumference = 0.072 * PI;
      
      pinMode(this->EN, OUTPUT);
      pinMode(this->IN1, OUTPUT);
      pinMode(this->IN2, OUTPUT);
      pinMode(this->ENCA, INPUT);
      pinMode(this->ENCB, INPUT);
    }
    
    void set_drive_speed(float angular_velocity, float dt) {
      this->target_angular_velocity = angular_velocity;
      float v_target = angular_velocity;
      // float v_target = (this->target_angular_velocity * 60.0)/(2 * M_PI);
      // float v_target = (this->target_angular_velocity * this->pulses_per_rotation)/(2 * M_PI);
      // float d_target = (this->target_angular_velocity * dt * this->pulses_per_rotation)/(2 * M_PI);

      // v_target = 100*(sin(micros()/1e6)) + 100;
      // v_target = 0;

      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        this->enc_pos = this->enc_posi;
        this->enc_velocity = this->enc_velocityi;
      }

      float v1 = (this->enc_pos - this->prev_enc_pos)/(dt * this->pulses_per_rotation) * 2 * M_PI;
      float v2 = this->enc_velocity/this->pulses_per_rotation * 2 * M_PI;
      
      this->v1f = this->af * this->v1f + this->bf * v1 + this->bf * this->prev_v1;
      this->prev_v1 = v1;
      this->v2f = this->af * this->v2f + this->bf * v2 + this->bf * this->prev_v2;
      this->prev_v2 = v2;

      float v = (this->v1f < EPSILON) ? this->v1f : this->v2f;
      // float v = (this->enc_pos - this->prev_enc_pos)/(dt * this->pulses_per_rotation) * 2 * M_PI;
      this->prev_enc_pos = this->enc_pos;

      // this->target_enc_pos = (long) ceil(this->target_enc_pos + d_target);
      // this->target_enc_pos = -1600 * 2;

      Serial.print("v_target:");
      Serial.print(v_target), 5;
      // Serial.print(" v1:");
      // Serial.print(v1, 5);
      // Serial.print(" v1f:");
      // Serial.print(this->v1f, 5);
      // Serial.print(" v2:");
      // Serial.print(v2, 5);
      // Serial.print(" v2f:");
      // Serial.print(this->v2f, 5);
      Serial.print(" v:");
      Serial.print(v, 5);
      // Serial.print(" pos:");
      // Serial.print(this->enc_pos);
      // Serial.print(" dt:");
      // Serial.print(dt, 5);
      
      // int motor_pwm = (int) this->pid_controller->get_controller_output(this->target_enc_pos, this->enc_pos, dt);
      int motor_pwm = (int) this->pid_controller->get_controller_output(v_target, v, dt);
      // int motor_pwm = 35;
      // int motor_pwm = 100*(sin(1.0 * M_PI * micros()/1e6)) + 100;
      motor_pwm = constrain(motor_pwm, -255, 255);

    //  Serial.print("T ");
    //  Serial.print(this->id);
    //  Serial.print(" ");
    //  Serial.print(this->target_enc_pos);
    //  Serial.print(" E ");
    //  Serial.print(this->enc_pos);
     Serial.print(" M ");
     Serial.print(motor_pwm);

      Serial.print(" 0:");
      Serial.print(0);
//      Serial.println();

      write_pwm(motor_pwm);
    }

    void write_pwm(int motor_pwm) {
      int mapped_pwm = this->bal * motor_pwm;
      // if (abs(mapped_pwm) < MIN_DRIVE_PWM) mapped_pwm = 0;

      // Serial.print(mapped_pwm);
      // Serial.print(", ");
    
      // control motor direction
      if (mapped_pwm > 0) {
        digitalWrite(this->IN1, HIGH);
        digitalWrite(this->IN2, LOW);
      } else if (mapped_pwm < 0) {
        digitalWrite(this->IN1, LOW);
        digitalWrite(this->IN2, HIGH);
      } else {
        digitalWrite(this->IN1, LOW);
        digitalWrite(this->IN2, LOW);
      }
    
      // set motor speed
      analogWrite(this->EN, abs(mapped_pwm));
    }
};

PIDController left_drive_motor_pid_controller = PIDController(100.0, 50.0, 5.0);
PIDController right_drive_motor_pid_controller = PIDController(2.5, 0.1, 0.28);
PIDController back_drive_motor_pid_controller = PIDController(2.5, 0.1, 0.28);

DriveMotor left_drive_motor = DriveMotor(LEFT_DRIVE_MOTOR, 10, 31, 30, 18, 19, 1.0, &left_drive_motor_pid_controller);
DriveMotor right_drive_motor = DriveMotor(RIGHT_DRIVE_MOTOR, 11, 33, 32, 20, 21, 1.0, &right_drive_motor_pid_controller);
DriveMotor back_drive_motor = DriveMotor(BACK_DRIVE_MOTOR, 5, 23, 22, 2, 3, 1.0, &back_drive_motor_pid_controller);
DriveMotor* drive_motors[NUM_DRIVE_MOTORS] = { &left_drive_motor, &right_drive_motor, &back_drive_motor };