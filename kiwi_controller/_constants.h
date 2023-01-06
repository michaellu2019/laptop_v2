#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>

#define M_PI 3.14159265
#define DEG_TO_RAD M_PI/180.0
#define EPSILON 0.00001

#define DEBUG 0

// radio
RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00001";
int data[5];

// radio inputs
int input_x;
int input_y;
float drive_xf;
float drive_x;
float prev_drive_x;
float drive_yf;
float drive_y;
float prev_drive_y;
const float af = 0.893;
const float bf = (1.0 - af)/2.0; 

// drive motors
#define NUM_DRIVE_MOTORS 3
#define MAX_DRIVE_MOTOR_PWM 255

#define LEFT_DRIVE_MOTOR 0
#define RIGHT_DRIVE_MOTOR 1
#define BACK_DRIVE_MOTOR 2

// global timer
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

// PID controller class
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
    /**
     * constructor for the PIDController class
     *
     * @class PIDController
     * @constructor 
     *
     * @param {float} kp the proportional gain constant for the PID controller
     * @param {float} ki the integral gain constant for the PID controller
     * @param {float} kd the derivative gain constant for the PID controller
     */
    PIDController(float kp, float ki, float kd) {
      this->kp = kp;
      this->ki = ki;
      this->kd = kd;
    }

    /**
     * calculates the output of the PID controller
     * 
     * @method get_controller_output
     * 
     * @param {float} setpoint the target value for the controller
     * @param {float} value the current measured value for the controller
     * @param {float} dt the change in time since the last controller update
     * 
     * @return {float} the controller output value
     */
    float get_controller_output(float setpoint, float value, float dt) {
      this->setpoint = setpoint;
      this->value = value;
      this->dt = dt;
      
      // calculate all error values
      this->error = this->setpoint - this->value;
      this->sum_error = this->sum_error + this->error * this->dt;
      this->d_error = (this->error - this->prev_error)/this->dt;
      this->prev_error = this->error;

      if (DEBUG) {
        Serial.print(" e:");
        Serial.print(this->error);
      }

      // calculate and return output
      this->output = this->kp * this->error + this->ki * this->sum_error + this->kd * this->d_error;

      return this->output;
    }

    /**
     * resets the integrated error of the controller, preventing integrator windup https://en.wikipedia.org/wiki/Integral_windup
     * 
     * @method reset_sum_error
     */
    void reset_sum_error() {
      this->sum_error = 0.0;
    }
};

// drive motor controller class
// the drive motors are Polulu 50:1 200RPM 12V DC Gearmotors with 64 CPR Encoders: https://www.pololu.com/product/4753
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
    
  private:
    int id;
    int EN;
    int IN1;
    int IN2;

    float bal;
    PIDController* pid_controller;

    float pulses_per_rotation = 64/4 * 50;
    float wheel_circumference = 0.072 * PI;

    const float cutoff_target_v = 0.5;
    float target_v;
    float v1f;
    float prev_v1;
    float v2f;
    float prev_v2;
    const float af = 0.893;
    const float bf = (1.0 - af)/2.0; 

  public:
    /**
     * constructor for the DriveMotor class
     *
     * @class DriveMotor
     * @constructor 
     *
     * @param {int} id integer ID number for the drive motor
     * @param {int} EN pin for analogWrite to control PWM duty cycle to motor
     * @param {int} IN1 pin to control the direction the motor spins
     * @param {int} IN2 pin to control the direction the motor spins
     * @param {int} ENCA pin to read the encoder A signal of the motor
     * @param {int} ENCB pin to read the encoder B signal of the motor
     * @param {float} bal coefficient to adjust motor power
     * @param {PIDController*} pid_controller controller to adjust the drive motor velocity
     */
    DriveMotor(int id, int EN, int IN1, int IN2, int ENCA, int ENCB, float bal, PIDController* pid_controller) {
      this->id = id;
      this->EN = EN;
      this->IN1 = IN1;
      this->IN2 = IN2;
      this->ENCA = ENCA;
      this->ENCB = ENCB;

      this->bal = bal;
      this->pid_controller = pid_controller;
      
      pinMode(this->EN, OUTPUT);
      pinMode(this->IN1, OUTPUT);
      pinMode(this->IN2, OUTPUT);
      pinMode(this->ENCA, INPUT);
      pinMode(this->ENCB, INPUT);
    }
    
    /**
     * uses the drive motor's PID controller to adjust the motor's speed to the desired value
     * 
     * @method set_drive_speed
     * 
     * @param {float} target_v the desired angular velocity for the drive motor, in rad/s
     * @param {float} dt the change in time since the last controller update
     * 
     * @return {float} the controller output value
     */
    void set_drive_speed(float target_v, float dt) {
      this->target_v = (abs(target_v) < this->cutoff_target_v) ? 0 : target_v;

      // read encoder positions and velocities
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        this->enc_pos = this->enc_posi;
        this->enc_velocity = this->enc_velocityi;

        Serial.print(" e:");
        Serial.print(this->enc_pos);
        Serial.print(" ei:");
        Serial.print(this->enc_posi);
      }

      // calculate measured wheel velocities in rad/s
      // v1 involves counting the number of encoder ticks and dividing by time
      // v2 involves dividing the time it takes for one encoder tick to occur
      float v1 = (this->enc_pos - this->prev_enc_pos)/(dt * this->pulses_per_rotation) * 2 * M_PI;
      float v2 = this->enc_velocity/this->pulses_per_rotation * 2 * M_PI;
      this->prev_enc_pos = this->enc_pos;

      Serial.print(" v1:");
      Serial.print(v1);
      Serial.print(" v2:");
      Serial.print(v2);
      Serial.println();
      
      // run a low pass filter on both v1 and v2 values to smooth out signal
      this->v1f = this->af * this->v1f + this->bf * v1 + this->bf * this->prev_v1;
      this->prev_v1 = v1;
      this->v2f = this->af * this->v2f + this->bf * v2 + this->bf * this->prev_v2;
      this->prev_v2 = v2;

      // only v1 will accurately measure a zero velocity, v2 can never be zero
      // set v to v2 unless v1 measures a zero velocity
      float v = (abs(this->v1f) < EPSILON) ? this->v1f : this->v2f;
      // if the target velocity is zero, force the error to 0 and reset the PID controller's integrated error to prevent integrator windup
      if (abs(this->target_v) < EPSILON) {
        v = 0.0;
        this->pid_controller->reset_sum_error();
      }
      
      // get motor PWM from PID controller and constrain it within max pwm values 
      int motor_pwm = (int) this->pid_controller->get_controller_output(this->target_v, v, dt);
      motor_pwm = constrain(motor_pwm, -MAX_DRIVE_PWM, MAX_DRIVE_PWM);
      write_pwm(motor_pwm);

      if (DEBUG) {
        Serial.print(" vt:");
        Serial.print(this->target_v), 5;
        Serial.print(" v:");
        Serial.print(v, 5);
        Serial.print(" pwm:");
        Serial.print(motor_pwm);
      }
    }
    
    /**
     * write a PWM value to the drive motor
     * 
     * @method write_pwm
     * 
     * @param {int} write a pwm duty cycle value between -255 and 255 to the drive motor
     */
    void write_pwm(int motor_pwm) {
      int mapped_pwm = this->bal * motor_pwm;
    
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

// initialize PID controllers
PIDController left_drive_motor_pid_controller = PIDController(100.0, 15.0, 5.0);
PIDController right_drive_motor_pid_controller = PIDController(100.0, 15.0, 5.0);
PIDController back_drive_motor_pid_controller = PIDController(100.0, 15.0, 5.0);

// initialize drive motors
DriveMotor left_drive_motor = DriveMotor(LEFT_DRIVE_MOTOR, 5, 30, 31, 18, 19, 1.0, &left_drive_motor_pid_controller);
DriveMotor right_drive_motor = DriveMotor(RIGHT_DRIVE_MOTOR, 6, 26, 27, 20, 21, 1.0, &right_drive_motor_pid_controller);
DriveMotor back_drive_motor = DriveMotor(BACK_DRIVE_MOTOR, 9, 22, 23, 2, 3, 1.0, &back_drive_motor_pid_controller);
DriveMotor* drive_motors[NUM_DRIVE_MOTORS] = { &left_drive_motor, &right_drive_motor, &back_drive_motor };